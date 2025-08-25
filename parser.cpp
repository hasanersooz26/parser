// gps_parser_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <tuna_interfaces/msg/gps_data.hpp>
#include <tuna_interfaces/msg/imu_data.hpp>
#include <tuna_interfaces/msg/lidar_data.hpp>
#include <tuna_interfaces/msg/velocity.hpp>
#include <tuna_interfaces/msg/pid_params.hpp>
#include <tuna_interfaces/msg/mission_point.hpp>
#include <tuna_interfaces/msg/mission_start.hpp>
#include <tuna_interfaces/msg/control_mode.hpp>
#include <tuna_interfaces/msg/sensor_control.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <cctype>
#include <cerrno>
#include <algorithm>

// I2C / math
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cmath>

using namespace std::chrono_literals;
using std::string;

#define MAGENTA "\033[35m"
#define RESET   "\033[0m"

// ---------- helpers
static inline double wrapDeg(double a) {
  double r = std::fmod(a, 360.0);
  if (r < 0) r += 360.0;
  return r;
}
static inline double wrapDegDiff(double diff) {
  diff = std::fmod(diff + 180.0, 360.0);
  if (diff < 0) diff += 360.0;
  return diff - 180.0;
}

// ---------- simple 1D Kalman for heading
struct HeadingKalman {
  double x = 0.0;
  double P = 10.0;
  double Q = 1.0;
  double R = 9.0;
  void setQR(double q, double r) { Q = q; R = r; }
  double update(double z) {
    P += Q;
    double y = wrapDegDiff(z - x);
    double S = P + R;
    double K = P / S;
    x = wrapDeg(x + K * y);
    P = (1.0 - K) * P;
    return x;
  }
};

// ---------- sign-magnitude (IMU 03 formatı)
static std::string packSignMag4(double v) {
  int sign = (v >= 0.0) ? 1 : 0; // 1: pozitif, 0: negatif
  int mag  = (int)std::lround(std::fabs(v));
  if (mag > 999) mag = 999;
  std::ostringstream oss;
  oss << sign << std::setw(3) << std::setfill('0') << mag;
  return oss.str();
}

// ---------- I2C helpers
static int openI2C(int bus) {
  char path[32];
  std::snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
  return ::open(path, O_RDWR);
}
static bool i2cWriteByte(int fd, uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  struct i2c_msg msg;
  msg.addr  = addr;
  msg.flags = 0;
  msg.len   = 2;
  msg.buf   = buf;
  struct i2c_rdwr_ioctl_data data;
  data.msgs  = &msg;
  data.nmsgs = 1;
  return ioctl(fd, I2C_RDWR, &data) >= 0;
}
static bool i2cReadBytes(int fd, uint8_t addr, uint8_t reg, uint8_t *out, size_t len) {
  struct i2c_msg msgs[2];
  msgs[0].addr  = addr; msgs[0].flags = 0; msgs[0].len = 1; msgs[0].buf = &reg;
  msgs[1].addr  = addr; msgs[1].flags = I2C_M_RD; msgs[1].len = static_cast<__u16>(len); msgs[1].buf = out;
  struct i2c_rdwr_ioctl_data data;
  data.msgs  = msgs;
  data.nmsgs = 2;
  return ioctl(fd, I2C_RDWR, &data) >= 0;
}

class GpsParserNode : public rclcpp::Node {
public:
  GpsParserNode()
  : Node("gps_parser_node"),
    serial_fd_(-1),
    stop_thread_(false),
    baud_rate_(9600),
    last_pwm_send_time_(std::chrono::steady_clock::now())
  {
    // --- params
    this->declare_parameter<int>("serial_retry_delay_ms", 2000);
    this->declare_parameter<std::string>("serial_port", std::string(""));
    serial_retry_delay_ms_ = this->get_parameter("serial_retry_delay_ms").as_int();
    explicit_serial_port_  = this->get_parameter("serial_port").as_string();

    // I2C / fusion / calibration params
    this->declare_parameter<bool>("i2c_enable", true);
    this->declare_parameter<int>("mpu_bus", 1);
    this->declare_parameter<int>("qmc_bus", 8);
    this->declare_parameter<int>("calib_duration_sec", 30);
    this->declare_parameter<bool>("gyro_calibrate_on_boot", true);
    this->declare_parameter<bool>("mag_calibrate_on_boot", true);
    this->declare_parameter<double>("declination_deg", 0.0);
    this->declare_parameter<double>("cf_alpha", 0.98);
    this->declare_parameter<double>("slope_alpha", 0.96);
    this->declare_parameter<double>("lpf_alpha_acc", 0.2);
    this->declare_parameter<double>("lpf_alpha_mag", 0.2);
    this->declare_parameter<bool>("softiron_enable", true);
    this->declare_parameter<double>("kalman_q", 1.0);
    this->declare_parameter<double>("kalman_r", 9.0);
    this->declare_parameter<double>("slope_threshold", 0.1);
    this->declare_parameter<double>("max_slope", 45.0);
    this->declare_parameter<bool>("mirror_i2c_as_serial03", true);
    this->declare_parameter<bool>("emit_uart_serial03", false);

    i2c_enable_   = this->get_parameter("i2c_enable").as_bool();
    mpu_bus_      = this->get_parameter("mpu_bus").as_int();
    qmc_bus_      = this->get_parameter("qmc_bus").as_int();
    calib_secs_   = this->get_parameter("calib_duration_sec").as_int();
    gyro_cal_boot_= this->get_parameter("gyro_calibrate_on_boot").as_bool();
    mag_cal_boot_ = this->get_parameter("mag_calibrate_on_boot").as_bool();
    declination_deg_ = this->get_parameter("declination_deg").as_double();
    cf_alpha_        = this->get_parameter("cf_alpha").as_double();
    slope_alpha_     = this->get_parameter("slope_alpha").as_double();
    lpf_alpha_acc_   = this->get_parameter("lpf_alpha_acc").as_double();
    lpf_alpha_mag_   = this->get_parameter("lpf_alpha_mag").as_double();
    softiron_enable_ = this->get_parameter("softiron_enable").as_bool();
    slope_threshold_ = this->get_parameter("slope_threshold").as_double();
    max_slope_       = this->get_parameter("max_slope").as_double();
    mirror_i2c_as_serial03_ = this->get_parameter("mirror_i2c_as_serial03").as_bool();
    emit_uart_serial03_     = this->get_parameter("emit_uart_serial03").as_bool();
    kalman_.setQR(this->get_parameter("kalman_q").as_double(),
                  this->get_parameter("kalman_r").as_double());

    RCLCPP_INFO(this->get_logger(),
      "Node starting. Baud=%d, explicit='%s', i2c_enable=%d (mpu_bus=%d, qmc_bus=%d)",
      baud_rate_, explicit_serial_port_.c_str(), (int)i2c_enable_, mpu_bus_, qmc_bus_);

    // --- publishers (existing)
    gps_pub_ = this->create_publisher<tuna_interfaces::msg::GpsData>("/sensors/gps_data", 10);
    imu_pub_ = this->create_publisher<tuna_interfaces::msg::ImuData>("/sensors/imu_data", 10);
    lidar_pub_ = this->create_publisher<tuna_interfaces::msg::LidarData>("/sensors/lidar_data", 10);
    mission_pub_ = this->create_publisher<tuna_interfaces::msg::MissionPoint>("/mission/point", 10);
    velocity_pub_ = this->create_publisher<tuna_interfaces::msg::Velocity>("/sensors/velocity", 10);
    pid_pub_ = this->create_publisher<tuna_interfaces::msg::PidParams>("/control/pid_params", 10);
    parkur_pub_ = this->create_publisher<std_msgs::msg::Int8>("/mission/parkur_id", 10);
    control_mode_pub_ = this->create_publisher<std_msgs::msg::Int8>("/control/mode", 10);
    sensor_start_pub_ = this->create_publisher<std_msgs::msg::Int8>("/control/start_sensors", 10);
    speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/speed", 10);
    heading_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/heading", 10);
    mission_start_pub_ = this->create_publisher<tuna_interfaces::msg::MissionStart>("/mission/start_code", 10);
    control_mode_new_pub_ = this->create_publisher<tuna_interfaces::msg::ControlMode>("/control/control_mode", 10);
    heading_kalman_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/heading_kalman", 10);
    target_color_pub_ = this->create_publisher<std_msgs::msg::String>("/mission/target_color", 10);
    sensor_control_pub_ = this->create_publisher<tuna_interfaces::msg::SensorControl>("/control/sensor_control", 10);

    // --- NEW: I2C-based extra publishers
    imu_i2c_pub_   = this->create_publisher<tuna_interfaces::msg::ImuData>("/sensors/imu_data_i2c", 10);
    heading_i2c_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/heading_i2c", 10);
    heading_kalman_i2c_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/heading_kalman_i2c", 10);
    slope_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/slope", 10);
    calib_status_pub_ = this->create_publisher<std_msgs::msg::String>("/sensors/calibration_status", 10);

    // --- subscribers (existing)
    left_pwm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/motors/left_pwm", 10, std::bind(&GpsParserNode::leftPwmCallback, this, std::placeholders::_1));
    right_pwm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/motors/right_pwm", 10, std::bind(&GpsParserNode::rightPwmCallback, this, std::placeholders::_1));
    yki_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/yki/commands", 10, std::bind(&GpsParserNode::ykiCommandCallback, this, std::placeholders::_1));

    // --- HW echo pubs
    left_pwm_pub_  = this->create_publisher<std_msgs::msg::Int32>("/hw/left_pwm", 5);
    right_pwm_pub_ = this->create_publisher<std_msgs::msg::Int32>("/hw/right_pwm", 5);

    // --- threads
    parser_thread_ = std::thread(&GpsParserNode::parseLoop, this);

    if (i2c_enable_) {
      RCLCPP_INFO(this->get_logger(), "Starting I2C thread...");
      i2c_thread_ = std::thread(&GpsParserNode::i2cLoop, this);
    } else {
      RCLCPP_WARN(this->get_logger(), "I2C disabled by param; no local IMU/Compass fusion will run.");
    }
  }

  ~GpsParserNode() override {
    stop_thread_ = true;
    if (parser_thread_.joinable()) parser_thread_.join();
    if (i2c_thread_.joinable())    i2c_thread_.join();
    closeSerialIfOpen();
    closeI2CIfOpen();
  }

private:
  // ------------------- ROS pubs/subs -------------------
  rclcpp::Publisher<tuna_interfaces::msg::GpsData>::SharedPtr gps_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::ImuData>::SharedPtr imu_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::LidarData>::SharedPtr lidar_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::MissionPoint>::SharedPtr mission_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::Velocity>::SharedPtr velocity_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::PidParams>::SharedPtr pid_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr parkur_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr sensor_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::MissionStart>::SharedPtr mission_start_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::ControlMode>::SharedPtr control_mode_new_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_kalman_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_color_pub_;
  rclcpp::Publisher<tuna_interfaces::msg::SensorControl>::SharedPtr sensor_control_pub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_pwm_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_pwm_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yki_command_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_pwm_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_pwm_pub_;

  // --- NEW I2C pubs
  rclcpp::Publisher<tuna_interfaces::msg::ImuData>::SharedPtr imu_i2c_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_i2c_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_kalman_i2c_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slope_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_status_pub_;

  // ------------------- Serial state -------------------
  std::mutex serial_mutex_;
  int serial_fd_;
  std::atomic<bool> stop_thread_;
  std::thread parser_thread_;

  // Paramlar
  int serial_retry_delay_ms_;
  string explicit_serial_port_;
  const int baud_rate_;

  // PWM cache — atomic
  std::atomic<int> current_left_pwm_{0};
  std::atomic<int> current_right_pwm_{0};

  // Mod takibi
  std::atomic<int> current_mode_{0};   // 0: manuel, 1: otonom

  // Yazma hata sayacı
  int write_error_count_ = 0;

  // Read buffer & discard
  static constexpr size_t kMaxLine = 1024;
  string line_buffer_;
  bool discarding_long_line_ = false;

  // 06 rate limit
  std::chrono::steady_clock::time_point last_pwm_send_time_;

  // ------------------- I2C / IMU -------------------
  bool i2c_enable_ = true;
  int mpu_bus_ = 1;
  int qmc_bus_ = 8;
  int i2c_mpu_fd_ = -1;
  int i2c_qmc_fd_ = -1;
  std::thread i2c_thread_;
  HeadingKalman kalman_;

  // Fusion & calibration params
  int    calib_secs_ = 30;
  bool   gyro_cal_boot_ = true;
  bool   mag_cal_boot_  = true;
  double declination_deg_ = 0.0;
  double cf_alpha_ = 0.98;
  double slope_alpha_ = 0.96;
  double lpf_alpha_acc_ = 0.2;
  double lpf_alpha_mag_ = 0.2;
  bool   softiron_enable_ = true;
  double slope_threshold_ = 0.1;
  double max_slope_ = 45.0;
  bool   mirror_i2c_as_serial03_ = true;
  bool   emit_uart_serial03_     = false;

  // CF state
  bool cf_init_ = false;
  double roll_cf_ = 0.0, pitch_cf_ = 0.0, yaw_cf_ = 0.0;
  std::chrono::steady_clock::time_point last_cf_time_;
  
  // Enhanced slope composition parameters
  double slope_alpha_ = 0.96;  // Complementary filter coefficient for slope
  double slope_threshold_ = 0.1; // Minimum slope threshold in degrees
  double max_slope_ = 45.0;    // Maximum slope angle in degrees

  // LPF state
  bool lpf_init_ = false;
  double ax_f_=0, ay_f_=0, az_f_=0;
  double mx_f_=0, my_f_=0, mz_f_=0;

  // Gyro bias
  double gbx_=0.0, gby_=0.0, gbz_=0.0;

  // Enhanced Hard-iron + soft-iron calibration
  bool mag_cal_initialized_ = false;
  int16_t mag_min_x_ =  32767, mag_max_x_ = -32768;
  int16_t mag_min_y_ =  32767, mag_max_y_ = -32768;
  int16_t mag_min_z_ =  32767, mag_max_z_ = -32768;
  double bias_x_ = 0.0, bias_y_ = 0.0, bias_z_ = 0.0;
  
  // Hard iron calibration parameters
  int mag_cal_samples_ = 0;
  const int mag_cal_required_samples_ = 1000; // Minimum samples for calibration
  double mag_cal_quality_ = 0.0; // Calibration quality (0-1)
  
  // Soft iron correction matrix
  double soft_iron_matrix_[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  bool soft_iron_calibrated_ = false;

  // UART 03 throttle (1 Hz)
  std::chrono::steady_clock::time_point last_uart03_send_{std::chrono::steady_clock::now()};
  // Parser 03 throttle (5 Hz -> 200 ms)
  std::chrono::steady_clock::time_point last_parser03_send_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_terminal03_send_{std::chrono::steady_clock::now()};
  // Calibration status throttle (2 Hz -> 500 ms)
  std::chrono::steady_clock::time_point last_calib_status_send_{std::chrono::steady_clock::now()};

  // --- termios baud
  speed_t baudToSpeed(int baud) {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
#ifdef B57600
      case 57600: return B57600;
#endif
#ifdef B115200
      case 115200: return B115200;
#endif
      default: return B9600;
    }
  }

  bool tryOpenSerialPath(const string &path) {
    std::lock_guard<std::mutex> lk(serial_mutex_);
    if (serial_fd_ >= 0) return true;

    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) { close(fd); return false; }

    speed_t sp = baudToSpeed(baud_rate_);
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 2; // 200ms

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return false; }

    serial_fd_ = fd;
    RCLCPP_INFO(this->get_logger(), "Serial opened: %s (baud=%d)", path.c_str(), baud_rate_);
    return true;
  }

  void closeSerialIfOpen() {
    std::lock_guard<std::mutex> lk(serial_mutex_);
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
  }

  bool autoDetectAndOpenSerial() {
    if (!explicit_serial_port_.empty()) {
      if (tryOpenSerialPath(explicit_serial_port_)) return true;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        MAGENTA "Explicit '%s' failed; falling back to fixed list" RESET,
        explicit_serial_port_.c_str());
    }

    const char* candidates[] = {
      "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2",
      "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"
    };

    for (auto dev : candidates) {
      struct stat st{};
      if (stat(dev, &st) == 0 && S_ISCHR(st.st_mode)) {
        if (tryOpenSerialPath(dev)) return true;
      }
    }
    return false;
  }

  string readLineFromSerial() {
    std::lock_guard<std::mutex> lk(serial_mutex_);
    if (serial_fd_ < 0) return "";

    char ch;
    ssize_t r;
    while ((r = read(serial_fd_, &ch, 1)) == 1) {
      if (discarding_long_line_) {
        if (ch == '\n') { discarding_long_line_ = false; line_buffer_.clear(); }
        continue;
      }
      if (ch == '\n') {
        string line = line_buffer_;
        line_buffer_.clear();
        return line;
      }
      if (isprint(static_cast<unsigned char>(ch))) {
        line_buffer_.push_back(ch);
        if (line_buffer_.size() > kMaxLine) {
          discarding_long_line_ = true;
          line_buffer_.clear();
        }
      }
    }

    if (r < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return "";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        MAGENTA "Serial read error errno=%d" RESET, errno);
      close(serial_fd_);
      serial_fd_ = -1;
      return "";
    }
    return "";
  }

  bool writeLineToSerial(const string &line) {
    std::lock_guard<std::mutex> lk(serial_mutex_);
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        MAGENTA "Serial FD not open; skip write" RESET);
      return false;
    }

    string out = line + "\n";
    const char* buf = out.c_str();
    size_t total = out.size(), sent = 0;

    while (sent < total) {
      ssize_t w = write(serial_fd_, buf + sent, total - sent);
      if (w > 0) { sent += static_cast<size_t>(w); continue; }
      if (w < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;

      write_error_count_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        MAGENTA "Serial write error (errno=%d, cnt=%d)" RESET, errno, write_error_count_);
      return false;
    }
    write_error_count_ = 0;
    return true;
  }

  void parseLoop() {
    RCLCPP_INFO(this->get_logger(), "Parser thread started (auto-detect + retry).");
    while (rclcpp::ok() && !stop_thread_) {
      if (serial_fd_ < 0) {
        if (!autoDetectAndOpenSerial()) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Serial not found — retry in %d ms", serial_retry_delay_ms_);
          std::this_thread::sleep_for(std::chrono::milliseconds(serial_retry_delay_ms_));
          continue;
        }
      }
      string packet = readLineFromSerial();
      if (packet.empty()) { std::this_thread::sleep_for(10ms); continue; }
      if (packet.size() >= 2) parsePacket(packet);
    }
    closeSerialIfOpen();
  }

  static bool allDigits(const string &s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c){ return std::isdigit(c); });
  }

  void parsePacket(const string &packet) {
    if (packet.size() < 2) return;
    string id = packet.substr(0,2);
    double timestamp = getCurrentTimestamp();

    if (id == "01") { // GPS
      std::vector<string> parts; splitByDash(packet, parts);
      if (parts.size() != 9) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "GPS parts mismatch: %zu" RESET, parts.size());
        return;
      }

      string lat_raw = parts[1], lon_raw = parts[2], alt = parts[3], sats = parts[4];
      string hdop = parts[5], speed = parts[6], minute = parts[7], second = parts[8];

      if (!(allDigits(lat_raw) && allDigits(lon_raw) && allDigits(alt) && allDigits(sats)
            && allDigits(hdop) && allDigits(speed) && allDigits(minute) && allDigits(second))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "GPS non-digit field" RESET);
        return;
      }

      string lon_s = lon_raw.substr(0,2) + "." + lon_raw.substr(2);
      string lat_s = lat_raw.substr(0,2) + "." + lat_raw.substr(2);

      string lon = fixPrecision(lon_s);
      string lat = fixPrecision(lat_s);
      string time_formatted = minute + ":" + second;

      std::cout << "[01] GPS -> Lat: " << lat << ", Lon: " << lon << ", Alt: " << alt
                << ", Sats: " << sats << ", HDOP: " << hdop << ", Speed: " << speed
                << ", Time: " << time_formatted << std::endl;

      tuna_interfaces::msg::GpsData msg;
      msg.longitude = std::stod(lon);
      msg.latitude  = std::stod(lat);
      msg.altitude  = std::stod(alt);
      msg.satellites= std::stoi(sats);
      msg.hdop      = std::stod(hdop) / 100.0;
      msg.speed     = std::stod(speed) / 3.6;
      msg.time_stamp= time_formatted;
      msg.timestamp = timestamp;

      bool has_coordinates = (msg.latitude != 0.0 && msg.longitude != 0.0);
      bool is_test_data = (sats == "00" && hdop == "000");
      bool is_real_gps = (msg.satellites >= 4 && msg.hdop < 5.0);
      msg.valid = (is_test_data && has_coordinates) || is_real_gps;

      gps_pub_->publish(msg);
      std_msgs::msg::Float64 sp; sp.data = msg.speed; speed_pub_->publish(sp);

    } else if (id == "02") { // LIDAR
      if (packet.size() < 25) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "LIDAR short" RESET);
        return;
      }
      string angle = packet.substr(2,3);
      string dist  = packet.substr(5,4);
      string lon_raw = packet.substr(9,8);
      string lat_raw = packet.substr(17,8);

      if (!(allDigits(angle) && allDigits(dist) && allDigits(lon_raw) && allDigits(lat_raw))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "LIDAR non-digit field" RESET);
        return;
      }

      string lon = lon_raw.substr(0,2) + "." + lon_raw.substr(2);
      string lat = lat_raw.substr(0,2) + "." + lat_raw.substr(2);

      std::cout << "[02] LIDAR -> Angle: " << angle << ", Dist: " << dist
                << ", Lon: " << lon << ", Lat: " << lat << std::endl;

      tuna_interfaces::msg::LidarData msg;
      msg.angle = angle; msg.distance = dist; msg.longitude = lon; msg.latitude = lat; msg.timestamp = timestamp;
      lidar_pub_->publish(msg);

    } else if (id == "03") { // IMU (seri gelen)
      std::vector<string> parts; splitByDash(packet, parts);
      if (parts.size() != 6) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "IMU parts mismatch: %zu" RESET, parts.size());
        return;
      }

      const string &r = parts[1], &p = parts[2], &hdg_raw_s = parts[3], &hdg_kal_s = parts[4], &temp_s = parts[5];

      if (r.size() < 4 || p.size() < 4) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "IMU roll/pitch size err" RESET);
        return;
      }
      if (!((r[0]=='0'||r[0]=='1') && allDigits(r.substr(1,3)) && (p[0]=='0'||p[0]=='1') && allDigits(p.substr(1,3)))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "IMU sign/magnitude invalid" RESET);
        return;
      }
      if (!(allDigits(hdg_raw_s) && allDigits(hdg_kal_s) && allDigits(temp_s))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "IMU non-digit field" RESET);
        return;
      }

      double roll = signMagToDouble(r);
      double pitch = signMagToDouble(p);
      int hdg_raw = normalizeHeading(std::stoi(hdg_raw_s));
      int hdg_kal = normalizeHeading(std::stoi(hdg_kal_s));
      double temp = static_cast<double>(std::stoi(temp_s));

      tuna_interfaces::msg::ImuData msg;
      msg.x_angle = roll;
      msg.y_angle = pitch;
      msg.compass_angle = static_cast<double>(hdg_raw);
      msg.temperature = temp;
      msg.valid = true; msg.timestamp = timestamp;
      imu_pub_->publish(msg);

      std_msgs::msg::Float64 h;  h.data  = msg.compass_angle;           heading_pub_->publish(h);
      std_msgs::msg::Float64 hk; hk.data = static_cast<double>(hdg_kal); heading_kalman_pub_->publish(hk);

    } else if (id == "06") { // PWM (HW’den -> ROS)
      if (packet.size() < 12) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "06 packet too short" RESET);
        return;
      }
      string left_s = packet.substr(3,3), right_s = packet.substr(7,3);
      if (!(allDigits(left_s) && allDigits(right_s))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "06 non-digit" RESET);
        return;
      }

      int left = std::stoi(left_s), right = std::stoi(right_s);
      left = std::max(0, std::min(255, left));
      right = std::max(0, std::min(255, right));

      std::cout << "[06] PWM HW -> L:" << left << " R:" << right << std::endl;

      std_msgs::msg::Int32 lm; lm.data = left;  left_pwm_pub_->publish(lm);
      std_msgs::msg::Int32 rm; rm.data = right; right_pwm_pub_->publish(rm);

    } else if (id == "09") { // Mission point
      if (packet.size() != 24) {
       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         MAGENTA "09 wrong length" RESET);
       return;
     }
     string parkur_id = packet.substr(2,1);
     string coord_no  = packet.substr(3,3);
     string lat_raw   = packet.substr(6,8);
     string lon_raw   = packet.substr(14,8);
     string color     = packet.substr(22,2);
     if (!(allDigits(parkur_id) && allDigits(coord_no) && allDigits(lat_raw) && allDigits(lon_raw) && allDigits(color))) {
       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         MAGENTA "09 non-digit" RESET);
       return;
     }
     string lat_s = lat_raw.substr(0,2) + "." + lat_raw.substr(2);
     string lon_s = lon_raw.substr(0,2) + "." + lon_raw.substr(2);
     double lat=0.0, lon=0.0;
     try { lat = std::stod(lat_s); lon = std::stod(lon_s); }
     catch (...) {
       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         MAGENTA "09 stod fail" RESET);
       return;
     }
     if (!(lat >= 35.0 && lat <= 43.0 && lon >= 25.0 && lon <= 46.0)) {
       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         MAGENTA "09: point outside TR bounds (lat=%.6f lon=%.6f)" RESET, lat, lon);
       return;
     }
     int color_value = std::stoi(color);

     string target_color_name = "";
     if (color_value == 1) target_color_name = "KIRMIZI";
     else if (color_value == 2) target_color_name = "YESIL";
     else if (color_value == 3) target_color_name = "SIYAH";

     if (!target_color_name.empty()) {
       std_msgs::msg::String target_msg;
       target_msg.data = target_color_name;
       target_color_pub_->publish(target_msg);
       std::cout << "[09] Target Color -> " << target_color_name << std::endl;
     }

     std::cout << "[09] Parkur -> ID: " << parkur_id << ", Coord: " << coord_no
               << ", Lat: " << lat << ", Lon: " << lon << ", Renk: " << color << std::endl;
     tuna_interfaces::msg::MissionPoint msg;
     msg.parkur_id = std::stoi(parkur_id);
     msg.point_number = std::stoi(coord_no);
     msg.latitude = lat;
     msg.longitude = lon;
     msg.color = color_value;
     msg.timestamp = timestamp;
     mission_pub_->publish(msg);

    } else if (id == "10") { // Velocity
      if (packet.size() < 6) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "10 packet too short" RESET);
        return;
      }
      string speed = packet.substr(2,3);
      string dir   = packet.substr(5,1);
      if (!(allDigits(speed) && allDigits(dir))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "10 non-digit" RESET);
        return;
      }
      tuna_interfaces::msg::Velocity msg;
      msg.speed = std::stod(speed);
      msg.direction = std::stoi(dir);
      msg.timestamp = timestamp;
      velocity_pub_->publish(msg);
      std::cout << "[10] Hız -> Speed: " << speed << ", Yön: " << dir << std::endl;

    } else if (id == "11") { // PID
      if (packet.size() < 18) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "11 packet too short" RESET);
        return;
      }
      string kp = packet.substr(2,4), ki = packet.substr(6,4), kd = packet.substr(10,4), lim = packet.substr(14,4);
      if (!(allDigits(kp) && allDigits(ki) && allDigits(kd) && allDigits(lim))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "11 non-digit" RESET);
        return;
      }
      string kp_f = kp.substr(0,2) + "." + kp.substr(2,2);
      string ki_f = ki.substr(0,2) + "." + ki.substr(2,2);
      string kd_f = kd.substr(0,2) + "." + kd.substr(2,2);
      string lim_f = lim.substr(0,2) + "." + lim.substr(2,2);
      tuna_interfaces::msg::PidParams msg;
      msg.kp = std::stod(kp_f); msg.ki = std::stod(ki_f); msg.kd = std::stod(kd_f); msg.limit = std::stod(lim_f);
      msg.timestamp = timestamp;
      pid_pub_->publish(msg);
      std::cout << "[11] PID -> Kp: " << kp_f << ", Ki: " << ki_f << ", Kd: " << kd_f << ", Limit: " << lim_f << std::endl;

    } else if (id == "12") { // Mission start
      if (packet.size() < 3) return;
      string code = packet.substr(2,1);
      if (!allDigits(code)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "12 non-digit" RESET);
        return;
      }
      tuna_interfaces::msg::MissionStart nm;
      nm.start_code = std::stoi(code);
      nm.timestamp = timestamp;
      mission_start_pub_->publish(nm);
      std::cout << "[12] Görev Başlatma -> Kod: " << code << std::endl;

    } else if (id == "13") { // Control mode
      if (packet.size() < 3) return;
      string mode = packet.substr(2,1);
      if (!allDigits(mode)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "13 non-digit" RESET);
        return;
      }
      std_msgs::msg::Int8 oldm; oldm.data = std::stoi(mode); control_mode_pub_->publish(oldm);
      tuna_interfaces::msg::ControlMode nm; nm.mode = std::stoi(mode); nm.mode_name = (mode == "0" ? "Manuel" : "Otonom"); nm.timestamp = timestamp;
      control_mode_new_pub_->publish(nm);
      current_mode_.store(nm.mode);
      std::cout << "[13] Kontrol Modu -> " << (nm.mode == 0 ? "Manuel" : "Otonom") << std::endl;

    } else if (id == "14") { // Sensor control
      if (packet.size() < 3) return;
      string code = packet.substr(2,1);
      if (!allDigits(code)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          MAGENTA "14 non-digit" RESET);
        return;
      }
      tuna_interfaces::msg::SensorControl sm; sm.sensor_code = std::stoi(code); sm.timestamp = timestamp;
      sensor_control_pub_->publish(sm);
      std::cout << "[14] Sensör Başlatma -> Kod: " << code << std::endl;
    }
  }

  // ================= I2C LOOP =================
  void i2cLoop() {
    RCLCPP_INFO(this->get_logger(), "I2C loop starting (MPU bus=%d, QMC bus=%d)", mpu_bus_, qmc_bus_);

    i2c_mpu_fd_ = openI2C(mpu_bus_);
    i2c_qmc_fd_ = openI2C(qmc_bus_);
    if (i2c_mpu_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Open I2C mpu_bus failed");
      return;
    }
    if (i2c_qmc_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Open I2C qmc_bus failed (heading olmayabilir)");
    }

    // MPU6050 init (0x68)
    if (!i2cWriteByte(i2c_mpu_fd_, 0x68, 0x6B, 0x00)) { // wake
      RCLCPP_ERROR(this->get_logger(), "Failed to wake MPU6050");
      return;
    }
    if (!i2cWriteByte(i2c_mpu_fd_, 0x68, 0x1C, 0x00)) { // ±2g
      RCLCPP_ERROR(this->get_logger(), "Failed to set MPU6050 accel range");
      return;
    }
    if (!i2cWriteByte(i2c_mpu_fd_, 0x68, 0x1B, 0x00)) { // ±250 dps
      RCLCPP_ERROR(this->get_logger(), "Failed to set MPU6050 gyro range");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "MPU6050 initialized successfully");

    // QMC5883L init (0x0D): 200Hz, 2G, continuous
    if (i2c_qmc_fd_ >= 0) {
      if (!i2cWriteByte(i2c_qmc_fd_, 0x0D, 0x09, 0x1D)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize QMC5883L");
        close(i2c_qmc_fd_);
        i2c_qmc_fd_ = -1;
      } else {
        RCLCPP_INFO(this->get_logger(), "QMC5883L initialized successfully");
      }
    }

    // --- BOOT CALIBRATION (countdown)
    if (calib_secs_ > 0 && (gyro_cal_boot_ || (mag_cal_boot_ && i2c_qmc_fd_ >= 0))) {
      RCLCPP_WARN(this->get_logger(), "CALIBRATION started: %d seconds. Move sensor in '8' pattern for MAG; keep still for GYRO.", calib_secs_);
      auto start = std::chrono::steady_clock::now();
      int last_print = -1;

      // reset accumulators
      long n_gyro = 0;
      double sum_gx = 0, sum_gy = 0, sum_gz = 0;
      mag_min_x_ =  32767; mag_max_x_ = -32768;
      mag_min_y_ =  32767; mag_max_y_ = -32768;
      mag_min_z_ =  32767; mag_max_z_ = -32768;

      while (rclcpp::ok()) {
        auto nowtp = std::chrono::steady_clock::now();
        int elapsed = (int)std::chrono::duration_cast<std::chrono::seconds>(nowtp - start).count();
        int remain = calib_secs_ - elapsed;
        if (remain < 0) break;
        if (remain != last_print) {
          std::cout << "CALIB " << remain << std::endl;
          last_print = remain;
        }

        // read MPU (acc+temp+gyro)
        uint8_t buf[14];
        if (i2cReadBytes(i2c_mpu_fd_, 0x68, 0x3B, buf, 14)) {
          int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
          int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
          int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
          if (gyro_cal_boot_) {
            sum_gx += gx / 131.0;
            sum_gy += gy / 131.0;
            sum_gz += gz / 131.0;
            n_gyro++;
          }
        }

        // read MAG
        if (mag_cal_boot_ && i2c_qmc_fd_ >= 0) {
          uint8_t mb[6];
          if (i2cReadBytes(i2c_qmc_fd_, 0x0D, 0x00, mb, 6)) {
            int16_t mx = (int16_t)((mb[1] << 8) | mb[0]);
            int16_t my = (int16_t)((mb[3] << 8) | mb[2]);
            int16_t mz = (int16_t)((mb[5] << 8) | mb[4]);
            mag_min_x_ = std::min(mag_min_x_, mx); mag_max_x_ = std::max(mag_max_x_, mx);
            mag_min_y_ = std::min(mag_min_y_, my); mag_max_y_ = std::max(mag_max_y_, my);
            mag_min_z_ = std::min(mag_min_z_, mz); mag_max_z_ = std::max(mag_max_z_, mz);
          }
        }

        std::this_thread::sleep_for(20ms); // ~50Hz during calib
      }

      if (gyro_cal_boot_ && n_gyro > 0) {
        gbx_ = sum_gx / (double)n_gyro;
        gby_ = sum_gy / (double)n_gyro;
        gbz_ = sum_gz / (double)n_gyro;
      }

      if (mag_cal_boot_ && i2c_qmc_fd_ >= 0) {
        bias_x_ = (mag_max_x_ + mag_min_x_) * 0.5;
        bias_y_ = (mag_max_y_ + mag_min_y_) * 0.5;
        bias_z_ = (mag_max_z_ + mag_min_z_) * 0.5;
        mag_cal_initialized_ = true;
        mag_cal_samples_ = 1000; // Set initial samples for quality assessment
        mag_cal_quality_ = assessCalibrationQuality();
        
        RCLCPP_INFO(this->get_logger(), 
          "Magnetic calibration completed. Quality: %.2f, Bias: (%.1f, %.1f, %.1f)", 
          mag_cal_quality_, bias_x_, bias_y_, bias_z_);
      }

      // reset LPF init
      lpf_init_ = false;
      std::cout << "CALIB DONE" << std::endl;
    }

    // ---- 200 Hz döngü
    const auto period = 5ms; // 200 Hz
    auto next = std::chrono::steady_clock::now();
    last_cf_time_ = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !stop_thread_) {
      next += period;

      // ---- MPU: ACCEL+TEMP+GYRO (14 byte)
      int16_t ax=0, ay=0, az=0, rawTemp=0, gx=0, gy=0, gz=0;
      {
        uint8_t buf[14];
        if (i2cReadBytes(i2c_mpu_fd_, 0x68, 0x3B, buf, 14)) {
          ax = (int16_t)((buf[0] << 8) | buf[1]);
          ay = (int16_t)((buf[2] << 8) | buf[3]);
          az = (int16_t)((buf[4] << 8) | buf[5]);
          rawTemp = (int16_t)((buf[6] << 8) | buf[7]);
          gx = (int16_t)((buf[8] << 8) | buf[9]);
          gy = (int16_t)((buf[10] << 8) | buf[11]);
          gz = (int16_t)((buf[12] << 8) | buf[13]);
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, MAGENTA "MPU read fail" RESET);
        }
      }

      // ölçekler
      double axg = ax / 16384.0, ayg = ay / 16384.0, azg = az / 16384.0;
      double gx_dps = gx / 131.0 - gbx_;
      double gy_dps = gy / 131.0 - gby_;
      double gz_dps = gz / 131.0 - gbz_;
      (void)gz_dps;

      // sıcaklık
      double tempC = (rawTemp / 340.0) + 36.53;

      // LPF (EMA) accel
      if (!lpf_init_) {
        ax_f_=axg; ay_f_=ayg; az_f_=azg;
      } else {
        ax_f_ = lpf_alpha_acc_*axg + (1.0-lpf_alpha_acc_)*ax_f_;
        ay_f_ = lpf_alpha_acc_*ayg + (1.0-lpf_alpha_acc_)*ay_f_;
        az_f_ = lpf_alpha_acc_*azg + (1.0-lpf_alpha_acc_)*az_f_;
      }

      // accel'den açı
      double pitch_acc = std::atan2(ay_f_, std::sqrt(ax_f_*ax_f_ + az_f_*az_f_)) * 180.0 / 3.14159265358979323846;
      double roll_acc  = std::atan2(-ax_f_, az_f_) * 180.0 / 3.14159265358979323846;

      // Complementary filter (gyro + accel)
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last_cf_time_).count();
      if (dt <= 0.0 || dt > 0.05) dt = 0.005; // 200 Hz fallback
      last_cf_time_ = now;

      if (!cf_init_) {
        roll_cf_  = roll_acc;
        pitch_cf_ = pitch_acc;
        yaw_cf_   = 0.0;
        cf_init_ = true;
      } else {
        double roll_g  = roll_cf_  + gx_dps * dt;
        double pitch_g = pitch_cf_ + gy_dps * dt;
        double yaw_g   = yaw_cf_   + gz_dps * dt;
        
        roll_cf_  = cf_alpha_*roll_g  + (1.0 - cf_alpha_)*roll_acc;
        pitch_cf_ = cf_alpha_*pitch_g + (1.0 - cf_alpha_)*pitch_acc;
        yaw_cf_   = yaw_g; // Yaw only from gyro for now
      }

      // Enhanced slope composition
      double slope_magnitude = calculateSlope(roll_cf_, pitch_cf_);

      // ---- MAG (QMC5883L)
      int16_t mx=0, my=0, mz=0;
      bool mag_ok = false;
      if (i2c_qmc_fd_ >= 0) {
        uint8_t bufm[6];
        if (i2cReadBytes(i2c_qmc_fd_, 0x0D, 0x00, bufm, 6)) {
          mx = (int16_t)((bufm[1] << 8) | bufm[0]);
          my = (int16_t)((bufm[3] << 8) | bufm[2]);
          mz = (int16_t)((bufm[5] << 8) | bufm[4]);
          mag_ok = true;
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, MAGENTA "QMC read fail" RESET);
        }
      }

      // Enhanced Hard-iron calibration
      if (mag_ok && mag_cal_boot_) {
        updateHardIronCalibration(mx, my, mz);
      }

      // bias çıkar
      double mxc = (double)mx - bias_x_;
      double myc = (double)my - bias_y_;
      double mzc = (double)mz - bias_z_;

      // Soft-iron scale
      if (softiron_enable_ && mag_cal_initialized_) {
        double dx = 0.5 * (mag_max_x_ - mag_min_x_);
        double dy = 0.5 * (mag_max_y_ - mag_min_y_);
        double dz = 0.5 * (mag_max_z_ - mag_min_z_);
        double avg = (dx + dy + dz) / 3.0;
        if (dx > 1 && dy > 1 && dz > 1) {
          mxc *= (avg / dx);
          myc *= (avg / dy);
          mzc *= (avg / dz);
        }
      }

      // LPF (EMA) mag
      if (!lpf_init_) {
        mx_f_=mxc; my_f_=myc; mz_f_=mzc;
        lpf_init_ = true;
      } else {
        mx_f_ = lpf_alpha_mag_*mxc + (1.0-lpf_alpha_mag_)*mx_f_;
        my_f_ = lpf_alpha_mag_*myc + (1.0-lpf_alpha_mag_)*my_f_;
        mz_f_ = lpf_alpha_mag_*mzc + (1.0-lpf_alpha_mag_)*mz_f_;
      }

      // Enhanced tilt compensation
      double heading = calculateTiltCompensatedHeading(mx_f_, my_f_, mz_f_, roll_cf_, pitch_cf_);

      // Magnetic interference detection
      bool interference_detected = detectMagneticInterference(mx_f_, my_f_, mz_f_);
      if (interference_detected) {
        // Use gyro-based heading when interference is detected
        heading = yaw_cf_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
          MAGENTA "Magnetic interference detected, using gyro heading" RESET);
      }

      // declination & Kalman
      heading = wrapDeg(heading + declination_deg_);
      double heading_kal = kalman_.update(heading);

      // --- I2C topic'leri (200 Hz)
      {
        tuna_interfaces::msg::ImuData imsg;
        imsg.x_angle = roll_cf_;
        imsg.y_angle = pitch_cf_;
        imsg.compass_angle = heading;
        imsg.temperature = tempC;
        imsg.valid = true;
        imsg.timestamp = getCurrentTimestamp();
        imu_i2c_pub_->publish(imsg);
      }
      {
        std_msgs::msg::Float64 h;  h.data = heading;
        std_msgs::msg::Float64 hk; hk.data = heading_kal;
        std_msgs::msg::Float64 s;  s.data = slope_magnitude;
        heading_i2c_pub_->publish(h);
        heading_kalman_i2c_pub_->publish(hk);
        slope_pub_->publish(s);
      }

      // --- 03 formatlı paket (STM ile birebir; TEMP integer, ör: 029)
      std::string r_sm = packSignMag4(roll_cf_);
      std::string p_sm = packSignMag4(pitch_cf_);

      int hdg_i  = (int)std::lround(heading);
      if (hdg_i < 0) hdg_i = (hdg_i % 360 + 360) % 360; else hdg_i = hdg_i % 360;

      int hk_i   = (int)std::lround(heading_kal);
      if (hk_i < 0) hk_i = (hk_i % 360 + 360) % 360; else hk_i = hk_i % 360;

      int temp_i = (int)std::lround(tempC);

      std::ostringstream oss03;
      oss03 << "03-" << r_sm << "-" << p_sm << "-"
            << std::setw(3) << std::setfill('0') << hdg_i << "-"
            << std::setw(3) << std::setfill('0') << hk_i  << "-"
            << std::setw(3) << std::setfill('0') << std::max(0, std::min(999, temp_i));

      std::string pkt03 = oss03.str();

      // 1) Terminale formatlı bas (4 Hz -> her 250 ms)
      auto nowtp_terminal = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(nowtp_terminal - last_terminal03_send_).count() >= 250) {
        std::cout << pkt03 << std::endl;
        last_terminal03_send_ = nowtp_terminal;
      }

      // 2) Parser zincirine ENJEKSİYON (4 Hz -> her 250 ms)
      auto nowtp_parser = std::chrono::steady_clock::now();
      if (mirror_i2c_as_serial03_ &&
          std::chrono::duration_cast<std::chrono::milliseconds>(nowtp_parser - last_parser03_send_).count() >= 250) {
        parsePacket(pkt03);
        last_parser03_send_ = nowtp_parser;
     }
      // 3) Her 1 saniyede bir UART'a gönder (isteğe bağlı)
      auto nowtp2 = std::chrono::steady_clock::now();
      if (emit_uart_serial03_ &&
          std::chrono::duration_cast<std::chrono::seconds>(nowtp2 - last_uart03_send_).count() >= 1) {
        writeLineToSerial(pkt03);
        last_uart03_send_ = nowtp2;
      }

      // 4) Calibration status publishing (2 Hz)
      auto nowtp3 = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(nowtp3 - last_calib_status_send_).count() >= 500) {
        publishCalibrationStatus();
        last_calib_status_send_ = nowtp3;
      }

      std::this_thread::sleep_until(next);
    }

    closeI2CIfOpen();
  }

  void closeI2CIfOpen() {
    if (i2c_mpu_fd_ >= 0) { close(i2c_mpu_fd_); i2c_mpu_fd_ = -1; }
    if (i2c_qmc_fd_ >= 0) { close(i2c_qmc_fd_); i2c_qmc_fd_ = -1; }
  }

  // helpers
  static void splitByDash(const string &s, std::vector<string> &out) {
    out.clear();
    size_t start = 0, end;
    while ((end = s.find('-', start)) != string::npos) {
      out.push_back(s.substr(start, end - start));
      start = end + 1;
    }
    out.push_back(s.substr(start));
  }

  static int normalizeHeading(int h) {
    if (h < 0) h = (h % 360 + 360) % 360;
    else h = h % 360;
    return h;
  }

  static double signMagToDouble(const string &sm4) {
    int sign = (sm4[0] == '1') ? +1 : -1;
    int mag = std::stoi(sm4.substr(1,3));
    return static_cast<double>(sign * mag);
  }

  string fixPrecision(const string &coord_str) {
    size_t pos = coord_str.find('.');
    if (pos == string::npos) return coord_str + ".000000";
    size_t decimals = coord_str.size() - pos - 1;
    if (decimals >= 6) return coord_str;
    return coord_str + string(6 - decimals, '0');
  }

  double getCurrentTimestamp() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // Enhanced slope composition function with moving average
  double calculateSlope(double roll, double pitch) {
    // Calculate total slope magnitude from roll and pitch
    double slope_magnitude = std::sqrt(roll * roll + pitch * pitch);
    
    // Apply threshold to filter out noise
    if (slope_magnitude < slope_threshold_) {
      slope_magnitude = 0.0;
    }
    
    // Limit maximum slope
    if (slope_magnitude > max_slope_) {
      slope_magnitude = max_slope_;
    }
    
    // Apply moving average filter for stability
    static double slope_history[5] = {0.0};
    static int history_index = 0;
    static bool history_filled = false;
    
    slope_history[history_index] = slope_magnitude;
    history_index = (history_index + 1) % 5;
    if (history_index == 0) history_filled = true;
    
    double filtered_slope = 0.0;
    int count = history_filled ? 5 : history_index;
    for (int i = 0; i < count; i++) {
      filtered_slope += slope_history[i];
    }
    filtered_slope /= count;
    
    return filtered_slope;
  }

  // Enhanced compass calibration quality assessment
  double assessCalibrationQuality() {
    if (!mag_cal_initialized_ || mag_cal_samples_ < mag_cal_required_samples_) {
      return 0.0;
    }
    
    // Calculate calibration spread
    double dx = mag_max_x_ - mag_min_x_;
    double dy = mag_max_y_ - mag_min_y_;
    double dz = mag_max_z_ - mag_min_z_;
    
    // Check if we have sufficient spread in all axes
    double min_spread = std::min({dx, dy, dz});
    double max_spread = std::max({dx, dy, dz});
    
    if (min_spread < 1000 || max_spread < 2000) {
      return 0.0; // Insufficient spread
    }
    
    // Calculate quality based on spread uniformity
    double spread_ratio = min_spread / max_spread;
    double quality = std::min(1.0, spread_ratio * 2.0); // Scale to 0-1
    
    return quality;
  }

  // Enhanced hard iron calibration
  void updateHardIronCalibration(int16_t mx, int16_t my, int16_t mz) {
    if (!mag_cal_initialized_) {
      mag_min_x_ = mag_max_x_ = mx;
      mag_min_y_ = mag_max_y_ = my;
      mag_min_z_ = mag_max_z_ = mz;
      mag_cal_initialized_ = true;
      mag_cal_samples_ = 1;
    } else {
      mag_min_x_ = std::min(mag_min_x_, mx);
      mag_max_x_ = std::max(mag_max_x_, mx);
      mag_min_y_ = std::min(mag_min_y_, my);
      mag_max_y_ = std::max(mag_max_y_, my);
      mag_min_z_ = std::min(mag_min_z_, mz);
      mag_max_z_ = std::max(mag_max_z_, mz);
      mag_cal_samples_++;
    }
    
    // Update bias and quality
    bias_x_ = (mag_max_x_ + mag_min_x_) * 0.5;
    bias_y_ = (mag_max_y_ + mag_min_y_) * 0.5;
    bias_z_ = (mag_max_z_ + mag_min_z_) * 0.5;
    
    mag_cal_quality_ = assessCalibrationQuality();
  }

  // Enhanced tilt compensation with improved accuracy
  double calculateTiltCompensatedHeading(double mx, double my, double mz, double roll, double pitch) {
    // Convert to radians
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    
    // Apply tilt compensation matrix
    double Xh = mx * std::cos(pitch_rad) + mz * std::sin(pitch_rad);
    double Yh = mx * std::sin(roll_rad) * std::sin(pitch_rad) + 
                my * std::cos(roll_rad) - 
                mz * std::sin(roll_rad) * std::cos(pitch_rad);
    
    // Calculate heading
    double heading = std::atan2(Yh, Xh) * 180.0 / M_PI;
    
    // Normalize to 0-360
    heading = wrapDeg(heading);
    
    return heading;
  }

  // Publish calibration status
  void publishCalibrationStatus() {
    std::ostringstream oss;
    oss << "GYRO_CAL:" << (gyro_cal_boot_ ? "OK" : "SKIP") 
        << " MAG_CAL:" << (mag_cal_initialized_ ? "OK" : "PENDING")
        << " QUALITY:" << std::fixed << std::setprecision(2) << mag_cal_quality_
        << " SAMPLES:" << mag_cal_samples_;
    
    std_msgs::msg::String status_msg;
    status_msg.data = oss.str();
    calib_status_pub_->publish(status_msg);
  }

  // Detect magnetic interference
  bool detectMagneticInterference(double mx, double my, double mz) {
    // Calculate magnetic field magnitude
    double mag_magnitude = std::sqrt(mx*mx + my*my + mz*mz);
    
    // Normal magnetic field should be around 0.4-0.6 Gauss (40000-60000 in raw units)
    // Check for significant deviation
    if (mag_magnitude < 20000 || mag_magnitude > 80000) {
      return true; // Interference detected
    }
    
    // Check for sudden changes (indicating nearby magnetic objects)
    static double last_mag_magnitude = mag_magnitude;
    double mag_change = std::abs(mag_magnitude - last_mag_magnitude);
    last_mag_magnitude = mag_magnitude;
    
    if (mag_change > 10000) { // Sudden change threshold
      return true; // Interference detected
    }
    
    return false; // No interference
  }

  void sendCurrentPwmIfAutonomous(const char* src) {
    if (current_mode_.load() != 1) return;
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pwm_send_time_).count();
    if (elapsed_ms < 100) return;
    last_pwm_send_time_ = now;

    int left  = std::max(0, std::min(255, current_left_pwm_.load()));
    int right = std::max(0, std::min(255, current_right_pwm_.load()));

    std::ostringstream oss;
    oss << "06"
        << "2" << std::setw(3) << std::setfill('0') << left
        << "2" << std::setw(3) << std::setfill('0') << right
        << "00";
    if (!writeLineToSerial(oss.str())) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        MAGENTA "06 write failed (src=%s)" RESET, src);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "06 wrote L=%d R=%d (src=%s)", left, right, src);
    }
  }

  void leftPwmCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    int v = std::max(0, std::min(255, msg->data));
    current_left_pwm_.store(v);
    sendCurrentPwmIfAutonomous("left_cb");
  }
  void rightPwmCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    int v = std::max(0, std::min(255, msg->data));
    current_right_pwm_.store(v);
    sendCurrentPwmIfAutonomous("right_cb");
  }
  void ykiCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "YKI command (ignored): %s", msg->data.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsParserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
