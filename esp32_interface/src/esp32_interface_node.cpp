#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <grace_msgs/msg/environment_data.hpp>
#include <grace_msgs/msg/air_quality.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <sstream>
#include <fstream>
#include <memory>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>

#include "esp32_interface/esp32_parser.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace esp32_interface
{

class ESP32InterfaceNode : public rclcpp::Node
{
public:
  ESP32InterfaceNode()
  : Node("esp32_interface_node"),
    serial_fd_(-1),
    buffer_("")
  {
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "/dev/grace_esp32");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("wheel_radius", 0.065);
    this->declare_parameter<std::string>("imu1_frame_id", "imu1_link");
    this->declare_parameter<std::string>("imu2_frame_id", "imu2_link");
    this->declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
    this->declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");
    this->declare_parameter<bool>("enable_csv_logging", true);
    this->declare_parameter<std::string>("csv_log_dir", "");

    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    imu1_frame_id_ = this->get_parameter("imu1_frame_id").as_string();
    imu2_frame_id_ = this->get_parameter("imu2_frame_id").as_string();
    left_wheel_joint_ = this->get_parameter("left_wheel_joint").as_string();
    right_wheel_joint_ = this->get_parameter("right_wheel_joint").as_string();
    enable_csv_logging_ = this->get_parameter("enable_csv_logging").as_bool();
    csv_log_dir_ = this->get_parameter("csv_log_dir").as_string();

    // ===================== Publishers =====================

    // IMU
    imu1_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu1/data", 10);
    imu2_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu2/data", 10);

    // Motor / Drive
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    board_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("board_temperature", 10);

    // Environment (BME680) — custom message
    environment_pub_ = this->create_publisher<grace_msgs::msg::EnvironmentData>("environment", 10);

    // Air Quality (PMS5003) — custom message
    air_quality_pub_ = this->create_publisher<grace_msgs::msg::AirQuality>("air_quality", 10);

    // ===================== Subscribers =====================

    left_motor_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "left_motor_rpm", 10,
      std::bind(&ESP32InterfaceNode::leftMotorCallback, this, std::placeholders::_1));

    right_motor_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "right_motor_rpm", 10,
      std::bind(&ESP32InterfaceNode::rightMotorCallback, this, std::placeholders::_1));

    led_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "led_command", 10,
      std::bind(&ESP32InterfaceNode::ledCommandCallback, this, std::placeholders::_1));

    // ===================== CSV Logging =====================

    if (enable_csv_logging_) {
      initCsvLogging();
    }

    // ===================== Serial Port =====================

    if (!openSerialPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "ESP32 Interface Node started on port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing: imu1/data, imu2/data, joint_states, battery_state, board_temperature");
    RCLCPP_INFO(this->get_logger(), "Publishing: environment (BME680), air_quality (PMS5003)");
    RCLCPP_INFO(this->get_logger(), "Subscribing: left_motor_rpm, right_motor_rpm, led_command");

    if (enable_csv_logging_ && csv_file_.is_open()) {
      RCLCPP_INFO(this->get_logger(), "CSV logging to: %s", csv_file_path_.c_str());
    }

    // Timer for reading serial data (100Hz)
    timer_ = this->create_wall_timer(
      10ms, std::bind(&ESP32InterfaceNode::serialReadCallback, this));
  }

  ~ESP32InterfaceNode()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  // ===================== CSV Logging =====================

  void initCsvLogging()
  {
    // Determine log directory
    std::string log_dir = csv_log_dir_;
    if (log_dir.empty()) {
      // Default: esp32_interface/logs/ inside the package source
      // Use ament_index to find package share, or fallback to known path
      log_dir = getDefaultLogDir();
    }

    // Create directory if it doesn't exist
    try {
      fs::create_directories(log_dir);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create log directory '%s': %s",
                   log_dir.c_str(), e.what());
      return;
    }

    // Single CSV file
    csv_file_path_ = log_dir + "/esp32_data.csv";

    // Check if file already exists (to decide whether to write header)
    bool file_exists = fs::exists(csv_file_path_);

    // Open in append mode
    csv_file_.open(csv_file_path_, std::ios::app);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_file_path_.c_str());
      return;
    }

    // Write header only if file is new
    if (!file_exists) {
      csv_file_ << "datetime,"
                << "dt_ms,driver_alive,"
                << "battery_v,board_temp_c,"
                << "left_target_rpm,left_measured_rpm,"
                << "right_target_rpm,right_measured_rpm,"
                << "imu1_ok,ax1,ay1,az1,gx1,gy1,gz1,"
                << "imu2_ok,ax2,ay2,az2,gx2,gy2,gz2,"
                << "bme_temp_c,bme_humidity_pct,bme_pressure_hpa,bme_gas_kohm,"
                << "pms_ok,pm1_0,pm2_5,pm10"
                << "\n";
      csv_file_.flush();
    }
  }

  std::string getDefaultLogDir()
  {
    // Try to find the package source directory
    // Fallback chain: AMENT_PREFIX_PATH based, or hardcoded workspace path
    const char* ws = std::getenv("COLCON_PREFIX_PATH");
    if (ws) {
      // Navigate from install back to src
      fs::path ws_path(ws);
      fs::path src_logs = ws_path.parent_path() / "src" / "esp32_interface" / "logs";
      if (fs::exists(src_logs.parent_path())) {
        return src_logs.string();
      }
    }

    // Hardcoded fallback for this workspace
    return std::string(std::getenv("HOME")) + "/grace_ws/src/esp32_interface/logs";
  }

  std::string getCurrentDateTimeString()
  {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;

    std::tm tm_buf;
    localtime_r(&time_t_now, &tm_buf);

    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
  }

  void logToCsv(const ESP32Data& data)
  {
    if (!csv_file_.is_open()) {
      return;
    }

    csv_file_ << getCurrentDateTimeString() << ","
              << data.esp32_dt_ms << ","
              << (int)data.driver_status << ","
              << ESP32Parser::batteryMvToVolts(data.battery_mv) << ","
              << ESP32Parser::tempDeciCToCelsius(data.temperature_deci_c) << ","
              << data.left_target_rpm << ","
              << data.left_measured_rpm << ","
              << data.right_target_rpm << ","
              << data.right_measured_rpm << ","
              << (int)data.imu1_status << ","
              << data.ax1 << "," << data.ay1 << "," << data.az1 << ","
              << data.gx1 << "," << data.gy1 << "," << data.gz1 << ","
              << (int)data.imu2_status << ","
              << data.ax2 << "," << data.ay2 << "," << data.az2 << ","
              << data.gx2 << "," << data.gy2 << "," << data.gz2 << ","
              << data.bme_temperature << ","
              << data.bme_humidity << ","
              << data.bme_pressure << ","
              << data.bme_gas_resistance << ","
              << (int)data.pms_status << ","
              << data.pm1_0 << ","
              << data.pm2_5 << ","
              << data.pm10_0
              << "\n";

    // Flush every 100 lines for performance
    csv_line_count_++;
    if (csv_line_count_ % 100 == 0) {
      csv_file_.flush();
    }
  }

  // ===================== Serial Port =====================

  bool openSerialPort()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0) {
      return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    speed_t speed = B115200;
    switch (baud_rate_) {
      case 9600:   speed = B9600;   break;
      case 19200:  speed = B19200;  break;
      case 38400:  speed = B38400;  break;
      case 57600:  speed = B57600;  break;
      case 115200: speed = B115200; break;
      default:     speed = B115200; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    return true;
  }

  // ===================== Serial Read Loop =====================

  void serialReadCallback()
  {
    if (serial_fd_ < 0) {
      return;
    }

    char read_buf[256];
    ssize_t n = read(serial_fd_, read_buf, sizeof(read_buf) - 1);

    if (n > 0) {
      read_buf[n] = '\0';
      buffer_ += std::string(read_buf);

      size_t pos;
      while ((pos = buffer_.find('\n')) != std::string::npos) {
        std::string line = buffer_.substr(0, pos);
        buffer_.erase(0, pos + 1);

        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }

        processLine(line);
      }

      if (buffer_.size() > 2048) {
        buffer_.clear();
      }
    }
  }

  void processLine(const std::string& line)
  {
    if (line.empty() || line[0] != ',') {
      return;
    }

    ESP32Data data = parser_.parseLine(line);

    if (!data.valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Failed to parse line: %s", line.c_str());
      return;
    }

    auto timestamp = this->now();

    // --- IMU data (independent of driver) ---
    if (data.imu1_status == 1) {
      publishImu1(data, timestamp);
    }
    if (data.imu2_status == 1) {
      publishImu2(data, timestamp);
    }

    // --- Driver-dependent data ---
    if (data.driver_status == 1) {
      publishJointStates(data, timestamp);
      publishBatteryState(data, timestamp);
      publishBoardTemperature(data, timestamp);
    }

    // --- BME680 environment data (always publish, sensor_available flag indicates status) ---
    publishEnvironment(data, timestamp);

    // --- PMS5003 air quality data ---
    publishAirQuality(data, timestamp);

    // --- CSV logging ---
    if (enable_csv_logging_) {
      logToCsv(data);
    }
  }

  // ===================== IMU Publishers =====================

  void publishImu1(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = timestamp;
    msg.header.frame_id = imu1_frame_id_;

    msg.linear_acceleration.x = ESP32Parser::rawAccelToMps2(data.ax1);
    msg.linear_acceleration.y = ESP32Parser::rawAccelToMps2(data.ay1);
    msg.linear_acceleration.z = ESP32Parser::rawAccelToMps2(data.az1);

    msg.angular_velocity.x = ESP32Parser::rawGyroToRadps(data.gx1);
    msg.angular_velocity.y = ESP32Parser::rawGyroToRadps(data.gy1);
    msg.angular_velocity.z = ESP32Parser::rawGyroToRadps(data.gz1);

    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;  // orientation unavailable

    msg.linear_acceleration_covariance[0] = 0.0001;
    msg.linear_acceleration_covariance[4] = 0.0001;
    msg.linear_acceleration_covariance[8] = 0.0001;

    msg.angular_velocity_covariance[0] = 0.0001;
    msg.angular_velocity_covariance[4] = 0.0001;
    msg.angular_velocity_covariance[8] = 0.0001;

    imu1_pub_->publish(msg);
  }

  void publishImu2(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = timestamp;
    msg.header.frame_id = imu2_frame_id_;

    msg.linear_acceleration.x = ESP32Parser::rawAccelToMps2(data.ax2);
    msg.linear_acceleration.y = ESP32Parser::rawAccelToMps2(data.ay2);
    msg.linear_acceleration.z = ESP32Parser::rawAccelToMps2(data.az2);

    msg.angular_velocity.x = ESP32Parser::rawGyroToRadps(data.gx2);
    msg.angular_velocity.y = ESP32Parser::rawGyroToRadps(data.gy2);
    msg.angular_velocity.z = ESP32Parser::rawGyroToRadps(data.gz2);

    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;

    msg.linear_acceleration_covariance[0] = 0.0001;
    msg.linear_acceleration_covariance[4] = 0.0001;
    msg.linear_acceleration_covariance[8] = 0.0001;

    msg.angular_velocity_covariance[0] = 0.0001;
    msg.angular_velocity_covariance[4] = 0.0001;
    msg.angular_velocity_covariance[8] = 0.0001;

    imu2_pub_->publish(msg);
  }

  // ===================== Motor / Battery Publishers =====================

  void publishJointStates(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = timestamp;

    msg.name.push_back(left_wheel_joint_);
    msg.name.push_back(right_wheel_joint_);

    msg.velocity.push_back(ESP32Parser::rpmToRadps(data.left_measured_rpm));
    msg.velocity.push_back(ESP32Parser::rpmToRadps(data.right_measured_rpm));

    msg.position.resize(2, 0.0);
    msg.effort.resize(2, 0.0);

    joint_state_pub_->publish(msg);
  }

  void publishBatteryState(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::BatteryState();
    msg.header.stamp = timestamp;

    msg.voltage = ESP32Parser::batteryMvToVolts(data.battery_mv);
    msg.temperature = ESP32Parser::tempDeciCToCelsius(data.temperature_deci_c);

    double voltage = msg.voltage;
    if (voltage >= 42.0) {
      msg.percentage = 1.0;
    } else if (voltage <= 33.0) {
      msg.percentage = 0.0;
    } else {
      msg.percentage = (voltage - 33.0) / (42.0 - 33.0);
    }

    if (voltage < 35.0) {
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    msg.present = true;

    battery_pub_->publish(msg);
  }

  void publishBoardTemperature(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::Temperature();
    msg.header.stamp = timestamp;
    msg.header.frame_id = "base_footprint";

    msg.temperature = ESP32Parser::tempDeciCToCelsius(data.temperature_deci_c);
    msg.variance = 1.0;

    board_temp_pub_->publish(msg);
  }

  // ===================== Environment Publisher (BME680) =====================

  void publishEnvironment(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = grace_msgs::msg::EnvironmentData();
    msg.header.stamp = timestamp;
    msg.header.frame_id = "bme680_link";

    msg.temperature = static_cast<float>(data.bme_temperature);
    msg.humidity = static_cast<float>(data.bme_humidity);
    msg.pressure = static_cast<float>(data.bme_pressure);
    msg.gas_resistance = static_cast<float>(data.bme_gas_resistance);
    msg.sensor_available = (data.bme_temperature != 0 || data.bme_humidity != 0);

    environment_pub_->publish(msg);
  }

  // ===================== Air Quality Publisher (PMS5003) =====================

  void publishAirQuality(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = grace_msgs::msg::AirQuality();
    msg.header.stamp = timestamp;
    msg.header.frame_id = "pms5003_link";

    msg.pm1_0 = data.pm1_0;
    msg.pm2_5 = data.pm2_5;
    msg.pm10_0 = data.pm10_0;
    msg.sensor_available = (data.pms_status == 1);

    air_quality_pub_->publish(msg);
  }

  // ===================== Subscriber Callbacks =====================

  void leftMotorCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    left_motor_rpm_ = msg->data;
    sendMotorCommand();
  }

  void rightMotorCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    right_motor_rpm_ = msg->data;
    sendMotorCommand();
  }

  void sendMotorCommand()
  {
    // Grace-style unified command: "l<rpm> r<rpm>\n"
    std::string cmd = "l" + std::to_string(left_motor_rpm_) +
                      " r" + std::to_string(right_motor_rpm_) + "\n";
    sendSerialCommand(cmd);
  }

  void ledCommandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string cmd = msg->data + "\n";
    RCLCPP_INFO(this->get_logger(), "LED command: %s", msg->data.c_str());
    sendSerialCommand(cmd);
  }

  void sendSerialCommand(const std::string& command)
  {
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
      return;
    }

    ssize_t written = write(serial_fd_, command.c_str(), command.length());

    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
    }
  }

  // ===================== Member Variables =====================

  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;
  double wheel_radius_;
  std::string imu1_frame_id_;
  std::string imu2_frame_id_;
  std::string left_wheel_joint_;
  std::string right_wheel_joint_;
  std::string buffer_;

  ESP32Parser parser_;

  // Motor command state
  int16_t left_motor_rpm_ = 0;
  int16_t right_motor_rpm_ = 0;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr board_temp_pub_;
  rclcpp::Publisher<grace_msgs::msg::EnvironmentData>::SharedPtr environment_pub_;
  rclcpp::Publisher<grace_msgs::msg::AirQuality>::SharedPtr air_quality_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr left_motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr right_motor_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr led_cmd_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // CSV logging
  bool enable_csv_logging_ = true;
  std::string csv_log_dir_;
  std::string csv_file_path_;
  std::ofstream csv_file_;
  uint64_t csv_line_count_ = 0;
};

}  // namespace esp32_interface

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<esp32_interface::ESP32InterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
