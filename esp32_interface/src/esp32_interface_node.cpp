#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <string>
#include <sstream>
#include <memory>
#include <chrono>

#include "esp32_interface/esp32_parser.hpp"

using namespace std::chrono_literals;

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
    
    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    imu1_frame_id_ = this->get_parameter("imu1_frame_id").as_string();
    imu2_frame_id_ = this->get_parameter("imu2_frame_id").as_string();
    left_wheel_joint_ = this->get_parameter("left_wheel_joint").as_string();
    right_wheel_joint_ = this->get_parameter("right_wheel_joint").as_string();
    
    // ===================== Publishers =====================
    
    // Existing
    imu1_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu1/data", 10);
    imu2_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu2/data", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    board_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("board_temperature", 10);
    
    // BME680 environmental sensor
    bme_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("bme680/temperature", 10);
    bme_humidity_pub_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("bme680/humidity", 10);
    bme_pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("bme680/pressure", 10);
    bme_gas_pub_ = this->create_publisher<std_msgs::msg::Float32>("bme680/gas_resistance", 10);
    
    // PMS5003 air quality sensor
    pm1_0_pub_ = this->create_publisher<std_msgs::msg::UInt16>("air_quality/pm1_0", 10);
    pm2_5_pub_ = this->create_publisher<std_msgs::msg::UInt16>("air_quality/pm2_5", 10);
    pm10_pub_ = this->create_publisher<std_msgs::msg::UInt16>("air_quality/pm10", 10);
    
    // LED status
    led_status_pub_ = this->create_publisher<std_msgs::msg::String>("led_status", 10);
    
    // ===================== Subscribers =====================
    
    // Motor commands
    left_motor_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "left_motor_rpm", 10,
      std::bind(&ESP32InterfaceNode::leftMotorCallback, this, std::placeholders::_1));
    
    right_motor_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "right_motor_rpm", 10,
      std::bind(&ESP32InterfaceNode::rightMotorCallback, this, std::placeholders::_1));
    
    // LED commands (e.g. "1r", "2g", "3b", "1w", "a")
    led_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "led_command", 10,
      std::bind(&ESP32InterfaceNode::ledCommandCallback, this, std::placeholders::_1));
    
    // Open serial port
    if (!openSerialPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "ESP32 Interface Node started on port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing: imu1/data, imu2/data, joint_states, battery_state, board_temperature");
    RCLCPP_INFO(this->get_logger(), "Publishing: bme680/temperature, bme680/humidity, bme680/pressure, bme680/gas_resistance");
    RCLCPP_INFO(this->get_logger(), "Publishing: air_quality/pm1_0, air_quality/pm2_5, air_quality/pm10");
    RCLCPP_INFO(this->get_logger(), "Publishing: led_status");
    RCLCPP_INFO(this->get_logger(), "Subscribing: left_motor_rpm, right_motor_rpm, led_command");
    
    // Timer for reading serial data (100Hz)
    timer_ = this->create_wall_timer(
      10ms, std::bind(&ESP32InterfaceNode::serialReadCallback, this));
  }
  
  ~ESP32InterfaceNode()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
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
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default: speed = B115200; break;
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
    
    // --- BME680 environmental data ---
    if (data.bme_status == 1) {
      publishBME680(data, timestamp);
    }
    
    // --- PMS5003 air quality data ---
    if (data.pms_status == 1) {
      publishPMS5003(data, timestamp);
    }
    
    // --- LED status (always publish if data valid) ---
    publishLedStatus(data, timestamp);
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
    
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;
    
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
    
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
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
  
  // ===================== BME680 Publishers =====================
  
  void publishBME680(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    // Temperature
    {
      auto msg = sensor_msgs::msg::Temperature();
      msg.header.stamp = timestamp;
      msg.header.frame_id = "bme680_link";
      msg.temperature = ESP32Parser::bmeTempToCelsius(data.bme_temperature);
      msg.variance = 0.5;  // BME680 accuracy ±0.5°C
      bme_temp_pub_->publish(msg);
    }
    
    // Relative Humidity
    {
      auto msg = sensor_msgs::msg::RelativeHumidity();
      msg.header.stamp = timestamp;
      msg.header.frame_id = "bme680_link";
      msg.relative_humidity = ESP32Parser::bmeHumToPercent(data.bme_humidity) / 100.0;  // 0.0-1.0 range
      msg.variance = 0.03;  // BME680 accuracy ±3%
      bme_humidity_pub_->publish(msg);
    }
    
    // Fluid Pressure
    {
      auto msg = sensor_msgs::msg::FluidPressure();
      msg.header.stamp = timestamp;
      msg.header.frame_id = "bme680_link";
      msg.fluid_pressure = ESP32Parser::bmePresToPascals(data.bme_pressure);  // in Pascals
      msg.variance = 100.0;  // BME680 accuracy ±1 hPa = 100 Pa
      bme_pressure_pub_->publish(msg);
    }
    
    // Gas Resistance
    {
      auto msg = std_msgs::msg::Float32();
      msg.data = static_cast<float>(ESP32Parser::bmeGasToKohm(data.bme_gas_resistance));
      bme_gas_pub_->publish(msg);
    }
  }
  
  // ===================== PMS5003 Publishers =====================
  
  void publishPMS5003(const ESP32Data& data, const rclcpp::Time& /* timestamp */)
  {
    {
      auto msg = std_msgs::msg::UInt16();
      msg.data = data.pm1_0;
      pm1_0_pub_->publish(msg);
    }
    {
      auto msg = std_msgs::msg::UInt16();
      msg.data = data.pm2_5;
      pm2_5_pub_->publish(msg);
    }
    {
      auto msg = std_msgs::msg::UInt16();
      msg.data = data.pm10_0;
      pm10_pub_->publish(msg);
    }
  }
  
  // ===================== LED Publishers =====================
  
  void publishLedStatus(const ESP32Data& data, const rclcpp::Time& /* timestamp */)
  {
    auto msg = std_msgs::msg::String();
    std::ostringstream ss;
    ss << "S1:" << (int)data.strip1_r << (int)data.strip1_g << (int)data.strip1_b
       << " S2:" << (int)data.strip2_r << (int)data.strip2_g << (int)data.strip2_b
       << " S3:" << (int)data.strip3_r << (int)data.strip3_g << (int)data.strip3_b;
    msg.data = ss.str();
    led_status_pub_->publish(msg);
  }
  
  // ===================== Subscriber Callbacks =====================
  
  void leftMotorCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    int16_t rpm = msg->data;
    RCLCPP_INFO(this->get_logger(), "Left motor command: %d RPM", rpm);
    sendSerialCommand("l" + std::to_string(rpm) + "\n");
  }
  
  void rightMotorCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    int16_t rpm = msg->data;
    RCLCPP_INFO(this->get_logger(), "Right motor command: %d RPM", rpm);
    sendSerialCommand("r" + std::to_string(rpm) + "\n");
  }
  
  void ledCommandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Forward LED command directly to ESP32 (e.g. "1r", "2g", "3b", "a")
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
  
  // Existing publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr board_temp_pub_;
  
  // BME680 publishers
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr bme_temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr bme_humidity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr bme_pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bme_gas_pub_;
  
  // PMS5003 publishers
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pm1_0_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pm2_5_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pm10_pub_;
  
  // LED publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr led_status_pub_;
  
  // Motor subscribers
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr left_motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr right_motor_sub_;
  
  // LED subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr led_cmd_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
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
