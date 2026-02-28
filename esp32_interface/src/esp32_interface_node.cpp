#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/int16.hpp>

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
    this->declare_parameter<double>("wheel_radius", 0.065);  // meters
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
    
    // Create publishers
    imu1_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu1/data", 10);
    imu2_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu2/data", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    
    // Create subscribers for direct motor commands (RPM)
    left_motor_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "left_motor_rpm", 10,
      std::bind(&ESP32InterfaceNode::leftMotorCallback, this, std::placeholders::_1));
    
    right_motor_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "right_motor_rpm", 10,
      std::bind(&ESP32InterfaceNode::rightMotorCallback, this, std::placeholders::_1));
    
    // Open serial port
    if (!openSerialPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "ESP32 Interface Node started on port: %s", serial_port_.c_str());
    
    // Create timer for reading serial data (100Hz to match ESP32 output rate)
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
    
    // Configure serial port
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
    
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
    
    tty.c_lflag &= ~ICANON;        // Non-canonical mode
    tty.c_lflag &= ~ECHO;          // Disable echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;          // Disable signal chars
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                    // No software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    tty.c_oflag &= ~OPOST;         // No output processing
    tty.c_oflag &= ~ONLCR;
    
    tty.c_cc[VTIME] = 0;           // No blocking
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    
    // Flush any existing data
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
      
      // Process complete lines
      size_t pos;
      while ((pos = buffer_.find('\n')) != std::string::npos) {
        std::string line = buffer_.substr(0, pos);
        buffer_.erase(0, pos + 1);
        
        // Remove carriage return if present
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        
        // Process the line
        processLine(line);
      }
      
      // Prevent buffer from growing indefinitely
      if (buffer_.size() > 1024) {
        buffer_.clear();
      }
    }
  }
  
  void processLine(const std::string& line)
  {
    // Skip empty lines or lines that don't start with comma (non-data lines)
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
    
    // Publish IMU data (IMUs are independent of driver)
    if (data.imu1_status == 1) {
      publishImu1(data, timestamp);
    }
    
    if (data.imu2_status == 1) {
      publishImu2(data, timestamp);
    }
    
    // Only publish driver-dependent data when driver is alive
    if (data.driver_status == 1) {
      publishJointStates(data, timestamp);
      publishBatteryState(data, timestamp);
      publishTemperature(data, timestamp);
    }
  }
  
  void publishImu1(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = timestamp;
    msg.header.frame_id = imu1_frame_id_;
    
    // Linear acceleration (in m/s²)
    msg.linear_acceleration.x = ESP32Parser::rawAccelToMps2(data.ax1);
    msg.linear_acceleration.y = ESP32Parser::rawAccelToMps2(data.ay1);
    msg.linear_acceleration.z = ESP32Parser::rawAccelToMps2(data.az1);
    
    // Angular velocity (in rad/s)
    msg.angular_velocity.x = ESP32Parser::rawGyroToRadps(data.gx1);
    msg.angular_velocity.y = ESP32Parser::rawGyroToRadps(data.gy1);
    msg.angular_velocity.z = ESP32Parser::rawGyroToRadps(data.gz1);
    
    // No orientation data available from MPU6050 (raw gyro + accel only)
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;  // Mark as unavailable (use filter to compute)
    
    // Linear acceleration covariance (MPU6050 noise ~0.01 m/s²)
    // Covariance matrix is 3x3 in row-major order [xx, xy, xz, yx, yy, yz, zx, zy, zz]
    msg.linear_acceleration_covariance[0] = 0.0001;  // xx
    msg.linear_acceleration_covariance[1] = 0.0;
    msg.linear_acceleration_covariance[2] = 0.0;
    msg.linear_acceleration_covariance[3] = 0.0;
    msg.linear_acceleration_covariance[4] = 0.0001;  // yy
    msg.linear_acceleration_covariance[5] = 0.0;
    msg.linear_acceleration_covariance[6] = 0.0;
    msg.linear_acceleration_covariance[7] = 0.0;
    msg.linear_acceleration_covariance[8] = 0.0001;  // zz
    
    // Angular velocity covariance (MPU6050 noise ~0.01 rad/s)
    msg.angular_velocity_covariance[0] = 0.0001;  // xx
    msg.angular_velocity_covariance[1] = 0.0;
    msg.angular_velocity_covariance[2] = 0.0;
    msg.angular_velocity_covariance[3] = 0.0;
    msg.angular_velocity_covariance[4] = 0.0001;  // yy
    msg.angular_velocity_covariance[5] = 0.0;
    msg.angular_velocity_covariance[6] = 0.0;
    msg.angular_velocity_covariance[7] = 0.0;
    msg.angular_velocity_covariance[8] = 0.0001;  // zz
    
    imu1_pub_->publish(msg);
  }
  
  void publishImu2(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = timestamp;
    msg.header.frame_id = imu2_frame_id_;
    
    // Linear acceleration (in m/s²)
    msg.linear_acceleration.x = ESP32Parser::rawAccelToMps2(data.ax2);
    msg.linear_acceleration.y = ESP32Parser::rawAccelToMps2(data.ay2);
    msg.linear_acceleration.z = ESP32Parser::rawAccelToMps2(data.az2);
    
    // Angular velocity (in rad/s)
    msg.angular_velocity.x = ESP32Parser::rawGyroToRadps(data.gx2);
    msg.angular_velocity.y = ESP32Parser::rawGyroToRadps(data.gy2);
    msg.angular_velocity.z = ESP32Parser::rawGyroToRadps(data.gz2);
    
    // No orientation data available from MPU6050 (raw gyro + accel only)
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;  // Mark as unavailable (use filter to compute)
    
    // Linear acceleration covariance (MPU6050 noise ~0.01 m/s²)
    msg.linear_acceleration_covariance[0] = 0.0001;  // xx
    msg.linear_acceleration_covariance[1] = 0.0;
    msg.linear_acceleration_covariance[2] = 0.0;
    msg.linear_acceleration_covariance[3] = 0.0;
    msg.linear_acceleration_covariance[4] = 0.0001;  // yy
    msg.linear_acceleration_covariance[5] = 0.0;
    msg.linear_acceleration_covariance[6] = 0.0;
    msg.linear_acceleration_covariance[7] = 0.0;
    msg.linear_acceleration_covariance[8] = 0.0001;  // zz
    
    // Angular velocity covariance (MPU6050 noise ~0.01 rad/s)
    msg.angular_velocity_covariance[0] = 0.0001;  // xx
    msg.angular_velocity_covariance[1] = 0.0;
    msg.angular_velocity_covariance[2] = 0.0;
    msg.angular_velocity_covariance[3] = 0.0;
    msg.angular_velocity_covariance[4] = 0.0001;  // yy
    msg.angular_velocity_covariance[5] = 0.0;
    msg.angular_velocity_covariance[6] = 0.0;
    msg.angular_velocity_covariance[7] = 0.0;
    msg.angular_velocity_covariance[8] = 0.0001;  // zz
    
    imu2_pub_->publish(msg);
  }
  
  void publishJointStates(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = timestamp;
    
    msg.name.push_back(left_wheel_joint_);
    msg.name.push_back(right_wheel_joint_);
    
    // Velocity in rad/s
    msg.velocity.push_back(ESP32Parser::rpmToRadps(data.left_measured_rpm));
    msg.velocity.push_back(ESP32Parser::rpmToRadps(data.right_measured_rpm));
    
    // We don't have position or effort data
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
    
    // Estimate battery percentage (assuming 36V nominal battery)
    // Full charge: ~42V, Empty: ~33V
    double voltage = msg.voltage;
    if (voltage >= 42.0) {
      msg.percentage = 1.0;
    } else if (voltage <= 33.0) {
      msg.percentage = 0.0;
    } else {
      msg.percentage = (voltage - 33.0) / (42.0 - 33.0);
    }
    
    // Set power supply status
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
  
  void publishTemperature(const ESP32Data& data, const rclcpp::Time& timestamp)
  {
    auto msg = sensor_msgs::msg::Temperature();
    msg.header.stamp = timestamp;
    msg.header.frame_id = "base_footprint";
    
    msg.temperature = ESP32Parser::tempDeciCToCelsius(data.temperature_deci_c);
    msg.variance = 1.0;  // ±1°C variance for board temperature sensor
    
    temperature_pub_->publish(msg);
  }
  
  void leftMotorCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    // Positive = forward, Negative = reverse
    int16_t rpm = msg->data;
    RCLCPP_INFO(this->get_logger(), "Left motor command: %d RPM", rpm);
    sendLeftMotorCommand(rpm);
  }
  
  void rightMotorCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    // Positive = forward, Negative = reverse
    int16_t rpm = msg->data;
    RCLCPP_INFO(this->get_logger(), "Right motor command: %d RPM", rpm);
    sendRightMotorCommand(rpm);
  }
  
  void sendLeftMotorCommand(int16_t rpm)
  {
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
      return;
    }
    
    // Format: "l<rpm>\n"
    std::string command = "l" + std::to_string(rpm) + "\n";
    
    RCLCPP_INFO(this->get_logger(), "Sending to ESP32: %s", command.c_str());
    
    ssize_t written = write(serial_fd_, command.c_str(), command.length());
    
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent %ld bytes", written);
    }
  }
  
  void sendRightMotorCommand(int16_t rpm)
  {
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
      return;
    }
    
    // Format: "r<rpm>\n"
    std::string command = "r" + std::to_string(rpm) + "\n";
    
    RCLCPP_INFO(this->get_logger(), "Sending to ESP32: %s", command.c_str());
    
    ssize_t written = write(serial_fd_, command.c_str(), command.length());
    
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent %ld bytes", written);
    }
  }

  // Member variables
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
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr left_motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr right_motor_sub_;
  
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
