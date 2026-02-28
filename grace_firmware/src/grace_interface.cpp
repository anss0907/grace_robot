#include "grace_firmware/grace_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <iomanip>


namespace grace_firmware
{
GraceInterface::GraceInterface()
{
}


GraceInterface::~GraceInterface()
{
  if (esp32_.IsOpen())
  {
    try
    {
      esp32_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("GraceInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn GraceInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("GraceInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());
  // No timestamp state needed — ESP32 sends dt directly

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> GraceInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> GraceInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn GraceInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("GraceInterface"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0 };
  position_states_ = { 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0 };
  read_buffer_ = "";

  try
  {
    esp32_.Open(port_);
    esp32_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("GraceInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }
  
  // Create ROS2 node for publishers
  node_ = rclcpp::Node::make_shared("grace_hardware_publishers");
  
  // Create publishers
  imu1_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/out", 10);
  imu2_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu2/data", 10);
  battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  temperature_pub_ = node_->create_publisher<sensor_msgs::msg::Temperature>("/temperature", 10);

  RCLCPP_INFO(rclcpp::get_logger("GraceInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn GraceInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("GraceInterface"), "Stopping robot hardware ...");

  if (esp32_.IsOpen())
  {
    try
    {
      esp32_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("GraceInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("GraceInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type GraceInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Read data from ESP32
  if(esp32_.IsDataAvailable())
  {
    // Read available data into buffer
    std::string temp;
    esp32_.ReadLine(temp);
    read_buffer_ += temp;
    
    // Process complete lines
    size_t pos;
    while((pos = read_buffer_.find('\n')) != std::string::npos)
    {
      std::string line = read_buffer_.substr(0, pos);
      read_buffer_.erase(0, pos + 1);
      
      // Remove carriage return if present
      if(!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }
      
      // Parse the line if it starts with comma (data line)
      if(!line.empty() && line[0] == ',')
      {
        ESP32Data data = parseESP32Line(line);
        
        if(data.valid)
        {
          // Use delta time sent directly from ESP32 (in ms)
          double dt = data.esp32_dt_ms / 1000.0;

          // Only use driver data when driver is alive
          if(data.driver_status == 1)
          {
            // Update wheel velocities (convert RPM to rad/s)
            // Right motor is physically mirrored on hoverboard chassis,
            // so its feedback sign is inverted relative to robot frame
            velocity_states_.at(0) = rpmToRadps(-data.right_measured_rpm);
            velocity_states_.at(1) = rpmToRadps(data.left_measured_rpm);
            
            // Update positions using ESP32-derived dt
            position_states_.at(0) += velocity_states_.at(0) * dt;
            position_states_.at(1) += velocity_states_.at(1) * dt;
            
            // Publish battery and temperature
            publishBattery(data);
            publishTemperature(data);
          }
          else
          {
            // Driver is off — zero out velocities
            velocity_states_.at(0) = 0.0;
            velocity_states_.at(1) = 0.0;
          }
          
          // Publish IMU data (IMUs are independent of driver)
          if(data.imu1_status == 1)
          {
            publishIMU1(data);
          }
          
          if(data.imu2_status == 1)
          {
            publishIMU2(data);
          }
        }
      }
    }
    
    // Prevent buffer from growing indefinitely
    if(read_buffer_.size() > 1024)
    {
      read_buffer_.clear();
    }
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type GraceInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Convert velocity commands (rad/s) to RPM
  int16_t right_rpm = static_cast<int16_t>(velocity_commands_.at(0) / RPM_TO_RADPS);
  int16_t left_rpm = static_cast<int16_t>(velocity_commands_.at(1) / RPM_TO_RADPS);
  
  // Send commands to ESP32 in the format: "l<rpm> r<rpm>\n"
  std::stringstream message_stream;
  message_stream << "l" << left_rpm << " r" << right_rpm << "\n";

  try
  {
    esp32_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("GraceInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
// Helper function implementations

int16_t GraceInterface::parseField(const std::string& line, const std::string& label, int digits)
{
  size_t pos = line.find(label);
  if(pos == std::string::npos)
  {
    return 0;
  }
  
  pos += label.length();
  if(pos + digits + 1 > line.length())
  {
    return 0;
  }
  
  std::string value_str = line.substr(pos, digits);
  char sign = line[pos + digits];
  
  int16_t value = std::stoi(value_str);
  if(sign == 'n')
  {
    value = -value;
  }
  
  return value;
}

uint32_t GraceInterface::parseDtField(const std::string& line, const std::string& label)
{
  size_t pos = line.find(label);
  if(pos == std::string::npos)
  {
    return 0;
  }
  
  pos += label.length();
  // Delta time is 4 digits, no sign character
  if(pos + 4 > line.length())
  {
    return 0;
  }
  
  std::string value_str = line.substr(pos, 4);
  return static_cast<uint32_t>(std::stoul(value_str));
}

uint8_t GraceInterface::parseStatusField(const std::string& line, const std::string& label)
{
  size_t pos = line.find(label);
  if(pos == std::string::npos)
  {
    return 0;
  }
  
  pos += label.length();
  if(pos >= line.length())
  {
    return 0;
  }
  
  return line[pos] - '0';
}

GraceInterface::ESP32Data GraceInterface::parseESP32Line(const std::string& line)
{
  ESP32Data data;
  data.valid = false;
  
  if(line.empty() || line[0] != ',')
  {
    return data;
  }
  
  try
  {
    // Parse all fields
    data.esp32_dt_ms = parseDtField(line, "T");
    data.driver_status = parseStatusField(line, "D");
    data.battery_mv = parseField(line, "b", 5);
    data.temperature_deci_c = parseField(line, "t", 4);
    data.left_measured_rpm = parseField(line, "Lm", 4);
    data.right_measured_rpm = parseField(line, "Rm", 4);
    
    data.imu1_status = parseStatusField(line, "I1");
    data.ax1 = parseField(line, "ax1", 5);
    data.ay1 = parseField(line, "ay1", 5);
    data.az1 = parseField(line, "az1", 5);
    data.gx1 = parseField(line, "gx1", 5);
    data.gy1 = parseField(line, "gy1", 5);
    data.gz1 = parseField(line, "gz1", 5);
    
    data.imu2_status = parseStatusField(line, "I2");
    data.ax2 = parseField(line, "ax2", 5);
    data.ay2 = parseField(line, "ay2", 5);
    data.az2 = parseField(line, "az2", 5);
    data.gx2 = parseField(line, "gx2", 5);
    data.gy2 = parseField(line, "gy2", 5);
    data.gz2 = parseField(line, "gz2", 5);
    
    data.valid = true;
  }
  catch(...)
  {
    data.valid = false;
  }
  
  return data;
}

double GraceInterface::rawAccelToMps2(int16_t raw_value)
{
  // MPU6050 with ±2g range: 16384 LSB/g
  return (static_cast<double>(raw_value) / ACCEL_SCALE) * G_TO_MPS2;
}

double GraceInterface::rawGyroToRadps(int16_t raw_value)
{
  // MPU6050 with ±250°/s range: 131 LSB/(°/s)
  return (static_cast<double>(raw_value) / GYRO_SCALE) * DEG_TO_RAD;
}

double GraceInterface::rpmToRadps(int16_t rpm)
{
  return static_cast<double>(rpm) * RPM_TO_RADPS;
}

void GraceInterface::publishIMU1(const ESP32Data& data)
{
  if(!imu1_pub_)
  {
    return;
  }
  
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "base_footprint";
  
  // Linear acceleration (in m/s²)
  msg.linear_acceleration.x = rawAccelToMps2(data.ax1);
  msg.linear_acceleration.y = rawAccelToMps2(data.ay1);
  msg.linear_acceleration.z = rawAccelToMps2(data.az1);
  
  // Angular velocity (in rad/s)
  msg.angular_velocity.x = rawGyroToRadps(data.gx1);
  msg.angular_velocity.y = rawGyroToRadps(data.gy1);
  msg.angular_velocity.z = rawGyroToRadps(data.gz1);
  
  // No orientation data (will be computed by filter)
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  msg.orientation_covariance[0] = -1.0;  // Mark as unavailable
  
  // Linear acceleration covariance
  msg.linear_acceleration_covariance[0] = 0.0001;
  msg.linear_acceleration_covariance[4] = 0.0001;
  msg.linear_acceleration_covariance[8] = 0.0001;
  
  // Angular velocity covariance
  msg.angular_velocity_covariance[0] = 0.0001;
  msg.angular_velocity_covariance[4] = 0.0001;
  msg.angular_velocity_covariance[8] = 0.0001;
  
  imu1_pub_->publish(msg);
}

void GraceInterface::publishIMU2(const ESP32Data& data)
{
  if(!imu2_pub_)
  {
    return;
  }
  
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "imu2_link";
  
  // Linear acceleration (in m/s²)
  msg.linear_acceleration.x = rawAccelToMps2(data.ax2);
  msg.linear_acceleration.y = rawAccelToMps2(data.ay2);
  msg.linear_acceleration.z = rawAccelToMps2(data.az2);
  
  // Angular velocity (in rad/s)
  msg.angular_velocity.x = rawGyroToRadps(data.gx2);
  msg.angular_velocity.y = rawGyroToRadps(data.gy2);
  msg.angular_velocity.z = rawGyroToRadps(data.gz2);
  
  // No orientation data (will be computed by filter)
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  msg.orientation_covariance[0] = -1.0;  // Mark as unavailable
  
  // Linear acceleration covariance
  msg.linear_acceleration_covariance[0] = 0.0001;
  msg.linear_acceleration_covariance[4] = 0.0001;
  msg.linear_acceleration_covariance[8] = 0.0001;
  
  // Angular velocity covariance
  msg.angular_velocity_covariance[0] = 0.0001;
  msg.angular_velocity_covariance[4] = 0.0001;
  msg.angular_velocity_covariance[8] = 0.0001;
  
  imu2_pub_->publish(msg);
}

void GraceInterface::publishBattery(const ESP32Data& data)
{
  if(!battery_pub_)
  {
    return;
  }
  
  auto msg = sensor_msgs::msg::BatteryState();
  msg.header.stamp = rclcpp::Clock().now();
  
  msg.voltage = static_cast<double>(data.battery_mv) / 1000.0;  // mV to V
  msg.temperature = static_cast<double>(data.temperature_deci_c) / 10.0;  // deci-C to C
  
  // Estimate battery percentage (36V nominal battery)
  // Full: ~42V, Empty: ~33V
  double voltage = msg.voltage;
  if(voltage >= 42.0)
  {
    msg.percentage = 1.0;
  }
  else if(voltage <= 33.0)
  {
    msg.percentage = 0.0;
  }
  else
  {
    msg.percentage = (voltage - 33.0) / (42.0 - 33.0);
  }
  
  // Set power supply status
  if(voltage < 35.0)
  {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }
  else
  {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
  
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  msg.present = true;
  
  battery_pub_->publish(msg);
}

void GraceInterface::publishTemperature(const ESP32Data& data)
{
  if(!temperature_pub_)
  {
    return;
  }
  
  auto msg = sensor_msgs::msg::Temperature();
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "base_footprint";
  
  msg.temperature = static_cast<double>(data.temperature_deci_c) / 10.0;  // deci-C to C
  msg.variance = 1.0;  // ±1°C variance
  
  temperature_pub_->publish(msg);
}

}  // namespace grace_firmware

PLUGINLIB_EXPORT_CLASS(grace_firmware::GraceInterface, hardware_interface::SystemInterface)