#ifndef GRACE_INTERFACE_HPP
#define GRACE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <map>


namespace grace_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GraceInterface : public hardware_interface::SystemInterface
{
public:
  GraceInterface();
  virtual ~GraceInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // ESP32 data structure
  struct ESP32Data
  {
    uint32_t esp32_dt_ms;             // Delta time in ms from ESP32
    uint8_t driver_status;        // 1 = driver alive, 0 = driver off
    int16_t battery_mv;
    int16_t temperature_deci_c;
    int16_t left_measured_rpm;
    int16_t right_measured_rpm;
    uint8_t imu1_status;
    int16_t ax1, ay1, az1;
    int16_t gx1, gy1, gz1;
    uint8_t imu2_status;
    int16_t ax2, ay2, az2;
    int16_t gx2, gy2, gz2;
    bool valid;
  };

  // Helper functions
  ESP32Data parseESP32Line(const std::string& line);
  uint32_t parseDtField(const std::string& line, const std::string& label);
  int16_t parseField(const std::string& line, const std::string& label, int digits);
  uint8_t parseStatusField(const std::string& line, const std::string& label);
  double rawAccelToMps2(int16_t raw_value);
  double rawGyroToRadps(int16_t raw_value);
  double rpmToRadps(int16_t rpm);
  void publishIMU1(const ESP32Data& data);
  void publishIMU2(const ESP32Data& data);
  void publishBattery(const ESP32Data& data);
  void publishTemperature(const ESP32Data& data);

  LibSerial::SerialPort esp32_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::string read_buffer_;
  
  // ROS2 Node and Publishers
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  
  // Conversion constants
  static constexpr double ACCEL_SCALE = 16384.0;  // ±2g range
  static constexpr double GYRO_SCALE = 131.0;     // ±250°/s range
  static constexpr double G_TO_MPS2 = 9.81;
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RPM_TO_RADPS = 2.0 * M_PI / 60.0;
};
}  // namespace grace_firmware


#endif  // GRACE_INTERFACE_HPP