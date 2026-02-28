#ifndef ESP32_INTERFACE__ESP32_PARSER_HPP_
#define ESP32_INTERFACE__ESP32_PARSER_HPP_

#include <string>
#include <regex>
#include <map>
#include <cmath>

namespace esp32_interface
{

/**
 * @brief Structure to hold parsed ESP32 data
 */
struct ESP32Data
{
  // Delta time
  uint32_t esp32_dt_ms;               // Delta time in ms from ESP32
  
  // Driver status
  uint8_t driver_status;          // 1 = driver alive, 0 = driver off
  
  // Hoverboard data
  int16_t battery_mv;           // Battery voltage in millivolts
  int16_t temperature_deci_c;   // Temperature in 0.1°C
  int16_t left_target_rpm;      // Left motor target speed in RPM
  int16_t left_measured_rpm;    // Left motor measured speed in RPM
  int16_t right_target_rpm;     // Right motor target speed in RPM
  int16_t right_measured_rpm;   // Right motor measured speed in RPM
  
  // IMU1 data
  uint8_t imu1_status;          // 1 = connected, 0 = disconnected
  int16_t ax1, ay1, az1;        // Accelerometer raw values
  int16_t gx1, gy1, gz1;        // Gyroscope raw values
  
  // IMU2 data
  uint8_t imu2_status;          // 1 = connected, 0 = disconnected
  int16_t ax2, ay2, az2;        // Accelerometer raw values
  int16_t gx2, gy2, gz2;        // Gyroscope raw values
  
  bool valid;                   // True if parsing was successful
};

/**
 * @brief Parser class for ESP32 serial data
 */
class ESP32Parser
{
public:
  ESP32Parser() = default;
  
  /**
   * @brief Parse a line of ESP32 serial data
   * @param line The serial line to parse
   * @return Parsed data structure
   */
  ESP32Data parseLine(const std::string& line)
  {
    ESP32Data data;
    data.valid = false;
    
    // Check if line starts with comma
    if (line.empty() || line[0] != ',') {
      return data;
    }
    
    try {
      // Delta time (T + 4 digits, unsigned)
      data.esp32_dt_ms = parseDtField(line, "T");
      
      // Driver status (D + 0/1)
      data.driver_status = parseStatusField(line, "D");
      
      // Battery (b + 5 digits + p/n)
      data.battery_mv = parseField(line, "b", 5);
      
      // Temperature (t + 4 digits + p/n)
      data.temperature_deci_c = parseField(line, "t", 4);
      
      // Motor speeds (4 digits + p/n each)
      data.left_target_rpm = parseField(line, "L", 4);
      data.left_measured_rpm = parseField(line, "Lm", 4);
      data.right_target_rpm = parseField(line, "R", 4);
      data.right_measured_rpm = parseField(line, "Rm", 4);
      
      // IMU1 status (no sign)
      data.imu1_status = parseStatusField(line, "I1");
      
      // IMU1 accelerometer (5 digits + p/n each)
      data.ax1 = parseField(line, "ax1", 5);
      data.ay1 = parseField(line, "ay1", 5);
      data.az1 = parseField(line, "az1", 5);
      
      // IMU1 gyroscope (5 digits + p/n each)
      data.gx1 = parseField(line, "gx1", 5);
      data.gy1 = parseField(line, "gy1", 5);
      data.gz1 = parseField(line, "gz1", 5);
      
      // IMU2 status (no sign)
      data.imu2_status = parseStatusField(line, "I2");
      
      // IMU2 accelerometer (5 digits + p/n each)
      data.ax2 = parseField(line, "ax2", 5);
      data.ay2 = parseField(line, "ay2", 5);
      data.az2 = parseField(line, "az2", 5);
      
      // IMU2 gyroscope (5 digits + p/n each)
      data.gx2 = parseField(line, "gx2", 5);
      data.gy2 = parseField(line, "gy2", 5);
      data.gz2 = parseField(line, "gz2", 5);
      
      data.valid = true;
    } catch (const std::exception& e) {
      data.valid = false;
    }
    
    return data;
  }
  
  /**
   * @brief Convert raw accelerometer value to m/s² (assuming ±2g range)
   * @param raw_value Raw ADC value from MPU6050
   * @return Acceleration in m/s²
   */
  static double rawAccelToMps2(int16_t raw_value)
  {
    // MPU6050 with ±2g range: 16384 LSB/g
    // 1g = 9.81 m/s²
    return (static_cast<double>(raw_value) / 16384.0) * 9.81;
  }
  
  /**
   * @brief Convert raw gyroscope value to rad/s (assuming ±250°/s range)
   * @param raw_value Raw ADC value from MPU6050
   * @return Angular velocity in rad/s
   */
  static double rawGyroToRadps(int16_t raw_value)
  {
    // MPU6050 with ±250°/s range: 131 LSB/(°/s)
    // Convert to rad/s
    double deg_per_sec = static_cast<double>(raw_value) / 131.0;
    return deg_per_sec * M_PI / 180.0;
  }
  
  /**
   * @brief Convert battery voltage to volts
   * @param millivolts Battery voltage in millivolts
   * @return Voltage in volts
   */
  static double batteryMvToVolts(int16_t millivolts)
  {
    return static_cast<double>(millivolts) / 1000.0;
  }
  
  /**
   * @brief Convert temperature to Celsius
   * @param deci_celsius Temperature in 0.1°C
   * @return Temperature in °C
   */
  static double tempDeciCToCelsius(int16_t deci_celsius)
  {
    return static_cast<double>(deci_celsius) / 10.0;
  }
  
  /**
   * @brief Convert RPM to rad/s
   * @param rpm Rotational speed in RPM
   * @return Angular velocity in rad/s
   */
  static double rpmToRadps(int16_t rpm)
  {
    return static_cast<double>(rpm) * 2.0 * M_PI / 60.0;
  }

private:
  /**
   * @brief Parse a fixed-width field with sign
   * @param line The full line to search
   * @param label Field label (e.g., "b", "t", "ax1")
   * @param digits Number of digits in the field
   * @return Parsed signed integer value
   */
  int16_t parseField(const std::string& line, const std::string& label, int digits)
  {
    std::string pattern = label + "(\\d{" + std::to_string(digits) + "})([pn])";
    std::regex re(pattern);
    std::smatch match;
    
    if (std::regex_search(line, match, re)) {
      int value = std::stoi(match[1].str());
      char sign = match[2].str()[0];
      return (sign == 'n') ? -value : value;
    }
    
    throw std::runtime_error("Field not found: " + label);
  }
  
  /**
   * @brief Parse IMU status field (no sign)
   * @param line The full line to search
   * @param label Field label (e.g., "I1", "I2")
   * @return Status value (0 or 1)
   */
  uint8_t parseStatusField(const std::string& line, const std::string& label)
  {
    std::string pattern = label + "(\\d)";
    std::regex re(pattern);
    std::smatch match;
    
    if (std::regex_search(line, match, re)) {
      return static_cast<uint8_t>(std::stoi(match[1].str()));
    }
    
    throw std::runtime_error("Status field not found: " + label);
  }
  
  /**
   * @brief Parse a delta time field (unsigned, 4 digits, no sign)
   * @param line The full line to search
   * @param label Field label (e.g., "T")
   * @return Parsed unsigned delta time value in milliseconds
   */
  uint32_t parseDtField(const std::string& line, const std::string& label)
  {
    std::string pattern = label + "(\\d{4})";
    std::regex re(pattern);
    std::smatch match;
    
    if (std::regex_search(line, match, re)) {
      return static_cast<uint32_t>(std::stoul(match[1].str()));
    }
    
    throw std::runtime_error("Delta time not found: " + label);
  }
};

}  // namespace esp32_interface

#endif  // ESP32_INTERFACE__ESP32_PARSER_HPP_
