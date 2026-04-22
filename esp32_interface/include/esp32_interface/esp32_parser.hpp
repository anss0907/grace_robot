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
  
  // BME680 environmental sensor
  uint8_t bme_status;           // 1 = connected, 0 = disconnected
  int16_t bme_temperature;      // °C × 10 (e.g. 250 = 25.0°C)
  int16_t bme_humidity;         // % × 10  (e.g. 450 = 45.0%)
  int16_t bme_pressure;         // hPa × 10 (e.g. 10135 = 1013.5 hPa)
  int16_t bme_gas_resistance;   // kΩ × 10 (e.g. 150 = 15.0 kΩ)
  
  // PMS5003 particulate matter sensor
  uint8_t  pms_status;          // 1 = connected, 0 = disconnected
  uint16_t pm1_0;               // PM1.0 µg/m³
  uint16_t pm2_5;               // PM2.5 µg/m³
  uint16_t pm10_0;              // PM10 µg/m³
  
  // LED strip states (3 strips, each has R/G/B on/off)
  uint8_t strip1_r, strip1_g, strip1_b;
  uint8_t strip2_r, strip2_g, strip2_b;
  uint8_t strip3_r, strip3_g, strip3_b;
  
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
    ESP32Data data = {};
    data.valid = false;
    
    // Check if line starts with comma
    if (line.empty() || line[0] != ',') {
      return data;
    }
    
    try {
      // === Core fields (always present) ===
      data.esp32_dt_ms = parseDtField(line, "T");
      data.driver_status = parseStatusField(line, "D");
      data.battery_mv = parseField(line, "b", 5);
      data.temperature_deci_c = parseField(line, "t", 4);
      data.left_target_rpm = parseField(line, "L", 4);
      data.left_measured_rpm = parseField(line, "Lm", 4);
      data.right_target_rpm = parseField(line, "R", 4);
      data.right_measured_rpm = parseField(line, "Rm", 4);
      
      // === IMU1 (status always present, data only if connected) ===
      data.imu1_status = parseStatusField(line, "I1");
      if (data.imu1_status == 1) {
        data.ax1 = parseField(line, "ax1", 5);
        data.ay1 = parseField(line, "ay1", 5);
        data.az1 = parseField(line, "az1", 5);
        data.gx1 = parseField(line, "gx1", 5);
        data.gy1 = parseField(line, "gy1", 5);
        data.gz1 = parseField(line, "gz1", 5);
      }
      
      // === IMU2 (status always present, data only if connected) ===
      data.imu2_status = parseStatusField(line, "I2");
      if (data.imu2_status == 1) {
        data.ax2 = parseField(line, "ax2", 5);
        data.ay2 = parseField(line, "ay2", 5);
        data.az2 = parseField(line, "az2", 5);
        data.gx2 = parseField(line, "gx2", 5);
        data.gy2 = parseField(line, "gy2", 5);
        data.gz2 = parseField(line, "gz2", 5);
      }
      
      // === BME680 (optional — may not exist in old firmware) ===
      try {
        data.bme_status = parseStatusField(line, "BM");
        if (data.bme_status == 1) {
          data.bme_temperature = parseField(line, "BT", 5);
          data.bme_humidity = parseField(line, "BH", 5);
          data.bme_pressure = parseField(line, "BP", 5);
          data.bme_gas_resistance = parseField(line, "BG", 5);
        }
      } catch (...) {
        data.bme_status = 0;
      }
      
      // === PMS5003 (optional — may not exist in old firmware) ===
      try {
        data.pms_status = parseStatusField(line, "PM");
        if (data.pms_status == 1) {
          data.pm1_0 = parseUField(line, "P1", 5);
          data.pm2_5 = parseUField(line, "P2", 5);
          data.pm10_0 = parseUField(line, "P10", 5);
        }
      } catch (...) {
        data.pms_status = 0;
      }
      
      // === LED strip states (optional — may not exist in old firmware) ===
      try {
        parseLedStrip(line, "S1", data.strip1_r, data.strip1_g, data.strip1_b);
        parseLedStrip(line, "S2", data.strip2_r, data.strip2_g, data.strip2_b);
        parseLedStrip(line, "S3", data.strip3_r, data.strip3_g, data.strip3_b);
      } catch (...) {
        // LED data not available
      }
      
      data.valid = true;
    } catch (const std::exception& e) {
      data.valid = false;
    }
    
    return data;
  }
  
  // ======================== CONVERSION HELPERS ========================
  
  /** Convert raw accelerometer value to m/s² (±2g range, 16384 LSB/g) */
  static double rawAccelToMps2(int16_t raw_value)
  {
    return (static_cast<double>(raw_value) / 16384.0) * 9.81;
  }
  
  /** Convert raw gyroscope value to rad/s (±250°/s range, 131 LSB/(°/s)) */
  static double rawGyroToRadps(int16_t raw_value)
  {
    double deg_per_sec = static_cast<double>(raw_value) / 131.0;
    return deg_per_sec * M_PI / 180.0;
  }
  
  /** Battery millivolts to volts */
  static double batteryMvToVolts(int16_t millivolts)
  {
    return static_cast<double>(millivolts) / 1000.0;
  }
  
  /** Temperature deci-celsius to celsius */
  static double tempDeciCToCelsius(int16_t deci_celsius)
  {
    return static_cast<double>(deci_celsius) / 10.0;
  }
  
  /** RPM to rad/s */
  static double rpmToRadps(int16_t rpm)
  {
    return static_cast<double>(rpm) * 2.0 * M_PI / 60.0;
  }
  
  /** BME680 temperature (×10) to Celsius */
  static double bmeTempToCelsius(int16_t temp_x10)
  {
    return static_cast<double>(temp_x10) / 10.0;
  }
  
  /** BME680 humidity (×10) to percent */
  static double bmeHumToPercent(int16_t hum_x10)
  {
    return static_cast<double>(hum_x10) / 10.0;
  }
  
  /** BME680 pressure (×10) to hPa */
  static double bmePresToHpa(int16_t pres_x10)
  {
    return static_cast<double>(pres_x10) / 10.0;
  }
  
  /** BME680 pressure (×10) to Pascals (for FluidPressure msg) */
  static double bmePresToPascals(int16_t pres_x10)
  {
    return static_cast<double>(pres_x10) / 10.0 * 100.0;  // hPa -> Pa
  }
  
  /** BME680 gas resistance (×10) to kΩ */
  static double bmeGasToKohm(int16_t gas_x10)
  {
    return static_cast<double>(gas_x10) / 10.0;
  }

private:
  /**
   * @brief Parse a fixed-width signed field: label + digits + 'p'/'n'
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
   * @brief Parse a fixed-width unsigned field: label + digits + 'p'
   */
  uint16_t parseUField(const std::string& line, const std::string& label, int digits)
  {
    std::string pattern = label + "(\\d{" + std::to_string(digits) + "})p";
    std::regex re(pattern);
    std::smatch match;
    
    if (std::regex_search(line, match, re)) {
      return static_cast<uint16_t>(std::stoul(match[1].str()));
    }
    
    throw std::runtime_error("Unsigned field not found: " + label);
  }
  
  /**
   * @brief Parse status field: label + single digit
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
   * @brief Parse delta time field: label + 4 digits (unsigned, no sign)
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
  
  /**
   * @brief Parse LED strip state: label (e.g. "S1") + 3 binary digits
   */
  void parseLedStrip(const std::string& line, const std::string& label,
                     uint8_t& r, uint8_t& g, uint8_t& b)
  {
    std::string pattern = label + "([01])([01])([01])";
    std::regex re(pattern);
    std::smatch match;
    
    if (std::regex_search(line, match, re)) {
      r = static_cast<uint8_t>(std::stoi(match[1].str()));
      g = static_cast<uint8_t>(std::stoi(match[2].str()));
      b = static_cast<uint8_t>(std::stoi(match[3].str()));
      return;
    }
    
    throw std::runtime_error("LED strip not found: " + label);
  }
};

}  // namespace esp32_interface

#endif  // ESP32_INTERFACE__ESP32_PARSER_HPP_
