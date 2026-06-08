#ifndef ESP32_INTERFACE__ESP32_PARSER_HPP_
#define ESP32_INTERFACE__ESP32_PARSER_HPP_

#include <string>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace esp32_interface
{

/**
 * @brief Structure to hold parsed ESP32 data
 */
struct ESP32Data
{
  // Delta time
  uint32_t esp32_dt_ms = 0;

  // Driver status
  uint8_t driver_status = 0;       // 1 = driver alive, 0 = driver off

  // Hoverboard data
  int16_t battery_mv = 0;
  int16_t temperature_deci_c = 0;
  int16_t left_target_rpm = 0;
  int16_t left_measured_rpm = 0;
  int16_t right_target_rpm = 0;
  int16_t right_measured_rpm = 0;

  // IMU1 data
  uint8_t imu1_status = 0;
  int16_t ax1 = 0, ay1 = 0, az1 = 0;
  int16_t gx1 = 0, gy1 = 0, gz1 = 0;

  // IMU2 data
  uint8_t imu2_status = 0;
  int16_t ax2 = 0, ay2 = 0, az2 = 0;
  int16_t gx2 = 0, gy2 = 0, gz2 = 0;

  // BME680 environmental sensor
  int16_t bme_temperature = 0;      // °C (integer)
  int16_t bme_humidity = 0;         // % (integer)
  int16_t bme_pressure = 0;         // hPa (integer, truncated to int16 from 5 digits)
  int16_t bme_gas_resistance = 0;   // kΩ (integer)

  // PMS5003 particulate matter sensor
  uint8_t  pms_status = 0;         // 1 = connected, 0 = disconnected
  uint16_t pm1_0 = 0;              // PM1.0 µg/m³
  uint16_t pm2_5 = 0;              // PM2.5 µg/m³
  uint16_t pm10_0 = 0;             // PM10 µg/m³

  bool valid = false;
};

/**
 * @brief Fast parser for ESP32 serial data
 *
 * Uses string::find() + substr() instead of regex for performance at 200Hz.
 * Protocol format:
 *   ,T0005,D1,b36000p,t0250p,L0050p,Lm0048p,R0050p,Rm0049p,
 *   I11,ax100500p,...,I21,...,BT025p,H045p,P10135p,G015p,
 *   PM1,P1010,P2025,P3040\n
 */
class ESP32Parser
{
public:
  ESP32Parser() = default;

  /**
   * @brief Parse a line of ESP32 serial data
   */
  ESP32Data parseLine(const std::string& line)
  {
    ESP32Data data;
    data.valid = false;

    if (line.empty() || line[0] != ',') {
      return data;
    }

    try {
      // Core fields
      data.esp32_dt_ms = parseDtField(line, "T");
      data.driver_status = parseStatusField(line, "D");
      data.battery_mv = parseField(line, "b", 5);
      data.temperature_deci_c = parseField(line, "t", 4);
      data.left_target_rpm = parseField(line, "L", 4);
      data.left_measured_rpm = parseField(line, "Lm", 4);
      data.right_target_rpm = parseField(line, "R", 4);
      data.right_measured_rpm = parseField(line, "Rm", 4);

      // IMU1
      data.imu1_status = parseStatusField(line, "I1");
      data.ax1 = parseField(line, "ax1", 5);
      data.ay1 = parseField(line, "ay1", 5);
      data.az1 = parseField(line, "az1", 5);
      data.gx1 = parseField(line, "gx1", 5);
      data.gy1 = parseField(line, "gy1", 5);
      data.gz1 = parseField(line, "gz1", 5);

      // IMU2
      data.imu2_status = parseStatusField(line, "I2");
      data.ax2 = parseField(line, "ax2", 5);
      data.ay2 = parseField(line, "ay2", 5);
      data.az2 = parseField(line, "az2", 5);
      data.gx2 = parseField(line, "gx2", 5);
      data.gy2 = parseField(line, "gy2", 5);
      data.gz2 = parseField(line, "gz2", 5);

      // BME680
      data.bme_temperature = parseField(line, "BT", 3);
      data.bme_humidity = parseField(line, "H", 3);
      data.bme_pressure = parseField(line, "P", 5);
      data.bme_gas_resistance = parseField(line, "G", 3);

      // PMS5003
      data.pms_status = parseStatusField(line, "PM");
      data.pm1_0 = parseUnsignedField(line, "P1", 3);
      data.pm2_5 = parseUnsignedField(line, "P2", 3);
      data.pm10_0 = parseUnsignedField(line, "P3", 3);

      data.valid = true;
    } catch (...) {
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
    return (static_cast<double>(raw_value) / 131.0) * M_PI / 180.0;
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

private:
  /**
   * @brief Parse a fixed-width signed field: label + digits + 'p'/'n'
   * Uses fast string::find() instead of regex
   */
  int16_t parseField(const std::string& line, const std::string& label, int digits)
  {
    size_t pos = line.find(label);
    if (pos == std::string::npos) {
      return 0;
    }

    pos += label.length();
    if (pos + digits + 1 > line.length()) {
      return 0;
    }

    std::string value_str = line.substr(pos, digits);
    char sign = line[pos + digits];

    int16_t value = static_cast<int16_t>(std::stoi(value_str));
    if (sign == 'n') {
      value = -value;
    }

    return value;
  }

  /**
   * @brief Parse unsigned field: label + digits (no sign char, comma-terminated)
   */
  uint16_t parseUnsignedField(const std::string& line, const std::string& label, int digits)
  {
    size_t pos = line.find(label);
    if (pos == std::string::npos) {
      return 0;
    }

    pos += label.length();
    if (pos + digits > line.length()) {
      return 0;
    }

    std::string value_str = line.substr(pos, digits);
    return static_cast<uint16_t>(std::stoul(value_str));
  }

  /**
   * @brief Parse status field: label + single digit
   */
  uint8_t parseStatusField(const std::string& line, const std::string& label)
  {
    size_t pos = line.find(label);
    if (pos == std::string::npos) {
      return 0;
    }

    pos += label.length();
    if (pos >= line.length()) {
      return 0;
    }

    return line[pos] - '0';
  }

  /**
   * @brief Parse delta time field: label + 4 digits (unsigned, no sign)
   */
  uint32_t parseDtField(const std::string& line, const std::string& label)
  {
    size_t pos = line.find(label);
    if (pos == std::string::npos) {
      return 0;
    }

    pos += label.length();
    if (pos + 4 > line.length()) {
      return 0;
    }

    std::string value_str = line.substr(pos, 4);
    return static_cast<uint32_t>(std::stoul(value_str));
  }
};

}  // namespace esp32_interface

#endif  // ESP32_INTERFACE__ESP32_PARSER_HPP_
