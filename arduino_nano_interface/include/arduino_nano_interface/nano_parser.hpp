#pragma once

#include <string>
#include <cstdint>

namespace arduino_nano_interface
{

// -----------------------------------------------------------
// Parsed data from one compact machine line
// -----------------------------------------------------------
struct NanoData
{
  bool     valid        = false;
  uint32_t dt_ms        = 0;

  // Ultrasonic (cm, -1.0 = no echo)
  float    front_cm     = -1.0f;
  float    rear_cm      = -1.0f;
  float    left_cm      = -1.0f;
  float    right_cm     = -1.0f;

  // Voltage (Volts)
  float    battery_24v_v  = 0.0f;
  float    buck_19v_v     = 0.0f;

  // Current (Amperes)
  float    battery_40v_a  = 0.0f;
  float    battery_24v_a  = 0.0f;
  float    charger_40v_a  = 0.0f;
  float    charger_24v_a  = 0.0f;

  // Gas (normalised ratio 0.0-1.0)
  float    mq_ratio     = 0.0f;
  float    mhmq_ratio   = 0.0f;

  // Relay bitmask (bit0 = relay1/40V, bit1 = relay2/24V)
  int      relay_mask   = 0;
};

// -----------------------------------------------------------
// Parser — fixed-width integer parsing based on ESP32 string
// format, converted back to floats for ROS.
// -----------------------------------------------------------
class NanoParser
{
public:

  static NanoData parseLine(const std::string & line)
  {
    NanoData d;
    d.valid = false;

    if (line.empty() || line[0] != ',') {
      return d;
    }

    try {
      d.dt_ms = parseUnsignedField(line, "T", 4);

      // Ultrasonic (tenths-of-cm)
      d.front_cm  = parseSignedField(line, "Uf", 5) / 10.0f;
      d.rear_cm   = parseSignedField(line, "Ub", 5) / 10.0f;
      d.left_cm   = parseSignedField(line, "Ul", 5) / 10.0f;
      d.right_cm  = parseSignedField(line, "Ur", 5) / 10.0f;

      // Voltages (millivolts)
      d.battery_24v_v = parseSignedField(line, "V1", 5) / 1000.0f;
      d.buck_19v_v    = parseSignedField(line, "V2", 5) / 1000.0f;

      // Currents (milliamps)
      d.battery_40v_a = parseSignedField(line, "C1", 5) / 1000.0f;
      d.battery_24v_a = parseSignedField(line, "C2", 5) / 1000.0f;
      d.charger_40v_a = parseSignedField(line, "C3", 5) / 1000.0f;
      d.charger_24v_a = parseSignedField(line, "C4", 5) / 1000.0f;

      // Gas (raw 0-1023)
      d.mq_ratio   = parseUnsignedField(line, "MQ", 4) / 1023.0f;
      d.mhmq_ratio = parseUnsignedField(line, "M2", 4) / 1023.0f;

      d.relay_mask = static_cast<int>(parseUnsignedField(line, "RB", 1));

      d.valid = true;
    } catch (...) {
      d.valid = false;
    }

    return d;
  }

private:

  // Parse signed field: label + N digits + 'p' or 'n'
  static int32_t parseSignedField(const std::string & line, const std::string & label, int digits)
  {
    size_t pos = line.find(label);
    if (pos == std::string::npos) return 0;
    
    pos += label.length();
    if (pos + digits + 1 > line.length()) return 0;

    std::string val_str = line.substr(pos, digits);
    char sign = line[pos + digits];

    int32_t val = static_cast<int32_t>(std::stoi(val_str));
    if (sign == 'n') {
      val = -val;
    }
    return val;
  }

  // Parse unsigned field: label + N digits
  static uint32_t parseUnsignedField(const std::string & line, const std::string & label, int digits)
  {
    size_t pos = line.find(label);
    if (pos == std::string::npos) return 0;
    
    pos += label.length();
    if (pos + digits > line.length()) return 0;

    std::string val_str = line.substr(pos, digits);
    return static_cast<uint32_t>(std::stoul(val_str));
  }

};

}  // namespace arduino_nano_interface
