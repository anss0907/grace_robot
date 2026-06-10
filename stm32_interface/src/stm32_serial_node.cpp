#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <grace_msgs/msg/environment_data.hpp>
#include <vector>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b=0; b<8; ++b) crc = (crc & 0x8000) ? (crc<<1) ^ 0x1021 : (crc<<1);
  }
  return crc;
}

class STM32SerialNode : public rclcpp::Node {
public:
  STM32SerialNode() : Node("stm32_serial_node") {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baud", 921600);
    port_ = get_parameter("port").as_string();
    int baud = get_parameter("baud").as_int();

    // Create publishers
    env_pub_ = create_publisher<grace_msgs::msg::EnvironmentData>("/sensors/environment", 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("/sensors/imu/data_raw", 10);
    mag_raw_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/sensors/imu/mag_raw", 10);
    tof_pub_ = create_publisher<sensor_msgs::msg::Range>("/sensors/tof", 10);

    serial_.Open(port_);
    
    LibSerial::BaudRate baud_rate;
    switch(baud) {
      case 9600:   baud_rate = LibSerial::BaudRate::BAUD_9600; break;
      case 19200:  baud_rate = LibSerial::BaudRate::BAUD_19200; break;
      case 38400:  baud_rate = LibSerial::BaudRate::BAUD_38400; break;
      case 57600:  baud_rate = LibSerial::BaudRate::BAUD_57600; break;
      case 115200: baud_rate = LibSerial::BaudRate::BAUD_115200; break;
      case 230400: baud_rate = LibSerial::BaudRate::BAUD_230400; break;
      case 460800: baud_rate = LibSerial::BaudRate::BAUD_460800; break;
      case 921600: baud_rate = LibSerial::BaudRate::BAUD_921600; break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d. Using 115200.", baud);
        baud_rate = LibSerial::BaudRate::BAUD_115200;
        break;
    }
    
    serial_.SetBaudRate(baud_rate);
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_.SetVTime(50); // timeout in deciseconds

    timer_ = create_wall_timer(1ms, std::bind(&STM32SerialNode::pollSerial, this));
    
    RCLCPP_INFO(this->get_logger(), "STM32 Serial Node started on %s at %d baud", port_.c_str(), baud);
  }

  ~STM32SerialNode() { 
    if (serial_.IsOpen()) serial_.Close(); 
  }

private:
  LibSerial::SerialPort serial_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<uint8_t> buf_;
  std::string port_;

  rclcpp::Publisher<grace_msgs::msg::EnvironmentData>::SharedPtr env_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_pub_;

  void publishData(uint16_t seq, uint32_t ts, int32_t temp_mC, int32_t hum_mpermil, int32_t press_Pa,
                   int32_t ax_mg, int32_t ay_mg, int32_t az_mg, int32_t gx_mdps, int32_t gy_mdps, int32_t gz_mdps,
                   int32_t mx_mgss, int32_t my_mgss, int32_t mz_mgss, int32_t tof_mm) {
    
    auto current_time = this->now();

    // 1. Environment Data
    auto env_msg = grace_msgs::msg::EnvironmentData();
    env_msg.header.stamp = current_time;
    env_msg.header.frame_id = "stm32_env_link";
    env_msg.temperature = temp_mC / 1000.0f;
    env_msg.humidity = hum_mpermil / 10.0f;
    env_msg.pressure = press_Pa / 100.0f; // hPa
    env_msg.gas_resistance = 0.0f; // Not available from STM32
    env_msg.sensor_available = true;
    env_pub_->publish(env_msg);

    // 2. IMU Raw Data
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "imu_link";
    
    // Accel: mg to m/s^2
    imu_msg.linear_acceleration.x = ax_mg * 9.80665 / 1000.0;
    imu_msg.linear_acceleration.y = ay_mg * 9.80665 / 1000.0;
    imu_msg.linear_acceleration.z = az_mg * 9.80665 / 1000.0;
    
    // Gyro: mdps to rad/s
    imu_msg.angular_velocity.x = gx_mdps * M_PI / (180.0 * 1000.0);
    imu_msg.angular_velocity.y = gy_mdps * M_PI / (180.0 * 1000.0);
    imu_msg.angular_velocity.z = gz_mdps * M_PI / (180.0 * 1000.0);
    
    // No orientation available yet
    imu_msg.orientation_covariance[0] = -1.0;
    imu_raw_pub_->publish(imu_msg);

    // 3. Magnetic Field Raw Data
    auto mag_msg = sensor_msgs::msg::MagneticField();
    mag_msg.header.stamp = current_time;
    mag_msg.header.frame_id = "imu_link";
    
    // Mag: mgauss to Tesla
    mag_msg.magnetic_field.x = mx_mgss / 1000.0 / 10000.0;
    mag_msg.magnetic_field.y = my_mgss / 1000.0 / 10000.0;
    mag_msg.magnetic_field.z = mz_mgss / 1000.0 / 10000.0;
    mag_raw_pub_->publish(mag_msg);

    // 4. Time of Flight (ToF)
    auto tof_msg = sensor_msgs::msg::Range();
    tof_msg.header.stamp = current_time;
    tof_msg.header.frame_id = "tof_link";
    tof_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    tof_msg.field_of_view = 0.436332; // ~25 degrees
    tof_msg.min_range = 0.03;
    tof_msg.max_range = 2.0;
    tof_msg.range = tof_mm / 1000.0f; // mm to meters
    tof_pub_->publish(tof_msg);
  }

  void pollSerial() {
    while (serial_.IsDataAvailable()) {
      char c;
      serial_.ReadByte(c, 5);
      buf_.push_back(static_cast<uint8_t>(c));
    }
    parseFrames();
  }

  static bool findSOF(const std::vector<uint8_t>& b, size_t& idx) {
    for (; idx + 1 < b.size(); ++idx) {
      if (b[idx] == 0xAA && b[idx+1] == 0x55) return true;
    }
    return false;
  }

  void parseFrames() {
    const size_t HDR_SIZE = 12; // SOF2+VER1+ID1+SEQ2+TS4+LEN2
    size_t i = 0;
    while (true) {
      if (buf_.size() - i < HDR_SIZE) {
        buf_.erase(buf_.begin(), buf_.begin() + i);
        return;
      }
      if (!(buf_[i]==0xAA && buf_[i+1]==0x55)) {
        if (!findSOF(buf_, i)) {
          buf_.clear();
          return;
        }
      }
      if (buf_.size() - i < HDR_SIZE) {
        buf_.erase(buf_.begin(), buf_.begin() + i);
        return;
      }
      
      uint8_t msgid = buf_[i+3];
      uint16_t seq  = (uint16_t)buf_[i+4] | ((uint16_t)buf_[i+5] << 8);
      uint32_t ts   = (uint32_t)buf_[i+6] | ((uint32_t)buf_[i+7]<<8) | ((uint32_t)buf_[i+8]<<16) | ((uint32_t)buf_[i+9]<<24);
      uint16_t len  = (uint16_t)buf_[i+10] | ((uint16_t)buf_[i+11] << 8);

      const size_t frame_len = HDR_SIZE + len + 2; // +CRC
      if (len > 256 || frame_len > 4096) {
        i += 2;
        continue;
      }
      if (buf_.size() - i < frame_len) {
        buf_.erase(buf_.begin(), buf_.begin() + i);
        return;
      }
      
      uint16_t rx_crc = (uint16_t)buf_[i+HDR_SIZE+len] | ((uint16_t)buf_[i+HDR_SIZE+len+1] << 8);
      uint16_t crc = 0xFFFF;
      crc = crc16_ccitt(&buf_[i], HDR_SIZE, crc);
      crc = crc16_ccitt(&buf_[i+HDR_SIZE], len, crc);
      if (crc != rx_crc) {
        i += 2;
        continue;
      }

      if (msgid == 0x01 && len == 52) {
        const uint8_t* p = &buf_[i+HDR_SIZE];
        auto rd32 = [&](int k)->int32_t {
          size_t o = 4*k;
          return (int32_t)((uint32_t)p[o] | ((uint32_t)p[o+1]<<8) | ((uint32_t)p[o+2]<<16) | ((uint32_t)p[o+3]<<24));
        };
        int32_t temp_mC     = rd32(0);
        int32_t hum_mpermil = rd32(1);
        int32_t press_Pa    = rd32(2);
        int32_t ax_mg       = rd32(3);
        int32_t ay_mg       = rd32(4);
        int32_t az_mg       = rd32(5);
        int32_t gx_mdps     = rd32(6);
        int32_t gy_mdps     = rd32(7);
        int32_t gz_mdps     = rd32(8);
        int32_t mx_mgss     = rd32(9);
        int32_t my_mgss     = rd32(10);
        int32_t mz_mgss     = rd32(11);
        int32_t tof_mm      = rd32(12);

        publishData(seq, ts, temp_mC, hum_mpermil, press_Pa,
                    ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps,
                    mx_mgss, my_mgss, mz_mgss, tof_mm);
      }

      i += frame_len;
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<STM32SerialNode>();
  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}
