#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <grace_msgs/msg/ultrasonic_scan.hpp>
#include <grace_msgs/msg/gas_readings.hpp>
#include <grace_msgs/msg/power_monitor.hpp>
#include <grace_msgs/msg/charger_relay.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>

#include "arduino_nano_interface/nano_parser.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace arduino_nano_interface
{

class ArduinoNanoInterfaceNode : public rclcpp::Node
{
public:
  ArduinoNanoInterfaceNode()
  : Node("arduino_nano_interface_node"),
    serial_fd_(-1),
    buffer_("")
  {
    // ---- Parameters ----
    this->declare_parameter<std::string>("serial_port",        "/dev/grace_nano");
    this->declare_parameter<int>        ("baud_rate",          115200);
    this->declare_parameter<bool>       ("enable_csv_logging", true);
    this->declare_parameter<std::string>("csv_log_dir",        "");

    serial_port_       = this->get_parameter("serial_port").as_string();
    baud_rate_         = this->get_parameter("baud_rate").as_int();
    enable_csv_logging_ = this->get_parameter("enable_csv_logging").as_bool();
    csv_log_dir_       = this->get_parameter("csv_log_dir").as_string();

    // ---- Publishers ----
    ultrasonic_pub_ = this->create_publisher<grace_msgs::msg::UltrasonicScan>(
      "/nano/ultrasonic", 10);
    gas_pub_ = this->create_publisher<grace_msgs::msg::GasReadings>(
      "/nano/gas", 10);
    power_pub_ = this->create_publisher<grace_msgs::msg::PowerMonitor>(
      "/nano/power", 10);
    relay_pub_ = this->create_publisher<grace_msgs::msg::ChargerRelay>(
      "/nano/charger_relay", 10);

    // ---- Subscriber: relay command ----
    relay_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/nano/relay_command", 10,
      std::bind(&ArduinoNanoInterfaceNode::relayCommandCallback, this, std::placeholders::_1));

    // ---- CSV logging (gas + power only) ----
    if (enable_csv_logging_) {
      initCsvLogging();
    }

    // ---- Serial port ----
    if (!openSerialPort()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open serial port: %s", serial_port_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Arduino Nano Interface started on port: %s @ %d baud",
                serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(),
                "Publishing: /nano/ultrasonic  /nano/gas  /nano/power  /nano/charger_relay");
    RCLCPP_INFO(this->get_logger(),
                "Subscribing: /nano/relay_command  (Int8: 0=off 1=40V 2=24V 3=both)");

    if (enable_csv_logging_ && csv_file_.is_open()) {
      RCLCPP_INFO(this->get_logger(), "CSV logging → %s", csv_file_path_.c_str());
    }

    // ---- Read timer at 100 Hz (10 ms) ----
    timer_ = this->create_wall_timer(
      10ms, std::bind(&ArduinoNanoInterfaceNode::serialReadCallback, this));
  }

  ~ArduinoNanoInterfaceNode()
  {
    if (serial_fd_ >= 0) { close(serial_fd_); }
    if (csv_file_.is_open()) { csv_file_.close(); }
  }

private:

  // ===========================================================
  // CSV LOGGING (gas + power only — NO ultrasonic, NO relay)
  // ===========================================================

  void initCsvLogging()
  {
    std::string log_dir = csv_log_dir_.empty() ? getDefaultLogDir() : csv_log_dir_;

    try {
      fs::create_directories(log_dir);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to create log directory '%s': %s", log_dir.c_str(), e.what());
      return;
    }

    csv_file_path_ = log_dir + "/nano_data.csv";
    bool file_exists = fs::exists(csv_file_path_);
    csv_file_.open(csv_file_path_, std::ios::app);

    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open CSV file: %s", csv_file_path_.c_str());
      return;
    }

    if (!file_exists) {
      csv_file_ << "datetime,dt_ms,"
                << "battery_24v_v,buck_19v_v,"
                << "battery_40v_a,battery_24v_a,"
                << "charger_40v_a,charger_24v_a,"
                << "mq_ratio,mhmq_ratio\n";
      csv_file_.flush();
    }
  }

  std::string getDefaultLogDir()
  {
    const char * ws = std::getenv("COLCON_PREFIX_PATH");
    if (ws) {
      fs::path src_logs = fs::path(ws).parent_path() / "src" / "arduino_nano_interface" / "logs";
      if (fs::exists(src_logs.parent_path())) {
        return src_logs.string();
      }
    }
    return std::string(std::getenv("HOME")) + "/grace_ws/src/arduino_nano_interface/logs";
  }

  std::string getDateTimeString()
  {
    auto now       = std::chrono::system_clock::now();
    auto time_t_v  = std::chrono::system_clock::to_time_t(now);
    auto ms        = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;
    std::tm tm_buf;
    localtime_r(&time_t_v, &tm_buf);
    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
  }

  void logToCsv(const NanoData & d)
  {
    if (!csv_file_.is_open()) { return; }

    csv_file_ << getDateTimeString() << ","
              << d.dt_ms             << ","
              << d.battery_24v_v     << ","
              << d.buck_19v_v        << ","
              << d.battery_40v_a     << ","
              << d.battery_24v_a     << ","
              << d.charger_40v_a     << ","
              << d.charger_24v_a     << ","
              << d.mq_ratio          << ","
              << d.mhmq_ratio        << "\n";

    // Flush every 100 lines
    if ((++csv_line_count_ % 100) == 0) {
      csv_file_.flush();
    }
  }

  // ===========================================================
  // SERIAL PORT
  // ===========================================================

  bool openSerialPort()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) { return false; }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      close(serial_fd_); serial_fd_ = -1; return false;
    }

    speed_t speed = B115200;
    switch (baud_rate_) {
      case 9600:   speed = B9600;   break;
      case 57600:  speed = B57600;  break;
      case 115200: speed = B115200; break;
      default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |=  CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      close(serial_fd_); serial_fd_ = -1; return false;
    }
    tcflush(serial_fd_, TCIOFLUSH);
    return true;
  }

  // ===========================================================
  // SERIAL READ LOOP
  // ===========================================================

  void serialReadCallback()
  {
    if (serial_fd_ < 0) { return; }

    char buf[256];
    ssize_t n = read(serial_fd_, buf, sizeof(buf) - 1);

    if (n > 0) {
      buf[n] = '\0';
      buffer_ += std::string(buf);

      size_t pos;
      while ((pos = buffer_.find('\n')) != std::string::npos) {
        std::string line = buffer_.substr(0, pos);
        buffer_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') { line.pop_back(); }
        processLine(line);
      }

      if (buffer_.size() > 2048) { buffer_.clear(); }
    }
  }

  void processLine(const std::string & line)
  {
    // Only process compact machine lines (start with ',')
    if (line.empty() || line[0] != ',') { return; }

    NanoData d = NanoParser::parseLine(line);

    if (!d.valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Failed to parse Nano line: %s", line.c_str());
      return;
    }

    auto ts = this->now();

    publishUltrasonic(d, ts);
    publishGas(d, ts);
    publishPower(d, ts);
    publishChargerRelay(d, ts);

    if (enable_csv_logging_) { logToCsv(d); }
  }

  // ===========================================================
  // PUBLISHERS
  // ===========================================================

  void publishUltrasonic(const NanoData & d, const rclcpp::Time & ts)
  {
    auto msg = grace_msgs::msg::UltrasonicScan();
    msg.header.stamp    = ts;
    msg.header.frame_id = "base_footprint";
    msg.front_cm = d.front_cm;
    msg.rear_cm  = d.rear_cm;
    msg.left_cm  = d.left_cm;
    msg.right_cm = d.right_cm;
    ultrasonic_pub_->publish(msg);
  }

  void publishGas(const NanoData & d, const rclcpp::Time & ts)
  {
    auto msg = grace_msgs::msg::GasReadings();
    msg.header.stamp    = ts;
    msg.header.frame_id = "nano_link";
    msg.mq_ratio    = d.mq_ratio;
    msg.mhmq_ratio  = d.mhmq_ratio;
    gas_pub_->publish(msg);
  }

  void publishPower(const NanoData & d, const rclcpp::Time & ts)
  {
    auto msg = grace_msgs::msg::PowerMonitor();
    msg.header.stamp    = ts;
    msg.header.frame_id = "nano_link";
    msg.battery_24v_v  = d.battery_24v_v;
    msg.buck_19v_v     = d.buck_19v_v;
    msg.battery_40v_a  = d.battery_40v_a;
    msg.battery_24v_a  = d.battery_24v_a;
    msg.charger_40v_a  = d.charger_40v_a;
    msg.charger_24v_a  = d.charger_24v_a;
    power_pub_->publish(msg);
  }

  void publishChargerRelay(const NanoData & d, const rclcpp::Time & ts)
  {
    auto msg = grace_msgs::msg::ChargerRelay();
    msg.header.stamp    = ts;
    msg.header.frame_id = "nano_link";
    msg.charger_40v_on = (d.relay_mask & 0x01) != 0;
    msg.charger_24v_on = (d.relay_mask & 0x02) != 0;
    relay_pub_->publish(msg);
  }

  // ===========================================================
  // RELAY COMMAND SUBSCRIBER
  // ===========================================================

  void relayCommandCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    int val = static_cast<int>(msg->data);
    if (val < 0 || val > 3) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid relay command %d — valid range: 0-3", val);
      return;
    }

    char cmd[2] = {static_cast<char>('0' + val), '\n'};
    if (serial_fd_ >= 0) {
      if (write(serial_fd_, cmd, 2) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write relay command to serial port");
      } else {
        RCLCPP_INFO(this->get_logger(), "Relay command sent: %d", val);
      }
    }
  }

  // ===========================================================
  // MEMBER VARIABLES
  // ===========================================================

  int         serial_fd_;
  std::string serial_port_;
  int         baud_rate_;
  std::string buffer_;

  NanoParser  parser_;

  rclcpp::Publisher<grace_msgs::msg::UltrasonicScan>::SharedPtr  ultrasonic_pub_;
  rclcpp::Publisher<grace_msgs::msg::GasReadings>::SharedPtr     gas_pub_;
  rclcpp::Publisher<grace_msgs::msg::PowerMonitor>::SharedPtr    power_pub_;
  rclcpp::Publisher<grace_msgs::msg::ChargerRelay>::SharedPtr    relay_pub_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr           relay_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // CSV
  bool          enable_csv_logging_ = true;
  std::string   csv_log_dir_;
  std::string   csv_file_path_;
  std::ofstream csv_file_;
  uint64_t      csv_line_count_ = 0;
};

}  // namespace arduino_nano_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arduino_nano_interface::ArduinoNanoInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
