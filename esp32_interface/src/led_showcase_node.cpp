#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class LedShowcaseNode : public rclcpp::Node
{
public:
  LedShowcaseNode() : Node("led_showcase_node"), current_pattern_(0), step_(0), ticks_(0)
  {
    // Publisher to send commands to the ESP32 node
    publisher_ = this->create_publisher<std_msgs::msg::String>("led_command", 10);
    
    // Timer fires every 100ms to drive the state machine patterns
    timer_ = this->create_wall_timer(
      100ms, std::bind(&LedShowcaseNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "LED Showcase Node Started!");
    RCLCPP_INFO(this->get_logger(), "Showcasing patterns on Strip 2...");
  }

private:
  void set_color(const std::string& color)
  {
    auto msg = std_msgs::msg::String();
    msg.data = "2" + color; // Always target Strip 2
    publisher_->publish(msg);
  }

  void timer_callback()
  {
    ticks_++;

    // Execute current pattern
    switch (current_pattern_) {
      case 0: pattern_rainbow(); break;
      case 1: pattern_police(); break;
      case 2: pattern_strobe(); break;
      case 3: pattern_heartbeat(); break;
      default: current_pattern_ = 0; break;
    }
    
    // Switch to the next pattern every 10 seconds (100 ticks @ 100ms each)
    if (ticks_ >= 100) {
      ticks_ = 0;
      step_ = 0;
      current_pattern_++;
      
      if (current_pattern_ > 3) {
        current_pattern_ = 0;
      }
      
      // Turn off LEDs briefly before switching patterns
      set_color("o");
      RCLCPP_INFO(this->get_logger(), "Switching to pattern %d", current_pattern_);
    }
  }

  // ---------------------------------------------------------
  // PATTERN 1: Rainbow Cycle (cycles all 7 colors)
  // ---------------------------------------------------------
  void pattern_rainbow()
  {
    // Change color every 500ms (5 ticks)
    if (ticks_ % 5 == 0) {
      const std::vector<std::string> colors = {"r", "y", "g", "c", "b", "m", "w"};
      set_color(colors[step_ % colors.size()]);
      step_++;
    }
  }

  // ---------------------------------------------------------
  // PATTERN 2: Police Lights (fast alternating Red/Blue)
  // ---------------------------------------------------------
  void pattern_police()
  {
    // Change color every 100ms (1 tick)
    if (step_ % 2 == 0) {
      set_color("r");
    } else {
      set_color("b");
    }
    step_++;
  }

  // ---------------------------------------------------------
  // PATTERN 3: Strobe Light (fast alternating White/Off)
  // ---------------------------------------------------------
  void pattern_strobe()
  {
    // Change color every 100ms (1 tick)
    if (step_ % 2 == 0) {
      set_color("w");
    } else {
      set_color("o");
    }
    step_++;
  }

  // ---------------------------------------------------------
  // PATTERN 4: Heartbeat (Double red blip, then pause)
  // ---------------------------------------------------------
  void pattern_heartbeat()
  {
    // 100ms Red -> 100ms Off -> 100ms Red -> 700ms Off
    // One full heartbeat cycle is 10 ticks (1000ms)
    int phase = ticks_ % 10;
    if (phase == 0 || phase == 2) {
      set_color("r");
    } else {
      set_color("o");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  int current_pattern_;
  int step_;
  uint64_t ticks_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedShowcaseNode>());
  rclcpp::shutdown();
  return 0;
}
