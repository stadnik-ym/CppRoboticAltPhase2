#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace buzzer_node
{

class BuzzerNode : public rclcpp::Node
{
public:
  explicit BuzzerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BuzzerNode() override;

private:
  // GPIO
  void init_gpio(const std::string & chip_name);
  void set_gpio(bool high);

  // Beep logic
  void buzzer_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void start_beep();
  void stop_beep();
  void beep_loop();

  // libgpiod v2 handles (raw pointers — lifetime tied to node)
  struct gpiod_chip *           chip_;
  struct gpiod_line_request *   line_request_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  // Config
  unsigned int gpio_line_;
  int          beep_duration_ms_;
  int          beep_frequency_hz_;

  // Beep thread
  std::thread  beep_thread_;
  std::atomic<bool> buzzing_;
};

}  // namespace buzzer_node
