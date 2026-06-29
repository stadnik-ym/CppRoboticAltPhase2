#include "../include/buzzer/buzzer_node.hpp"
#include <chrono>
#include <stdexcept>
#include <string>

namespace buzzer_node
{

BuzzerNode::BuzzerNode(const rclcpp::NodeOptions & options)
: Node("buzzer_node", options),
  chip_(nullptr),
  line_request_(nullptr),
  buzzing_(false)
{
  // --- Parameters ---
  this->declare_parameter<std::string>("gpio_chip", "gpiochip0");
  this->declare_parameter<int>("gpio_line", 18);          // BCM 17 = physical pin 11
  this->declare_parameter<int>("beep_duration_ms", 500);  // default beep length
  this->declare_parameter<int>("beep_frequency_hz", 2000);// PWM software frequency

  const std::string chip_name = this->get_parameter("gpio_chip").as_string();
  gpio_line_   = static_cast<unsigned int>(this->get_parameter("gpio_line").as_int());
  beep_duration_ms_   = this->get_parameter("beep_duration_ms").as_int();
  beep_frequency_hz_  = this->get_parameter("beep_frequency_hz").as_int();

  // --- Init GPIO ---
  init_gpio(chip_name);

  // --- Subscription ---
  sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "buzzer",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      this->buzzer_callback(msg);
    });

  RCLCPP_INFO(this->get_logger(),
    "BuzzerNode ready | chip=%s line=%u | beep=%dms @%dHz",
    chip_name.c_str(), gpio_line_, beep_duration_ms_, beep_frequency_hz_);
}

BuzzerNode::~BuzzerNode()
{
  stop_beep();

  if (beep_thread_.joinable()) {
    beep_thread_.join();
  }

  // libgpiod v2: resources freed by unique_ptr destructors automatically
  RCLCPP_INFO(this->get_logger(), "BuzzerNode shutting down, GPIO released.");
}

// ---------------------------------------------------------------------------
// GPIO init (libgpiod v2 API)
// ---------------------------------------------------------------------------
void BuzzerNode::init_gpio(const std::string & chip_name)
{
  chip_ = gpiod_chip_open("/dev/gpiochip");
  if (!chip_) {
    throw std::runtime_error("Cannot open GPIO chip: " + chip_name);
  }

  // Build a line settings object
  struct gpiod_line_settings * settings = gpiod_line_settings_new();
  if (!settings) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    throw std::runtime_error("Cannot allocate line settings");
  }

  gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

  // Build a line config
  struct gpiod_line_config * line_cfg = gpiod_line_config_new();
  if (!line_cfg) {
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    throw std::runtime_error("Cannot allocate line config");
  }

  const unsigned int offsets[1] = {gpio_line_};
  int ret = gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings);
  gpiod_line_settings_free(settings);

  if (ret < 0) {
    gpiod_line_config_free(line_cfg);
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    throw std::runtime_error("Cannot configure GPIO line");
  }

  // Request the lines
  struct gpiod_request_config * req_cfg = gpiod_request_config_new();
  if (req_cfg) {
    gpiod_request_config_set_consumer(req_cfg, "buzzer_node");
  }

  line_request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

  if (req_cfg) {
    gpiod_request_config_free(req_cfg);
  }
  gpiod_line_config_free(line_cfg);

  if (!line_request_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    throw std::runtime_error("Cannot request GPIO line " + std::to_string(gpio_line_));
  }
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------
void BuzzerNode::buzzer_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_DEBUG(this->get_logger(), "Buzz ON");
    start_beep();
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Buzz OFF");
    stop_beep();
  }
}

// ---------------------------------------------------------------------------
// Beep control
// ---------------------------------------------------------------------------
void BuzzerNode::start_beep()
{
  // If already buzzing — just reset the flag so the thread keeps going
  bool expected = false;
  if (!buzzing_.compare_exchange_strong(expected, true)) {
    return;  // already running
  }

  if (beep_thread_.joinable()) {
    beep_thread_.join();
  }

  beep_thread_ = std::thread([this]() {
    beep_loop();
  });
}

void BuzzerNode::stop_beep()
{
  buzzing_.store(false);
  // Set line LOW immediately
  set_gpio(false);
}

void BuzzerNode::beep_loop()
{
  // Software PWM: toggle pin at beep_frequency_hz_
  // half_period_us = 1_000_000 / (2 * freq)
  const int half_period_us = 1'000'000 / (2 * beep_frequency_hz_);

  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(beep_duration_ms_);

  bool level = false;
  while (buzzing_.load() && std::chrono::steady_clock::now() < deadline) {
    level = !level;
    set_gpio(level);
    std::this_thread::sleep_for(std::chrono::microseconds(half_period_us));
  }

  set_gpio(false);
  buzzing_.store(false);
}

void BuzzerNode::set_gpio(bool high)
{
  if (!line_request_) {
    return;
  }
  const enum gpiod_line_value val = high ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
  const unsigned int offsets[1] = {gpio_line_};
  const enum gpiod_line_value values[1] = {val};
  gpiod_line_request_set_values_subset(line_request_, 1, offsets, values);
}

}  // namespace buzzer_node

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<buzzer_node::BuzzerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
