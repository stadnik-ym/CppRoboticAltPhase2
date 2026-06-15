// #include <functional>
// #include <memory>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/nav_msgs/msg/odometry.hpp>
#include <vector>
#include "../include/diff_drive/artemka.hpp"
#include "geometry_msgs/msg/twist.hpp"

extern {
    #include <lgpio.h>
}

class MotorNode : public rclcpp::Node
{
    public:
    MotorNode() : Node("motor_node")
    {
    RCLCPP_INFO(this->get_logger(), "Мотор нода була запущена");
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&MotorNode::cmd_cb, this, std::placeholders::_1));
    publisher_ = this->create_publisher<nav_msgs::nav_msgs::msg::Odometry>("/odom", 10);
    }
    private:

    void setup_gpio()
    {
        pins = std::array<int, 6>{artemka::ENA, artemka::IN1, artemka::IN2, artemka::ENB, artemka::IN3, artemka::IN4};

        for (auto x : pins)
        {
            lgGpioClaimOutput(this->gpio_handle, 0, p, 0);
        }
        this->stop();
    }

    void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        v = std::clamp(static_cast<float>(msg->linear.x), -artemka::MAX_LINEAR, artemka::MAX_LINEAR);
        w = std::clamp(static_cast<float>(msg->linear.z), -artemka::MAX_ANGULAR, artemka::MAX_ANGULAR);

        last_cmd = this->get_clock()->now();

        if (std::abs(v) < 0.001f && std::abs(w) < 0.001f)
        {
            this->stop();
        }
    }

    int pwm_from_speed(float s, int min_pwm)
    {
       float speed = std::abs(s);
       if (speed < 0.001f)
       {
           return 0;
       }
       speed = std::clamp(speed, 0.0f, 1.0f);
       float duty = min_pwm + speed * (100.0 - min_pwm);
       duty = std::clamp(duty, 0.0f, 100.0f);

       return static_cast<int>(duty);
    }

}
