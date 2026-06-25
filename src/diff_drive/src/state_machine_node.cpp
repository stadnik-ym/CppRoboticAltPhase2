#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>

class StateMachineNode : public rclcpp::Node {
public:
    StateMachineNode() : Node("state_machine_node") {
        RCLCPP_INFO(this->get_logger(), "State Machine Node started. Hunting for ArUco...");

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/aruco_target", 10,
            std::bind(&StateMachineNode::target_cb, this, std::placeholders::_1)
        );

        // Таймер контрольного цикла (работает на частоте 20 Гц)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&StateMachineNode::control_loop, this)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Состояние цели
    rclcpp::Time last_target_time_;
    double target_x_ = 0.0;
    double target_area_ = 0.0;
    bool target_active_ = false;

    // Конфигурация П-регулятора
    const double KP_ANGULAR = 1.2;     // Коэффициент резкости поворота
    const double MAX_LINEAR = 0.15;    // Максимальная скорость вперед (м/с)
    const double STOP_AREA = 50000.0;  // Площадь маркера, при которой нужно остановиться (подобрать опытным путем)
    const double TARGET_TIMEOUT = 0.5; // Секунды до признания маркера "потерянным"

    void target_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
        target_x_ = msg->x;
        target_area_ = msg->z;
        last_target_time_ = this->get_clock()->now();
        target_active_ = true;
    }

    void control_loop() {
        auto cmd_msg = geometry_msgs::msg::Twist();
        auto now = this->get_clock()->now();

        // Проверяем, не устарели ли данные (таймаут потери маркера)
        if (target_active_ && (now - last_target_time_).seconds() < TARGET_TIMEOUT) {

            // 1. Угловая скорость (довернуть на маркер)
            // Если маркер справа (target_x_ > 0), угловая скорость отрицательная (вправо)
            cmd_msg.angular.z = -KP_ANGULAR * target_x_;

            // Ограничиваем угловую скорость, чтобы робот не крутился слишком дико
            cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -1.5, 1.5);

            // 2. Линейная скорость (ехать вперед, пока маркер не станет большим)
            if (target_area_ < STOP_AREA) {
                // Плавное замедление при приближении
                double speed_factor = 1.0 - (target_area_ / STOP_AREA);
                cmd_msg.linear.x = MAX_LINEAR * std::max(0.2, speed_factor);
            } else {
                cmd_msg.linear.x = 0.0; // Приехали
            }

        } else {
            // Маркер потерян или еще не найден -> останавливаем робота
            target_active_ = false;
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}
