#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>

class ArenaTrackerNode : public rclcpp::Node {
public:
    ArenaTrackerNode() : Node("arena_tracker_node"), v_max_(160.0), s_max_(85.0) {
        // Создаем ROS 2 паблишеры вместо UDP
        robot_pub_ = this->create_publisher<geometry_msgs::msg::Point>("robot_position", 10);
        obstacles_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>("obstacles_positions", 10);

        // Подключаемся к камере (0 - дефолтный индекс)
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Не удалось открыть камеру!");
            rclcpp::shutdown();
            return;
        }

        frame_width_ = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        frame_height_ = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        px_to_cm_x_ = ARENA_LENGTH_CM / frame_width_;
        px_to_cm_y_ = ARENA_WIDTH_CM / frame_height_;
        px2_to_cm2_ = px_to_cm_x_ * px_to_cm_y_;

        cv::namedWindow("Brain Tracker", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Debug: Mask", cv::WINDOW_AUTOSIZE);

        // Таймер, который крутит цикл обработки (~30 FPS)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ArenaTrackerNode::process_frame, this)
        );

        RCLCPP_INFO(this->get_logger(), "✅ C++ Трекер запущен! Публикация в /robot_position и /obstacles_positions");
    }

private:
    void process_frame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        cv::Mat blurred, hsv, black_mask;
        cv::GaussianBlur(frame, blurred, cv::Size(7, 7), 0);
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower_black(0, 0, 0);
        cv::Scalar upper_black(180, s_max_, v_max_);
        cv::inRange(hsv, lower_black, upper_black, black_mask);

        // Морфология для склеивания силуэта
        cv::Mat kernel = cv::Mat();
        cv::Mat temp_mask;
        cv::dilate(black_mask, temp_mask, kernel, cv::Point(-1, -1), 6);
        cv::erode(temp_mask, black_mask, kernel, cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool robot_found = false;
        double robot_cx_cm = 0.0;
        double robot_cy_cm = 0.0;
        cv::Rect best_robot_rect;
        double max_robot_area = 0.0;
        std::vector<std::pair<cv::Rect, cv::Point2d>> raw_obstacles;

        for (const auto& contour : contours) {
            double area_px = cv::contourArea(contour);
            double area_cm2 = area_px * px2_to_cm2_;
            cv::Rect rect = cv::boundingRect(contour);

            int margin = 45;
            if (rect.x < margin || rect.y < margin ||
                rect.x + rect.width > frame_width_ - margin ||
                rect.y + rect.height > frame_height_ - margin) {
                continue;
            }

            if (area_cm2 > 10.0) {
                cv::rectangle(frame, rect, cv::Scalar(0, 255, 255), 1);
                cv::putText(frame, std::to_string((int)area_cm2) + "cm2",
                            cv::Point(rect.x, rect.y - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
            }

            if (area_cm2 >= MIN_ROBOT_AREA && area_cm2 <= MAX_ROBOT_AREA) {
                if (area_cm2 > max_robot_area) {
                    max_robot_area = area_cm2;
                    best_robot_rect = rect;
                }
            } else if (area_cm2 >= MIN_OBSTACLE_AREA && area_cm2 <= MAX_OBSTACLE_AREA) {
                double obs_cx_cm = (rect.x + rect.width / 2.0) * px_to_cm_x_;
                double obs_cy_cm = (rect.y + rect.height / 2.0) * px_to_cm_y_;
                raw_obstacles.push_back({rect, cv::Point2d(obs_cx_cm, obs_cy_cm)});
            }
        }

        // --- ФОРМИРОВАНИЕ ROS 2 СООБЩЕНИЙ ---
        geometry_msgs::msg::Point robot_msg;
        geometry_msgs::msg::Polygon obstacles_msg;

        if (max_robot_area > 0) {
            robot_cx_cm = (best_robot_rect.x + best_robot_rect.width / 2.0) * px_to_cm_x_;
            robot_cy_cm = (best_robot_rect.y + best_robot_rect.height / 2.0) * px_to_cm_y_;
            robot_found = true;

            // X и Y — координаты. Z используем как флаг: 1.0 (найден), 0.0 (потерян)
            robot_msg.x = robot_cx_cm;
            robot_msg.y = robot_cy_cm;
            robot_msg.z = 1.0;

            cv::rectangle(frame, best_robot_rect, cv::Scalar(0, 255, 0), 3);
            cv::putText(frame, "ROBOT", cv::Point(best_robot_rect.x, best_robot_rect.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            int cx_px = robot_cx_cm / px_to_cm_x_;
            int cy_px = robot_cy_cm / px_to_cm_y_;
            int radius_px = DETECTION_RADIUS_CM / px_to_cm_x_;
            cv::circle(frame, cv::Point(cx_px, cy_px), radius_px, cv::Scalar(0, 255, 0), 1);
        } else {
            robot_msg.z = 0.0; // Флаг потери робота
        }

        // Анализ препятствий внутри зоны радара
        for (const auto& obs : raw_obstacles) {
            if (robot_found) {
                double dx = obs.second.x - robot_cx_cm;
                double dy = obs.second.y - robot_cy_cm;
                double distance = std::sqrt(dx*dx + dy*dy);

                if (distance <= DETECTION_RADIUS_CM) {
                    geometry_msgs::msg::Point32 p;
                    p.x = obs.second.x;
                    p.y = obs.second.y;
                    p.z = distance; // Можно передавать саму дистанцию в Z
                    obstacles_msg.points.push_back(p);

                    cv::rectangle(frame, obs.first, cv::Scalar(0, 0, 255), 2);
                }
            }
        }

        // ПУБЛИКАЦИЯ В ТОПИКИ
        robot_pub_->publish(robot_msg);
        obstacles_pub_->publish(obstacles_msg);

        // Отрисовка интерфейса и калибровки
        cv::putText(frame, "V Max (Brightness): " + std::to_string((int)v_max_), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(frame, "S Max (Color): " + std::to_string((int)s_max_), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        cv::Rect safe_zone(45, 45, frame_width_ - 90, frame_height_ - 90);
        cv::rectangle(frame, safe_zone, cv::Scalar(0, 100, 255), 1);

        cv::imshow("Brain Tracker", frame);
        cv::imshow("Debug: Mask", black_mask);

        // Перехват кнопок
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 113) { rclcpp::shutdown(); }
        if (key == 'w' || key == 119) { v_max_ = std::min(v_max_ + 10.0, 255.0); }
        if (key == 's' || key == 115) { v_max_ = std::max(v_max_ - 10.0, 0.0); }
        if (key == 'e' || key == 101) { s_max_ = std::min(s_max_ + 5.0, 255.0); }
        if (key == 'd' || key == 100) { s_max_ = std::max(s_max_ - 5.0, 0.0); }
    }

    const double ARENA_LENGTH_CM = 142.0;
    const double ARENA_WIDTH_CM = 77.0;
    const double MIN_OBSTACLE_AREA = 15.0;
    const double MAX_OBSTACLE_AREA = 140.0;
    const double MIN_ROBOT_AREA = 60.0;
    const double MAX_ROBOT_AREA = 700.0;
    const double DETECTION_RADIUS_CM = 20.0;

    double v_max_, s_max_;
    double frame_width_, frame_height_, px_to_cm_x_, px_to_cm_y_, px2_to_cm2_;

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr obstacles_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArenaTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
