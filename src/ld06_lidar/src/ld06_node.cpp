#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

// Для работы с серийным портом используем стандартные POSIX-терминалы Linux
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class LD06Node : public rclcpp::Node
{
public:
    LD06Node() : Node("ld06_node")
    {
        // Инициализация параметров
        std::string port = this->declare_parameter("port", "/dev/ttyUSB0");
        frame_id_ = this->declare_parameter("frame_id", "laser_frame");
        double publish_rate = this->declare_parameter("publish_rate_hz", 50.0);
        range_min_ = this->declare_parameter("range_min_m", 0.05);
        range_max_ = this->declare_parameter("range_max_m", 12.0);
        min_conf_ = this->declare_parameter("min_confidence", 0);
        angle_offset_ = this->declare_parameter("angle_offset_deg", 0.0);
        invert_angle_ = this->declare_parameter("invert_angle_direction", false);
        front_min_ = this->declare_parameter("front_min_deg", -35.0);
        front_max_ = this->declare_parameter("front_max_deg", 35.0);
        point_max_age_ = this->declare_parameter("point_max_age_sec", 0.35);
        front_min_points_ = this->declare_parameter("front_min_points", 1);
        front_hold_ = this->declare_parameter("front_hold_sec", 0.8);
        front_window_ = this->declare_parameter("front_filter_window", 5);

        // Настройка массивов данных
        ranges_.assign(360, std::numeric_limits<float>::infinity());
        intensities_.assign(360, 0.0f);
        update_times_.assign(360, rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED));

        // Паблишеры
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        front_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lidar/front_distance", 10);
        closest_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/lidar/closest", 10);

        // Открытие Serial-порта
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", port.c_str());
            throw std::runtime_error("Serial port open failed");
        }

        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B230400);
        cfsetispeed(&tty, B230400);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
        tty.c_iflag = IGNPAR;
        tcsetattr(fd_, TCSANOW, &tty);

        // Основные таймеры (чтение порта и публикация данных)
        read_timer_ = this->create_wall_timer(2ms, std::bind(&LD06Node::read_serial, this));
        pub_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate), std::bind(&LD06Node::publish_data, this));
    }

    ~LD06Node() { if (fd_ >= 0) close(fd_); }

private:
    uint8_t crc8(const uint8_t* data, size_t len) {
        uint8_t crc = 0;
        for (size_t i = 0; i < len; ++i) {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x80) crc = (crc << 1) ^ 0x07;
                else crc <<= 1;
            }
        }
        return crc;
    }

    inline double norm_180(double angle) { return fmod(angle + 180.0 + 360.0, 360.0) - 180.0; }
    inline double norm_360(double angle) { return fmod(fmod(angle, 360.0) + 360.0, 360.0); }

    bool is_in_front(double angle) {
        angle = norm_180(angle);
        return (front_min_ <= front_max_) ? (angle >= front_min_ && angle <= front_max_)
                                          : (angle >= front_min_ || angle <= front_max_);
    }

    void read_serial() {
        uint8_t byte;
        while (read(fd_, &byte, 1) > 0) {
            buffer_.push_back(byte);
            if (buffer_.size() > 100) buffer_.erase(buffer_.begin()); // Защита от переполнения

            if (buffer_.size() >= 47) {
                if (buffer_[0] != 0x54 || buffer_[1] != 0x2C) {
                    buffer_.erase(buffer_.begin());
                    continue;
                }

                if (crc8(buffer_.data(), 46) == buffer_[46]) {
                    parse_packet(buffer_.data());
                    buffer_.erase(buffer_.begin(), buffer_.begin() + 47);
                } else {
                    buffer_.erase(buffer_.begin());
                }
            }
        }
    }

    void parse_packet(const uint8_t* p) {
        double start_angle = (p[5] << 8 | p[4]) / 100.0;
        double end_angle = (p[43] << 8 | p[42]) / 100.0;
        double angle_span = end_angle - start_angle;

        if (angle_span < -180.0) angle_span += 360.0;
        else if (angle_span > 180.0) angle_span -= 360.0;

        double angle_step = angle_span / 11.0;
        auto now = this->get_clock()->now();

        for (int i = 0; i < 12; ++i) {
            size_t offset = 6 + i * 3;
            uint16_t dist_mm = (p[offset + 1] << 8) | p[offset];
            uint8_t confidence = p[offset + 2];

            if (confidence < min_conf_ || dist_mm <= 0) continue;
            float dist_m = dist_mm / 1000.0f;
            if (dist_m < range_min_ || dist_m > range_max_) continue;

            double raw_angle = start_angle + angle_step * i;
            double robot_angle = norm_360((invert_angle_ ? -raw_angle : raw_angle) + angle_offset_);
            int idx = std::clamp(static_cast<int>(std::round(robot_angle)), 0, 359) % 360;

            ranges_[idx] = dist_m;
            intensities_[idx] = confidence;
            update_times_[idx] = now;
        }
    }

    void publish_data() {
        auto now = this->get_clock()->now();
        auto scan = sensor_msgs::msg::LaserScan();
        scan.header.stamp = now;
        scan.header.frame_id = frame_id_;
        scan.angle_min = 0.0;
        scan.angle_max = 2.0 * M_PI;
        scan.angle_increment = M_PI / 180.0;
        scan.range_min = range_min_;
        scan.range_max = range_max_;
        scan.ranges.resize(360);
        scan.intensities.resize(360);

        std::vector<float> front_points;
        float closest_dist = std::numeric_limits<float>::infinity();
        float closest_angle = 999.0f;

        for (int i = 0; i < 360; ++i) {
            double age = (now - update_times_[i]).seconds();
            if (age > point_max_age_ || std::isinf(ranges_[i])) {
                scan.ranges[i] = std::numeric_limits<float>::infinity();
                scan.intensities[i] = 0.0f;
            } else {
                scan.ranges[i] = ranges_[i];
                scan.intensities[i] = intensities_[i];

                if (scan.ranges[i] < closest_dist) {
                    closest_dist = scan.ranges[i];
                    closest_angle = norm_180(i);
                }
                if (is_in_front(i)) {
                    front_points.push_back(scan.ranges[i]);
                }
            }
        }
        scan_pub_->publish(scan);

        // Расчет переднего сектора безопасности
        float front_dist = -1.0f;
        if (front_points.size() >= static_cast<size_t>(front_min_points_)) {
            std::sort(front_points.begin(), front_points.end());
            front_history_.push_back(front_points[0]);
            if (front_history_.size() > static_cast<size_t>(front_window_)) front_history_.erase(front_history_.begin());
            front_dist = *std::min_element(front_history_.begin(), front_history_.end());
            last_valid_front_ = front_dist;
            last_front_time_ = now;
        } else if ((now - last_front_time_).seconds() <= front_hold_ && last_valid_front_ >= 0) {
            front_dist = last_valid_front_;
        }

        auto front_msg = std_msgs::msg::Float32();
        front_msg.data = front_dist;
        front_pub_->publish(front_msg);

        auto closest_msg = std_msgs::msg::Float32MultiArray();
        closest_msg.data = {closest_dist == std::numeric_limits<float>::infinity() ? -1.0f : closest_dist, closest_angle};
        closest_pub_->publish(closest_msg);
    }

    int fd_ = -1;
    std::string frame_id_;
    double range_min_, range_max_, angle_offset_, front_min_, front_max_, point_max_age_, front_hold_;
    int min_conf_, front_min_points_, front_window_;
    bool invert_angle_;
    float last_valid_front_ = -1.0f;
    rclcpp::Time last_front_time_{0, 0, RCL_CLOCK_UNINITIALIZED};

    std::vector<uint8_t> buffer_;
    std::vector<float> ranges_, intensities_, front_history_;
    std::vector<rclcpp::Time> update_times_;

    rclcpp::TimerBase::SharedPtr read_timer_, pub_timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr closest_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LD06Node>());
    rclcpp::shutdown();
    return 0;
}
