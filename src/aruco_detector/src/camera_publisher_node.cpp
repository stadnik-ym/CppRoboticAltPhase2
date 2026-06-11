#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <memory>

#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
    public:
    CameraPublisher() : Node("camera_publisher_node")
    {
        publisher_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "cannot open camera");
            throw std::runtime_error("camera open failed");
        }

        timer_ = create_wall_timer(33ms, std::bind(&CameraPublisher::publish_frame, this));
    }
    private:
    void publish_frame() {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) return;

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_ -> publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
}
