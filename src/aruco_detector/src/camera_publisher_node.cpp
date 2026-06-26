#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher_node")
    {
        this->declare_parameter<std::string>("camera_source", "libcamera");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);

        std::string source = this->get_parameter("camera_source").as_string();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();

        RCLCPP_INFO(this->get_logger(),
                    "Camera node starting: source=%s, %dx%d@%dfps",
                    source.c_str(), width, height, fps);

        std::string pipeline;

        if (source == "libcamera")
        {
            // Use libcamera - native for Pi 5, minimal buffer allocation
            pipeline = std::string("libcamerasrc ! ") +
                       "video/x-raw,format=YUY2,width=" + std::to_string(width) +
                       ",height=" + std::to_string(height) +
                       ",framerate=" + std::to_string(fps) + "/1 ! " +
                       "videoconvert ! " +
                       "video/x-raw,format=BGR ! " +
                       "appsink drop=true max-buffers=1 sync=false";
        }
        else if (source == "v4l2")
        {
            // Fallback to v4l2src with strict buffer control
            pipeline = std::string("v4l2src device=/dev/video0 num-buffers=2 io-mode=mmap ! ") +
                       "video/x-raw,format=YUY2,width=" + std::to_string(width) +
                       ",height=" + std::to_string(height) +
                       ",framerate=" + std::to_string(fps) + "/1 ! " +
                       "videoconvert ! " +
                       "video/x-raw,format=BGR ! " +
                       "appsink drop=true max-buffers=1 sync=false";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Invalid camera source: %s (use 'libcamera' or 'v4l2')",
                         source.c_str());
            throw std::runtime_error("invalid camera source");
        }

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline:\n%s", pipeline.c_str());

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera pipeline");
            throw std::runtime_error("camera open failed");
        }

        // Verify we can grab a frame
        cv::Mat test_frame;
        if (!cap_.read(test_frame) || test_frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read initial frame from camera");
            throw std::runtime_error("camera frame read failed");
        }

        RCLCPP_INFO(this->get_logger(),
                    "Camera opened successfully, frame size: %dx%d",
                    test_frame.cols, test_frame.rows);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10);

        // 33ms = ~30fps
        timer_ = this->create_wall_timer(
            33ms,
            std::bind(&CameraPublisher::publish_frame, this));

        RCLCPP_INFO(this->get_logger(), "Camera publisher node started");
    }

private:
    void publish_frame()
    {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
            return;
        }

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame).toImageMsg();

        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_frame";

        publisher_->publish(*msg);
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
    return 0;
}
