#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>

class CameraPublisher : public rclcpp::Node 
{
public:
    CameraPublisher() : Node("camera_publisher") {

        // GStreamer pipeline
        std::string pipeline =
            "udpsrc port=5000 caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! appsink";

        publisher = this->create_publisher<sensor_msgs::msg::Image>("camera1/image_raw", 10);

        cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video capture");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "OK");

        // Timer to fetch frames at ~30 Hz
        timer = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&CameraPublisher::timerCallback, this));
        
    }
    
private:
    void timerCallback() {

        cv::Mat frame;
        if (!cap.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame");
            return;
        }

        // Convert OpenCV frame to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera1";
        
        // Publish
        publisher->publish(*msg);     
           
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    cv::VideoCapture cap;
    rclcpp::TimerBase::SharedPtr timer;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}