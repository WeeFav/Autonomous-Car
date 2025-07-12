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
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera1/image_raw", 10);

        // create socket
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock == -1) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed.");
            throw std::runtime_error("Socket creation failed");
        }

        // define server ip and port
        std::string ip_address = "192.168.0.106"; // server's ip
        int port = 8000; // server's port
        sockaddr_in server_addr; // structure to hold the ip address and port for the socket connection
        server_addr.sin_family = AF_INET; // ipv4
        server_addr.sin_port = htons(port); // port
        inet_pton(AF_INET, ip_address.c_str(), &server_addr.sin_addr); // ip

        // connect socket to server
        int connect_res = connect(sock, (sockaddr*)&server_addr, sizeof(server_addr));
        if (connect_res == -1) {
            RCLCPP_ERROR(this->get_logger(), "Connect failed.");
            close(sock);    
            throw std::runtime_error("Connect failed");
        }

        // Start dedicated thread for receiving data
        recv_thread = std::thread(&CameraPublisher::receive_loop, this);
        
    }

    ~CameraPublisher() {
        close(sock);
        if (recv_thread.joinable()) {
            recv_thread.join();
        }
    }
    
private:
    void receive_loop() {
        while (rclcpp::ok()) {
            // receive image size
            uint32_t img_size_net;
            recv(sock, &img_size_net, sizeof(img_size_net), 0);
            uint32_t img_size = ntohl(img_size_net); // convert from network byte to local byte order

            // receive image
            std::vector<uchar> buf(img_size);
            size_t bytes_received = 0;
            while (bytes_received < img_size) {
                ssize_t bytes = recv(sock, buf.data() + bytes_received, img_size - bytes_received, 0);
                if (bytes <= 0) {
                    break;
                }
                bytes_received += bytes;
            }

            // decode image
            cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
            if (img.empty()) {
                RCLCPP_WARN(this->get_logger(), "Failed to decode image");
                continue;
            }
            // else {
            //     cv::imshow("Received Image", img);
            //     cv::waitKey(1);
            // }

            // publish
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
            msg->header.stamp = this->get_clock()->now();
            msg->header.frame_id = "camera";
            publisher_->publish(*msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::thread recv_thread;
    int sock;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}