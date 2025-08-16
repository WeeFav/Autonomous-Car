#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <opencv2/opencv.hpp>

int main() {
    // create socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Socket creation failed.\n";
        return 1;
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
        std::cerr << "Connect failed.\n";
        close(sock);    
        return 1;
    }

    cv::Size frame_size(640, 480);
    cv::VideoWriter writer("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 8, frame_size);
    if (!writer.isOpened()) {
        std::cerr << "Error: Cannot open video file for write" << std::endl;
        return 1;
    }

    // Start receive

    while (true) {
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
            std::cerr << "Failed to decode image\n";
        }
        else {
            writer.write(img);

            cv::imshow("Received Image", img);
        }

         // Stop if ESC pressed
        if (cv::waitKey(1) == 27) break;
    }

    writer.release();
    cv::destroyAllWindows();
    close(sock);
    return 0;
}