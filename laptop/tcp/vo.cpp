#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>

bool recv_all(int sock, void *data, size_t size) {
    char *ptr = (char*)data;
    while (size > 0) {
        ssize_t recvd = recv(sock, ptr, size, 0);
        if (recvd <= 0) return false;
        ptr += recvd;
        size -= recvd;
    }
    return true;
}

int main() {
    // ---------- CONNECT TO SERVER ----------
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Socket creation failed.\n";
        return 1;
    } 

    std::string ip_address = "192.168.0.106"; // server's ip
    int port = 8000;
    sockaddr_in server_addr; // structure to hold the ip address and port for the socket connection
    server_addr.sin_family = AF_INET; // ipv4
    server_addr.sin_port = htons(port); // port
    inet_pton(AF_INET, ip_address.c_str(), &server_addr.sin_addr); // ip

    int connect_res = connect(sock, (sockaddr*)&server_addr, sizeof(server_addr));
    if (connect_res == -1) {
        std::cerr << "Connect failed.\n";
        close(sock);    
        return 1;
    }

    while (true) {
        // receive image size
        uint32_t img_size_net;
        if(!recv_all(sock, &img_size_net, sizeof(img_size_net))) break;
        uint32_t img_size = ntohl(img_size_net); // convert from network byte to local byte order

        // receive image
        std::vector<uchar> buf(img_size);
        if(!recv_all(sock, buf.data(), img_size)) break;

        // receive pose
        cv::Point2d pose;
        if(!recv_all(sock, &pose, sizeof(pose))) break;
        
        // decode image
        cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);

        std::cout << "Pose: X=" << pose.x << " Z=" << pose.y << std::endl;
        
        cv::imshow("Received Image", img);
        cv::waitKey(1);
    }

    close(sock);
    return 0;
}
