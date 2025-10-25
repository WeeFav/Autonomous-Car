#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <netinet/tcp.h>

bool send_all(int sock, const uchar* data, size_t size) {
    size_t total_sent = 0;
    while (total_sent < size) {
        ssize_t sent = send(sock, data + total_sent, size - total_sent, 0);
        if (sent <= 0) return false; // Connection lost or error
        total_sent += sent;
    }
    return true;
}

int main() {
    // create listening socket
    int listen_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_socket == -1) {
        std::cerr << "Socket creation failed.\n";
        return 1;
    }

    // bind listening socket to server ip and port
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(8000);
    if (bind(listen_socket, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        std::cerr << "Bind failed.\n";
        close(listen_socket);
        return 1;
    }

    // tell the socket is for listening
    listen(listen_socket, SOMAXCONN);

    // define client ip and port structure
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    // wait for connection
    int sock = accept(listen_socket, (sockaddr*)&client_addr, &client_len);
    if (sock == -1) {
        std::cerr << "Accept failed.\n";
        close(listen_socket);
        return 1;
    }

    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    int snd_buf = 1 * 1024 * 1024; // 1 MB
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &snd_buf, sizeof(snd_buf));

    char host[NI_MAXHOST]; // client's remote name
    char service[NI_MAXSERV]; // client's port

    memset(host, 0, NI_MAXHOST);
    memset(service, 0, NI_MAXSERV);

    if (getnameinfo((sockaddr*)&client_addr, sizeof(client_addr), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0) {
        std::cout << host << " connected on port " << service << std::endl;
    }
    else {
        inet_ntop(AF_INET, &client_addr.sin_addr, host, NI_MAXHOST);
        std::cout << host << " connected on port " << ntohs(client_addr.sin_port) << std::endl;
    }

    // close listening socket (don't recieve other clients)
    close(listen_socket);

    // Start sending

    // GStreamer pipeline string for the Raspberry Pi camera on Jetson Nano
    std::string pipeline = "nvarguscamerasrc sensor-id=0 ! "
                           "video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=30/1 ! "
                           "nvvidconv flip-method=2 ! "
                           "video/x-raw, format=BGRx ! "
                           "videoconvert ! "
                           "video/x-raw, format=BGR ! appsink";

    // OpenCV video capture object
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    int counter = 0;
    while (true) {
        // read image
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }

        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

        // Encode image as JPEG
        std::vector<uchar> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70}; // adjust quality
        cv::imencode(".jpg", resized, buf, params);

        // // send image size
        // uint32_t img_size = buf.size();
        // uint32_t img_size_net = htonl(img_size); // Convert to network byte order
        // send(sock, &img_size_net, sizeof(img_size_net), 0);

        // // send image
        // send(sock, buf.data(), img_size, 0);

        uint32_t img_size = buf.size();
        uint32_t img_size_net = htonl(img_size);
        if (!send_all(sock, reinterpret_cast<uchar*>(&img_size_net), sizeof(img_size_net))) break;
        if (!send_all(sock, buf.data(), img_size)) break;

        std::cout << "Send frame " << std::to_string(counter) << std::endl;
        counter++;
    }

    // close socket
    close(sock);

    return 0;
}