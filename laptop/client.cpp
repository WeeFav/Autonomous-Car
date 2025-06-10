#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>

int main() {
    // create socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Socket creation failed.\n";
        return 1;
    }

    // define server ip and port
    std::string ip_address = "127.0.0.1"; // server's ip
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

    // while loop:
    char buf[4096];
    std::string user_input;

    while (true) {
        // wait for server to send data
        int bytes_received = recv(sock, buf, 4096, 0);
        if (bytes_received < 0) {
            std::cerr << "Error in recv()" << std::endl;
            break;
        }
        if (bytes_received == 0) {
            std::cout << "Server disconnected" << std::endl;
            break;
        }

        // display
        std::cout << "Server: " << std::string(buf, 0, bytes_received) << std::endl;

        // echo message back
        send(sock, buf, bytes_received + 1, 0);
    }

    
    close(sock);
    return 0;
}