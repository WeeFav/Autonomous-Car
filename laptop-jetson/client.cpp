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
    int client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd == -1) {
        std::cerr << "Socket creation failed.\n";
        return 1;
    }

    // connect to server on the socket
    int port = 8000;
    std::string ip_address = "127.0.0.1";
    
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip_address.c_str(), &server_addr.sin_addr);

    int connect_res = connect(client_fd, (sockaddr*)&server_addr, sizeof(server_addr));
    if (connect_res == -1) {
        std::cerr << "Connect failed.\n";
        close(client_fd);    
        return 1;
    }

    // while loop:
    char buf[4096];
    std::string user_input;

    while (true) {
        // wait for server to send data
        int bytes_received = recv(client_socket, buf, 4096, 0);
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
        send(client_socket, buf, bytes_received + 1, 0);
    }

    
    close(client_fd);
    return 0;
}