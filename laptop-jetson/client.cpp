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
        std::cout << "> ";
        getline(std::cin, user_input);
        int send_res = send(client_fd, user_input.c_str(), user_input.size() + 1, 0);
        if (send_res == -1) {
            std::cout << "Could not send to server" << std::endl;
            continue;
        }

        memset(buf, 0, 4096);
        int bytes_received = recv(client_fd, buf, 4096, 0);

        std::cout << "Server> " << std::string(buf, bytes_received) << std::endl;
    }

    
    close(client_fd);
    return 0;
}