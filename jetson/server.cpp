#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>

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

    // while loop: accept and echo message back to client
    char buf[4096];
    std::string user_input;

    while (true) {
        // send to client
        std::cout << "> ";
        getline(std::cin, user_input);
        int send_res = send(sock, user_input.c_str(), user_input.size() + 1, 0);
        if (send_res == -1) {
            std::cout << "Could not send to client" << std::endl;
            continue;
        }

        // wait for client echo
        memset(buf, 0, 4096);
        int bytes_received = recv(sock, buf, 4096, 0);
        std::cout << "Client: " << std::string(buf, bytes_received) << std::endl;

    }

    // close socket
    close(sock);

    return 0;
}