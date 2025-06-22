#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string.h>

enum MessageType {
    MSG_TYPE_XBOX,
    MSG_TYPE_IMU
};

struct imu_input_t {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

int uart_init(const char* device) {
    // Open device as a file
    // O_RDWR: Open for both reading and writing
    // O_NOCTTY: Do not assign this device as the controlling terminal for the process (prevents signals like CTRL-C from affecting it)
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cout << "Cannot open serial port" << std::endl;
        return -1;
    }

    // fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);

    // Get the current attributes of the Serial port
    struct termios tty;
    tcgetattr(fd, &tty);

    // Configure UART
    cfsetospeed(&tty, B115200); // output baud rate
    cfsetispeed(&tty, B115200); // input baud rate

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 1;                            // read blocks until at least 1 char
    tty.c_cc[VTIME] = 0;                            // timeout in deciseconds

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;                         // Use 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                        // Disables hardware flow control (RTS/CTS)

    // Apply the modified settings to the serial port immediately
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes" << std::endl;
    }

    return fd;
}

void send_data(int fd, const std::string& msg) {
    int bytes_to_write = write(fd, msg.c_str(), msg.size());
    if (bytes_to_write < 0) {
        std::cout << "UART TX error" << std::endl;
    }
}

void receive_data(int fd) {
    // char buf[256];
    // int n = read(fd, buf, sizeof(buf));
    // if (n > 0) {
    //     buf[n] = '\0';
    //     std::cout << "Received: " << buf << std::endl;
    // }    

    int n;
    MessageType type;
    uint16_t size;
    uint8_t payload[24];

    n = read(fd, &type, sizeof(type));
    // std::cout << "first n: " << n << std::endl;
    // std::cout << "type: " << type << std::endl;

    n = read(fd, &size, sizeof(size));
    // std::cout << "second n: " << n << std::endl;
    // std::cout << "size: " << size << std::endl;

    n = read(fd, payload, size);
    // std::cout << "third n: " << n << std::endl;

    if (type == MSG_TYPE_IMU) {
        imu_input_t imu;
        memcpy(&imu, payload, sizeof(imu));
        std::cout << "accel_x:" << imu.accel_x << std::endl;
        std::cout << "accel_y:" << imu.accel_y << std::endl;
        std::cout << "accel_z:" << imu.accel_z << std::endl;

        std::cout << "gyro_x:" << imu.gyro_x << std::endl;
        std::cout << "gyro_y:" << imu.gyro_y << std::endl;
        std::cout << "gyro_z:" << imu.gyro_z << std::endl;
    }    
}

int main() {
    int fd = uart_init("/dev/ttyTHS1");

    if (fd == -1) {
        return -1;
    }

    while (1) {
        send_data(fd, "Hello ESP32\n");
        receive_data(fd);
    }

    close(fd);
    return 0;
}