#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <thread>
#include <atomic>
#include <csignal>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>  // for Full, Screen
#include "ftxui/component/component.hpp"
#include "ftxui/component/loop.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/node.hpp"

using namespace ftxui;

std::atomic<bool> running(true);

enum MessageType {
    MSG_TYPE_XBOX,
    MSG_TYPE_ENCODER,
    MSG_TYPE_IMU,
    MSG_TYPE_INA
};

#pragma pack(push, 1)  // Ensure packed structure
struct xbox_input_t {
    // Joysticks (16-bit values, 0–65535)
    uint16_t left_stick_x;
    uint16_t left_stick_y;
    uint16_t right_stick_x;
    uint16_t right_stick_y;

    // Triggers (10-bit values, 0–1023)
    uint16_t left_trigger : 10;
    uint16_t left_trigger_padding : 6;
    uint16_t right_trigger : 10;
    uint16_t right_trigger_padding : 6;

    // D-Pad (4-bit value)
    uint8_t dpad : 4;
    uint8_t dpad_padding : 4;

    // Face Buttons & Bumpers (1-bit flags)
    uint8_t button_a : 1;
    uint8_t button_b : 1;
    uint8_t rfu_1 : 1;
    uint8_t button_x : 1;
    uint8_t button_y : 1;
    uint8_t rfu_2 : 1;
    uint8_t left_bumper : 1;
    uint8_t right_bumper : 1;

    // Special Buttons (1-bit flags)
    uint8_t rfu_3 : 1;
    uint8_t rfu_4 : 1;
    uint8_t view_button : 1;
    uint8_t menu_button : 1;
    uint8_t left_stick_press : 1;
    uint8_t right_stick_press : 1;
    uint8_t rfu_5 : 1;
    uint8_t rfu_6 : 1;

    // Share Button (1-bit flag)
    uint8_t share_button : 1;
    uint8_t share_button_padding : 7;
};
#pragma pack(pop)

#pragma pack(push, 1)  // Ensure packed structure
struct encoder_input_t {
    int left_rpm, right_rpm;
};
#pragma pack(pop)

#pragma pack(push, 1)  // Ensure packed structure
struct imu_input_t {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};
#pragma pack(pop)

#pragma pack(push, 1)  // Ensure packed structure
struct ina_input_t {
    float left_voltage, left_current;
    float right_voltage, right_current;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct uart_tx_message_t {
    MessageType type;
    uint8_t payload[24]; // maximum struct size is imu_input_t (24 bytes)
};
#pragma pack(pop)

static float mapToPercent(uint16_t value) {
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;
    return ((value * 100) / 1023) / 100.0;    
}

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

void receive_data(int fd) {
    std::cout << "Receive thread spawned" << std::endl;

    int n;
    uart_tx_message_t msg;

    int xbox_direction = 0;
    float xbox_speed_percent = 0;
    int left_rpm = 0;
    int right_rpm = 0;
    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;
    float gyro_x = 0;
    float gyro_y = 0;
    float gyro_z = 0;
    float left_voltage = 0; 
    float left_current = 0; 
    float right_voltage = 0;
    float right_current = 0;

    auto screen = Screen::Create(Dimension::Full(), Dimension::Full());    
    std::string reset_position;

    while (running) {
        n = read(fd, &msg, sizeof(msg));

        if (msg.type == MSG_TYPE_XBOX) {
            xbox_input_t report;
            memcpy(&report, msg.payload, sizeof(report));

            // update variables
            xbox_speed_percent = mapToPercent(report.right_trigger);
            xbox_direction = report.dpad;
        }
        else if (msg.type == MSG_TYPE_ENCODER) {
            encoder_input_t motor_speed;
            memcpy(&motor_speed, msg.payload, sizeof(motor_speed));

            // update variables
            left_rpm = motor_speed.left_rpm;
            right_rpm = motor_speed.right_rpm;
        }
        else if (msg.type == MSG_TYPE_IMU) {
            imu_input_t imu;
            memcpy(&imu, msg.payload, sizeof(imu));

            // update variables
            accel_x =  imu.accel_x;
            accel_y =  imu.accel_y;
            accel_z =  imu.accel_z;
            gyro_x = imu.gyro_x;
            gyro_y = imu.gyro_y;
            gyro_z = imu.gyro_z;
        }
        else if (msg.type == MSG_TYPE_INA) {
            ina_input_t ina;
            memcpy(&ina, msg.payload, sizeof(ina));

            // update variables
            if (xbox_direction == 1) {
                left_voltage = ina.left_voltage;
                right_voltage = ina.left_voltage;
            }
            if (xbox_direction == 5) {
                left_voltage = ina.right_voltage;
                right_voltage = ina.right_voltage;
            }
            else {
                left_voltage = ina.left_voltage;
                right_voltage = ina.right_voltage;                
            }
            left_current = ina.left_current;
            right_current = ina.right_current;
        }

        // ============ FTXUI ============

        // IMU data tiles
        auto imu_tile = vbox({
            text("IMU (6 DOF)") | bold | center,
            separator(),
            text("Accel X: " + std::to_string(accel_x) + " m/s²"),
            text("Accel Y: " + std::to_string(accel_y) + " m/s²"),
            text("Accel Z: " + std::to_string(accel_z) + " m/s²"),
            separator(),
            text("Gyro X: " + std::to_string(gyro_x) + " °/s"),
            text("Gyro Y: " + std::to_string(gyro_y) + " °/s"),
            text("Gyro Z: " + std::to_string(gyro_z) + " °/s"),
        }) | border | size(WIDTH, EQUAL, 28);

        // Motor tiles
        auto motor_tile = [&](std::string name, float voltage, float current, int rpm) {
            // std::ostringstream oss_v, oss_c, oss_r;
            // oss_v << std::fixed << std::setprecision(2) << voltage;
            // oss_c << std::fixed << std::setprecision(2) << current;
            // oss_r << std::fixed << std::setprecision(1) << rpm;

            return vbox({
                text(name) | bold | center,
                separator(),
                text("Voltage: " + std::to_string(voltage) + " V"),
                text("Current: " + std::to_string(current) + " A"),
                text("Speed: " + std::to_string(rpm) + " RPM"),
            }) | border | size(WIDTH, EQUAL, 25);
        };

        auto motors = hbox({
            motor_tile("Left Motor", left_voltage, left_current, left_rpm),
            separator(),
            motor_tile("Right Motor", right_voltage, right_current, right_rpm),
        });

        // Xbox direction buttons
        auto render_button = [](std::string label, bool pressed) {
            return text(label) |
                center |
                border |
                bgcolor(pressed ? Color::Green : Color::GrayDark) |
                size(WIDTH, EQUAL, 10) |
                size(HEIGHT, EQUAL, 3);
        };

        auto up_box = render_button("UP", xbox_direction == 1);
        auto right_box = render_button("RIGHT", xbox_direction == 3);
        auto down_box = render_button("DOWN", xbox_direction == 5);
        auto left_box = render_button("LEFT", xbox_direction == 7);

        // Arrange in D-pad layout:
        auto dpad = vbox({
            hbox({ filler(), up_box, filler() }),
            hbox({ left_box, filler(), right_box }),
            hbox({ filler(), down_box, filler() }),
        }) | border | center;

        // Xbox trigger
        auto trigger_gauge = vbox({
            text("Speed") | center,
            gaugeUp(xbox_speed_percent) | color(Color::BlueLight) | size(HEIGHT, EQUAL, 10) | border,
            text(std::to_string(int(xbox_speed_percent * 100)) + "%") | center,
        });

        // Combine everything
        auto layout = vbox({
            text("🚗  Autonomous Car Dashboard") | bold | center,
            separator(),
            hbox({
                imu_tile,
                separator(),
                motors,
                dpad,
                trigger_gauge
            }),
        }) | border;

        Render(screen, layout);
        std::cout << reset_position;
        screen.Print();  
        reset_position = screen.ResetPosition();

        // std::cout << gyro_z << std::endl;
        // usleep(1000); // sleep for 10 ms 
    }

    std::cout << "Exiting receive thread..." << std::endl;
}

int main() {
    int fd = uart_init("/dev/ttyTHS1"); // sudo ./uart

    if (fd == -1) {
        return -1;
    }

    std::thread reader(receive_data, fd);

    // Watch for SIGINT (Ctrl+C)
    std::signal(SIGINT, [](int) {
        running = false;
    });

    reader.join();

    close(fd);
    std::cout << "end" << std::endl;
    return 0;
}