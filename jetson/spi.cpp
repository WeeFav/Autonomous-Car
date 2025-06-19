#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <chrono>
#include <thread>

int main() {
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 100000; // 500 kHz

    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {
        std::cout << "Failed to open SPI communication on /dev/spidev0.0" << std::endl;
        return 1;
    }

    // set SPI mode
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cout << "Failed to set SPI mode" << std::endl;
        close(fd);
        return 1;
    }

    // set bits per word
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        std::cout << "Failed to set bits per word" << std::endl;
        close(fd);
        return 1;
    }

    // set max speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ , &speed) < 0) {
        std::cout << "Failed to set max speed" << std::endl;
        close(fd);
        return 1;
    }

    uint8_t tx[] = { 0x9F, 0x00, 0x00, 0x00, 0x9F, 0x00, 0x00, 0x00 }; // Transmit Buffer
    uint8_t rx[sizeof(tx)] = {0}; // Receive Buffer

    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = sizeof(tx);
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    tr.delay_usecs = 1000;

    while (1) {
        // Perform SPI Transaction
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            std::cout << "SPI transfer failed" << std::endl;
            close(fd);
            return 1;
        }

        std::cout << "Received: ";
        for (size_t i = 0; i < sizeof(rx); i++) {
            printf("0x%02X\n", rx[i]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}