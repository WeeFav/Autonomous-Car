#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

int main() {
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 500000; // 500 kHz

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

    uint8_t tx[] = { 0x9F }; // Transmit Buffer
    uint8_t rx[sizeof(tx)] = {0}; // Receive Buffer

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = sizeof(tx),
        .speed_hz = speed,
        .bits_per_word = bits
    };

    // Perform SPI Transaction
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        std::cout << "SPI transfer failed" << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "Received: ";
    for (size_t i = 0; i < sizeof(rx); i++) {
        printf("0x%02X", rx[i]);
    }

}