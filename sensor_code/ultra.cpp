#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <chrono>

unsigned char buffer_RTT[4] = {0};
uint8_t CS;
#define COM 0x55
int Distance = 0;

int main() {
    const char* portname = "/dev/ttyUSB0"; // or /dev/serial0 if using GPIO UART
    int serial_port = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (serial_port < 0) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }

    termios tty;
    tcgetattr(serial_port, &tty);

    // Configure serial settings
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;        // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;        // no remapping, no delays
    tty.c_cc[VMIN] = 0;     // read doesn't block
    tty.c_cc[VTIME] = 1;    // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes\n";
        return 1;
    }

    while (true) {
        // Send command byte
        write(serial_port, "\x55", 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        unsigned char read_buf[4];
        int n = read(serial_port, read_buf, sizeof(read_buf));

        if (n >= 4 && read_buf[0] == 0xff) {
            buffer_RTT[0] = read_buf[0];
            buffer_RTT[1] = read_buf[1];
            buffer_RTT[2] = read_buf[2];
            buffer_RTT[3] = read_buf[3];

            CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];

            if (buffer_RTT[3] == CS) {
                Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
                std::cout << "Distance: " << Distance << "mm\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

    close(serial_port);
    return 0;
}
