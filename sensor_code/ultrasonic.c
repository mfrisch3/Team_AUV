#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define SERIAL_PORT "/dev/serial0"  // Default UART port on Raspberry Pi
#define COM 0x55                    // Command byte sent to the sensor

unsigned char buffer_RTT[4] = {0};
uint8_t CS;
int Distance = 0;

int serial_fd;  // File descriptor for the serial port

// Function to initialize the UART (serial) communication
int UART_init(const char *port, int baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY); // Open the serial port
    if (fd == -1) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options; // Structure to store UART settings
    tcgetattr(fd, &options); // Get current settings

    // Set baud rate
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // Configure serial settings: 8-bit data, local connection, enable receiver
    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR; // Ignore parity errors
    options.c_oflag = 0;      // Raw output mode
    options.c_lflag = 0;      // Non-canonical mode (no line buffering)

    tcflush(fd, TCIFLUSH);  // Clear the serial input buffer
    tcsetattr(fd, TCSANOW, &options); // Apply the settings

    return fd;
}

// Function to send a single byte over UART
void UART_write(int fd, uint8_t data) {
    write(fd, &data, 1);
}

// Function to receive a single byte over UART
int UART_read(int fd) {
    uint8_t data;
    if (read(fd, &data, 1) > 0) {
        return data;  // Return received byte
    }
    return -1;  // Return -1 if no data is available
}

int main() {
    // Initialize serial communication
    serial_fd = UART_init(SERIAL_PORT, B115200);
    if (serial_fd == -1) {
        return 1;  // Exit if serial port fails to open
    }

    // Main loop
    while (1) {
        UART_write(serial_fd, COM);  // Send command byte to request data
        usleep(100000);  // Wait 100ms for the sensor to respond

        if (UART_read(serial_fd) == 0xff) {  // Check if the first byte is the start marker (0xFF)
            buffer_RTT[0] = 0xff;  // Store the start byte
            for (int i = 1; i < 4; i++) {
                buffer_RTT[i] = UART_read(serial_fd);  // Read the next 3 bytes
            }

            // Compute checksum
            CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];

            // Validate checksum
            if (buffer_RTT[3] == CS) {
                Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];  // Combine high and low bytes
                printf("Distance: %d mm\n", Distance);  // Print the distance
            } else {
                printf("Checksum error\n");
            }
        }
    }

    close(serial_fd);  // Close the serial port
    return 0;
}
