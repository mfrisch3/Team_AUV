#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <string>
#include <chrono>
#include <thread>

// Constants
#define COM 0x55
const std::string SERIAL_PORT = "/dev/ttyS0";  // or "/dev/ttyAMA0" depending on Pi model
const int BAUD_RATE = B115200;

// Global variables
unsigned char buffer_RTT[4] = {0};
uint8_t CS;
int Distance = 0;
int serial_fd;

void setupSerial() {
    // Open serial port
    serial_fd = open(SERIAL_PORT.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        std::cerr << "Error - Unable to open UART" << std::endl;
        exit(1);
    }

    // Configure serial port
    struct termios options;
    tcgetattr(serial_fd, &options);
    options.c_cflag = BAUD_RATE | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serial_fd, TCIFLUSH);
    tcsetattr(serial_fd, TCSANOW, &options);
}

void delay(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

int main() {
    setupSerial();
    
    while(true) {
        // Send command
        unsigned char command = COM;
        write(serial_fd, &command, 1);
        delay(100);

        // Check for available data
        int bytes_available;
        ioctl(serial_fd, FIONREAD, &bytes_available);
        
        if(bytes_available > 0) {
            delay(4);  // Small delay to ensure all data is received
            
            unsigned char start_byte;
            read(serial_fd, &start_byte, 1);
            
            if(start_byte == 0xff) {
                buffer_RTT[0] = 0xff;
                
                // Read remaining 3 bytes
                for(int i = 1; i < 4; i++) {
                    read(serial_fd, &buffer_RTT[i], 1);
                }
                
                // Calculate checksum
                CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
                
                if(buffer_RTT[3] == CS) {
                    Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
                    std::cout << "Distance: " << Distance << "mm" << std::endl;
                }
            }
        }
    }
    
    close(serial_fd);
    return 0;
}
