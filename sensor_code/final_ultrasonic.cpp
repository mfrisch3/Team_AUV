#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <sys/ioctl.h>

#include <chrono>
#include <iostream>
#include <thread>

int serial_port;
unsigned char cmd = 0x55;


void setup(){
	
	serial_port = open("/dev/ttyS0", O_RDWR | O_NONBLOCK);

	// check for error
	if (serial_port < 0){
		printf("Error %i from ope: %s\n", errno, strerror(errno));
		fflush(stdout);
	}

	struct termios tty;

	if(tcgetattr(serial_port, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno,strerror(errno));
		fflush(stdout);
	}
	
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;
	
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;
	
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
	
	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;

	tty.c_cc[VTIME] = 10;
	tty.c_cc[VMIN] = 1;

	// Set Baud Rate
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("error %i from tcsetattr: %s\n", errno, strerror(errno));
		fflush(stdout);
	}
}

int main() {
	printf("Begin UART setup\n");
	setup();
	fflush(stdout);
	printf("Setup ended. UART successfully initialized\n");
	fflush(stdout);

	uint8_t CS;

	while(true) {

		// send command to initialize ultrasonic
		write(serial_port, &cmd, 1);
		usleep(100000);

		int bytes_available;
		ioctl(serial_port, FIONREAD, &bytes_available);
		usleep(4000);

		printf("Available bytes: %i", bytes_available);
		if (bytes_available > 0) {
			// read from ultrasonic
			char read_buf [4];
			int distance;
		
			int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
				
			if (num_bytes < 0) {
				printf("Error reading: %s\n", strerror(errno));
				fflush(stdout);
			}
	 	
			// check if header data is correct
			if (read_buf[0] != 0xff){
				printf("header not correct: %x\n", read_buf[0]);
				fflush(stdout);
			}

			CS = read_buf[0] + read_buf[1] + read_buf[2];
		
			if (read_buf[3] != CS){
				printf("Incorrect CS value: %x, wanted: %x\n", CS, read_buf[3]);
				fflush(stdout);
			}
		
			distance = (read_buf[1] << 8) + read_buf[2];
			printf("distance measured: %i\n", distance);
			fflush(stdout);
			// printf("Read %i bytes. Received message %s\n", num_bytes, read_buf);
			fflush(stdout);
		}
	}

	return 0;
}

