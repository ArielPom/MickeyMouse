#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int main() {
    int fd; // File descriptor for the serial port
    struct termios serial;

    // Open the serial port
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open");
        return 1;
    }

    // Configure the serial port
    tcgetattr(fd, &serial);
    cfsetispeed(&serial, B115200); // Set baud rate to 9600
    cfsetospeed(&serial, B115200);
    serial.c_cflag &= ~PARENB;   // Disable parity bit
    serial.c_cflag &= ~CSTOPB;   // Use one stop bit
    serial.c_cflag &= ~CSIZE;
    serial.c_cflag |= CS8;       // 8 bits per byte
    tcsetattr(fd, TCSANOW, &serial);

    // Send numbers from 0 to 30
    for (int i = 0; i <= 30; ++i) {
        char command[10]; // Assuming numbers won't be more than 10 characters
        sprintf(command, "%d\r\n", i); // Convert integer to string
        write(fd, command, strlen(command));
        usleep(100000); // Sleep for 100 milliseconds (adjust as needed)
    }

    // Close the serial port
    close(fd);

    return 0;
}
