#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>


int initPort(termios *tty) {
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port == -1) {
        std::cerr << "Error opening the serial port" << std::endl;
        return -1;
    }
    
    tcgetattr(serial_port, tty);

    // Set the baud rate
    cfsetospeed(tty, B115200);
    cfsetispeed(tty, B115200);

    // Set other serial port settings
    tty->c_cflag &= ~PARENB; // No parity
    tty->c_cflag &= ~CSTOPB; // 1 stop bit
    tty->c_cflag &= ~CSIZE;
    tty->c_cflag |= CS8;     // 8 data bits
    tty->c_cflag &= ~CRTSCTS; // No flow control
    tty->c_cflag |= CREAD | CLOCAL; // Enable reading and ignore modem control lines

    tcsetattr(serial_port, TCSANOW, tty);

    return serial_port;
}

int writePortByte(int serialPort, const char *message) {
    int bytes_written = write(serialPort, message, strlen(message));

    if (bytes_written == -1) {
        std::cerr << "Error writing to the serial port" << std::endl;
        close(serialPort);
        return -1;
    }

    std::cout << "write: " << message << bytes_written << std::endl;
    return bytes_written;
}

int readPortByte(int serialPort) {
    char readBuff[256];
    int readBytes = read(serialPort, &readBuff, sizeof(readBuff));
    if (readBytes == -1) {
        std::cerr << "Error reading serial port" << std::endl;
        return -1;
    }

    std::cout << "Read: " << readBuff << std::endl;
    return readBytes;
}

int main() {
    struct termios tty;

    int serialPort = initPort(&tty);
    if (serialPort == -1) {
        return 1;
    }
    
    const char* message = "Bello, Serial Port!\n";
    int bytesWritten = writePortByte(serialPort, message);
    if (bytesWritten == -1) {
        return 1;
    }

    int readBytes = readPortByte(serialPort);
    if (readBytes == -1) {
        return 1;
    }

    close(serialPort);
    return 0;
}
