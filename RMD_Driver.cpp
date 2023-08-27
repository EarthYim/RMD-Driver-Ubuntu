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
    tty->c_cc[VTIME] = 5;    // Wait for up to 500 milliseconds returning as soon as any data is received.
    tty->c_cc[VMIN] = 0;

    tcsetattr(serial_port, TCSANOW, tty);

    return serial_port;
}

// int writePortByte(int serialPort, const char *message) {
//     int bytes_written = write(serialPort, message, strlen(message));

//     if (bytes_written == -1) {
//         std::cerr << "Error writing to the serial port" << std::endl;
//         close(serialPort);
//         return -1;
//     }

//     std::cout << "write: " << message << bytes_written << std::endl;
//     return bytes_written;
// }


int writePortByte(int serialPort, char message[]) {
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
    char currChar;
    int bytesRead;
    bytesRead = read(serialPort, &readBuff, sizeof(readBuff));
    if (bytesRead == -1) {
        std::cerr << "Error reading serial port" << std::endl;
        return -1;
    }

    std::cout << "Read: " << readBuff << std::endl;
    return bytesRead;
}

int setPosition(int serialPort) {
    char motorCommand[14];

    motorCommand[0] = 0x3e;
	motorCommand[1] = 0xa3;
	motorCommand[2] = 0x01;
	motorCommand[3] = 0x08;
	motorCommand[4] = 0xe5;
	motorCommand[5] = 0xA0;	
	motorCommand[6] = 0x8c;	
	motorCommand[7] = 0x00;
	motorCommand[8] = 0x00;
    motorCommand[9] = 0x00;	
	motorCommand[10] = 0x00;	
	motorCommand[11] = 0x00;
	motorCommand[12] = 0x00;
	motorCommand[13] = 0x12;	

    int writeBytes = writePortByte(serialPort, motorCommand);

    return writeBytes;

}

int main() {
    struct termios tty;

    int serialPort = initPort(&tty);
    if (serialPort == -1) {
        return 1;
    }
    
    // char message[25] = "BelloSerialPort!\n";
    // int writeBytes = writePortByte(serialPort, message);
    // if (writeBytes == -1) {
    //     return 1;
    // }

    int writeBytes = setPosition(serialPort);
    if (writeBytes == -1) {
        return 1;
    }

    int readBytes = readPortByte(serialPort);
    if (readBytes == -1) {
        return 1;
    }

    close(serialPort);
    return 0;
}
