#include "uart.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <stdio.h>

speed_t getBaudRate(int baudRate) {
    switch (baudRate) {
        case 9600: return B9600;
        case 115200: return B115200;
        case 921600: return B921600;
        default: return B115200;
    }
}

SerialPort::SerialPort(const char* portName, int baudRate) {
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "[警告] 无法打开串口 " << portName << " (进入模拟打印模式)" << std::endl;
        return;
    }

    // 设置为非阻塞模式，防止视觉死等数据卡住主循环
    fcntl(fd, F_SETFL, O_NONBLOCK);

    struct termios options;
    tcgetattr(fd, &options);
    speed_t speed = getBaudRate(baudRate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);
    std::cout << "[成功] 串口 " << portName << " 初始化完成 (非阻塞模式)" << std::endl;
}

SerialPort::~SerialPort() {
    if (fd != -1) close(fd);
}

bool SerialPort::isOpen() const { return fd != -1; }

bool SerialPort::sendData(const VisionToControl& data) {
    if (fd == -1) return false;
    ssize_t bytes_written = write(fd, (const void*)&data, sizeof(VisionToControl));
    return bytes_written == sizeof(VisionToControl);
}

bool SerialPort::receiveData(ControlToVision& cmd) {
    if (fd == -1) return false;
    
    uint8_t buffer[64];
    ssize_t n = read(fd, buffer, sizeof(buffer));
    
    if (n >= (ssize_t)sizeof(ControlToVision)) {
        // 简单的滑动窗口寻找帧头 0xBB 0x66
        for (ssize_t i = 0; i <= n - (ssize_t)sizeof(ControlToVision); ++i) {
            if (buffer[i] == 0xBB && buffer[i+1] == 0x66) {
                ControlToVision temp_cmd;
                memcpy(&temp_cmd, &buffer[i], sizeof(ControlToVision));
                if (calculate_ctrl_checksum(temp_cmd) == temp_cmd.checksum) {
                    cmd = temp_cmd;
                    return true; // 成功解析到有效指令
                }
            }
        }
    }
    return false;
}