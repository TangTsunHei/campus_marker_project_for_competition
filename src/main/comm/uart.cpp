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
    // =========================================================
    // 核心调试打印：增加 v_x，并保持终端输出对齐
    // =========================================================
    printf("[TX] Det:%d | X:%7.2f mm | Y:%7.2f mm | Z:%7.2f mm | Yaw:%7.2f deg | v_x:%8.2f mm/s\n", 
           data.is_detected, data.x_err, data.y_err, data.z_dist, data.yaw, data.v_x);
    fflush(stdout); // 强制刷新输出缓冲区，防止终端卡顿或丢行

    // 如果串口没打开（比如在电脑上跑测试视频），打印完直接返回，防止段错误
    if (fd == -1) {
        return false; 
    }

    // 串口发送（写入二进制数据）
    ssize_t bytes_written = write(fd, (const void*)&data, sizeof(VisionToControl));
    
    if (bytes_written == -1) {
        std::cerr << "[错误] 串口发送失败！" << std::endl;
        return false;
    } else if (bytes_written != sizeof(VisionToControl)) {
        std::cerr << "[警告] 串口发送不完整，期望发送 " << sizeof(VisionToControl) 
                  << " 字节，实际发送 " << bytes_written << " 字节" << std::endl;
        return false;
    }

    return true;
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