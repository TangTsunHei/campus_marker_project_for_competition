#include "uart.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <stdio.h>

// 辅助函数：将整数波特率转为 Linux 的 termios 波特率常量
speed_t getBaudRate(int baudRate) {
    switch (baudRate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200; // 默认 115200
    }
}

SerialPort::SerialPort(const char* portName, int baudRate) {
    // O_RDWR: 读写模式 | O_NOCTTY: 不成为控制终端 | O_NDELAY: 非阻塞模式
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (fd == -1) {
        // 如果打开失败（例如在 WSL 电脑上跑视频测试，没有真实串口）
        std::cerr << "[警告] 无法打开串口 " << portName << " (可能是在电脑上测试)" << std::endl;
        std::cerr << "[提示] 程序将进入模拟模式：只打印数据，不发送。" << std::endl;
        return;
    }

    // 恢复串口为阻塞模式
    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    speed_t speed = getBaudRate(baudRate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 设置数据位、停止位、无校验、无流控
    options.c_cflag |= (CLOCAL | CREAD); // 启用接收器，忽略调制解调器控制线
    options.c_cflag &= ~CSIZE;           // 屏蔽数据位设置
    options.c_cflag |= CS8;              // 8位数据位
    options.c_cflag &= ~CSTOPB;          // 1位停止位
    options.c_cflag &= ~PARENB;          // 无奇偶校验
    options.c_cflag &= ~CRTSCTS;         // 无硬件流控

    // 设置为原始模式（Raw Mode），不进行任何特殊字符处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 无软件流控
    options.c_oflag &= ~OPOST;                  // 原始输出

    // 设置读取超时（可根据需要调整）
    options.c_cc[VMIN] = 0;  // 读字符最少个数
    options.c_cc[VTIME] = 5; // 读取超时时间（单位：0.1秒）

    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH); // 清空未处理的输入输出

    std::cout << "[成功] 串口 " << portName << " 初始化完成，波特率: " << baudRate << std::endl;
}

SerialPort::~SerialPort() {
    if (fd != -1) {
        close(fd);
    }
}

bool SerialPort::isOpen() const {
    return fd != -1;
}

bool SerialPort::sendData(const VisionToControl& data) {
    // =========================================================
    // 核心调试打印：无论在电脑还是板子上，都能看到发出的数据！
    // =========================================================
    printf("[TX] Detected:%d | X:%7.2f mm | Y:%7.2f mm | Z:%7.2f mm | Yaw:%6.2f deg\n", 
           data.is_detected, data.x_err, data.y_err, data.z_dist, data.yaw);

    // 如果串口没打开（在 WSL 电脑上），打印完直接返回，防止段错误
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