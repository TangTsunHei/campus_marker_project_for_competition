#pragma once
#include "protocol.hpp"

class SerialPort {
public:
    SerialPort(const char* portName, int baudRate);
    ~SerialPort();
    bool isOpen() const;
    bool sendData(const VisionToControl& data);

private:
    int fd; // 串口文件描述符
};