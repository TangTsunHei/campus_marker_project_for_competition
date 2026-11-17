#pragma once
#include "protocol.hpp"

class SerialPort {
public:
    SerialPort(const char* portName, int baudRate);
    ~SerialPort();
    bool isOpen() const;
    
    // 发送视觉数据到电控
    bool sendData(const VisionToControl& data);
    
    // 接收电控指令（非阻塞）
    bool receiveData(ControlToVision& cmd);

private:
    int fd; 
};