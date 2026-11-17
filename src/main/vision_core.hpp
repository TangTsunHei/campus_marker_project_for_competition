#pragma once
#include <opencv2/core.hpp>
#include <array>
#include "comm/uart.hpp"
#include "comm/protocol.hpp"

struct FrameResult {
    cv::Mat annotated;
    std::size_t armor_count = 0;
};

// 外部传入当前的模式（1=找兑换框, 2=找退出Tag）
FrameResult processFrame(cv::Mat& frame, SerialPort& serial, int current_mode);