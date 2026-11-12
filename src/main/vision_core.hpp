#pragma once
#include <opencv2/core.hpp>
#include <array>
#include "comm/uart.hpp"
#include "comm/protocol.hpp"

// 定义返回结构体（移出匿名命名空间）
struct FrameResult {
    cv::Mat annotated;
    std::size_t armor_count = 0;
};

// 声明核心处理函数
// 注意：frame 去掉了 const，允许在原图上直接画框，省去 clone 的开销
// serial 通过引用传参，避免全局变量冲突
FrameResult processFrame(cv::Mat& frame, SerialPort& serial);