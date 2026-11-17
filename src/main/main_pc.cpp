#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include "vision_core.hpp"
#include "comm/uart.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "用法: " << argv[0] << " <输入视频.mp4> <输出目录> <模式(1=找兑换框, 2=找退出Tag)>\n";
        return 1;
    }

    try {
        // 在电脑端，它依然会打印一句“[警告] 无法打开串口”，但不会崩溃
        SerialPort serial("/dev/ttyS0", 115200);

        const fs::path input_path(argv[1]);
        const fs::path output_directory(argv[2]);
        int current_mode = std::stoi(argv[3]); // 从命令行参数获取模式

        if (!fs::is_regular_file(input_path)) {
            throw std::runtime_error("输入视频不存在: " + input_path.string());
        }

        cv::VideoCapture capture(input_path.string());
        if (!capture.isOpened()) {
            throw std::runtime_error("无法读取输入视频: " + input_path.string());
        }
        
        fs::create_directories(output_directory);
        const std::string stem = input_path.stem().string();
        const fs::path video_path = output_directory / (stem + "_annotated.mp4");

        double fps = capture.get(cv::CAP_PROP_FPS);
        if (!std::isfinite(fps) || fps <= 0.0) fps = 30.0;
        
        cv::VideoWriter annotated_writer;
        cv::Mat frame;
        int frame_index = 0;
        int detected_frames = 0;

        std::cout << "开始处理视频，当前模式: " << current_mode << std::endl;

        while (capture.read(frame)) {
            if (frame.empty()) continue;

            // 调用核心函数，传入 frame、serial 和 mode
            FrameResult result = processFrame(frame, serial, current_mode);

            // 写视频 (必须判断图片是否有效)
            // 如果 vision_core.cpp 里开启了 DISABLE_VISUAL，result.annotated 是 1x1 的，不能写
            if (result.annotated.cols > 10 && result.annotated.rows > 10) {
                if (!annotated_writer.isOpened()) {
                    annotated_writer.open(video_path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, result.annotated.size(), true);
                }
                annotated_writer.write(result.annotated);
            }
            
            if (result.armor_count > 0) ++detected_frames;
            ++frame_index;
        }
        
        annotated_writer.release();
        capture.release();

        std::cout << input_path.filename().string() << ": " << frame_index
                  << "帧，检测率: " << detected_frames << "/" << frame_index
                  << " (" << std::fixed << std::setprecision(2) << 100.0 * detected_frames / frame_index << "%)\n";

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return 1;
    }
}