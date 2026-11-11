#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/calib3d.hpp>

#include "comm/uart.hpp"
#include "comm/protocol.hpp"

// 全局或成员变量
SerialPort serial("/dev/ttyS0", 115200);

namespace fs = std::filesystem;

namespace {

struct FrameResult {
    cv::Mat annotated;
    std::size_t armor_count = 0;
};//删除了std::array<cv::Mat, 10> stages; ，因为在后续的处理过程中，暂时不需要保留每个阶段的图像，只需要最终的标注图像和最终装甲板识别视频

cv::Point pixelPoint(const cv::Point2f& point) {
    return {cvRound(point.x), cvRound(point.y)};
}  
//用于将浮点值point坐标转换成整型point坐标，方便后续函数使用



FrameResult processFrame(const cv::Mat& frame) {
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        1820.233722818336, 0, 1236.55574552644,
        0, 1773.261200756997, 916.1541888473997,
        0, 0, 1);

    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 
        -0.1192465801181609, 1.043492593771526, 
        -0.007001639753345667, -0.001851319401215841,
         -3.627406272769521);

    // 80x80mm 的四个物理角点（比赛实际标识板灯条矩形就是80*80,但是模拟的纸质灯板长宽只有46*46，所以模拟后要记得修改数据）
    std::vector<cv::Point3f> object_points = {
        {-23.0f, -23.0f, 0.0f}, { 23.0f, -23.0f, 0.0f}, 
        { 23.0f,  23.0f, 0.0f}, {-23.0f,  23.0f, 0.0f}
    };//模拟后将23全部换回40
      //模拟结束，以46*46mm的模拟纸片灯条测距误差在+-3cm,现已完成测距模拟，数据填回正式灯条40*40mm尺寸数据

    FrameResult result;
    // 1. 中值滤波去噪
    cv::Mat denoised;
    cv::medianBlur(frame, denoised, 3);

    // 2. 转灰度图gray
    cv::Mat gray;
    cv::cvtColor(denoised, gray, cv::COLOR_BGR2GRAY);

    // 3. 二值化，提取白色灯条
    cv::Mat binary;
    cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

    // 4. 保留一点轻微的闭运算（仅用来平滑边缘或连接小断口，不需要连成整体）
    cv::Mat morphology;
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, morphology, cv::MORPH_CLOSE, close_kernel);

    // 5. 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morphology.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //此处用morphology.clone()是为了避免对原始morphology图像进行修改，并方便测试阶段找bug
    // 6. 收集所有有效轮廓的点（过滤掉面积太小的噪点）
    std::vector<cv::Point> all_points;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > 50.0) { // 过滤掉微小噪点，只保留真实的灯条
            all_points.insert(all_points.end(), contour.begin(), contour.end());
        }
    }

    bool detected = false;
    std::array<cv::Point2f, 4> corners;
    // 因为想要保证图像在不发生巨大形变的前提下，利用闭运算将L型灯条全部连成一圈几乎不可能，所以在AI介绍下，选用凸包处理
    // 7. 如果收集到了足够多的点，开始求凸包并拟合四边形
    if (all_points.size() > 10) { 
        // 求凸包，把所有散落的 L 形灯条包成一个大四边形
        std::vector<cv::Point> hull;
        cv::convexHull(all_points, hull);
        //凸包的作用：它就像拿一根橡皮筋套住整个轮廓。
        //不管边缘多么破碎、内部有多少空洞，计算凸包后，你会得到一个完整、平滑、实心且绝对向外凸的轮廓
        // 对凸包进行多边形逼近
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(hull, true);
        cv::approxPolyDP(hull, approx, epsilon, true);

        // 如果拟合出 4 个点，说明完美
        if (approx.size() == 4) {
            detected = true;
            std::vector<cv::Point2f> pts;
            for (auto& p : approx) pts.push_back(cv::Point2f((float)p.x, (float)p.y));

            // 排序四个点
            std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.y < b.y;
            });
            if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
            if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);
            // 顺序：左上、右上、右下、左下
            corners = {pts[0], pts[1], pts[3], pts[2]};
        } 
        // 如果拟合不是4个点（形状不完美），使用最小外接旋转矩形兜底
        else {
            cv::RotatedRect rect = cv::minAreaRect(all_points);
            // 面积太小则忽略
            if (rect.size.area() > 2000.0) {
                detected = true;
                cv::Point2f vertices[4];
                rect.points(vertices);
                std::vector<cv::Point2f> pts(vertices, vertices + 4);
                std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                    return a.y < b.y;
                });
                if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
                if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);
                corners = {pts[0], pts[1], pts[3], pts[2]};
            }
        }
    }

    // 8. 绘制结果
    cv::Mat annotated = frame.clone();

    //准备发送给电控的数据结构
    VisionToControl tx_data;
    tx_data.header[0] = 0xAA;
    tx_data.header[1] = 0x55;

    if (detected) {
        // 1. 画框
        for (int k = 0; k < 4; ++k) {
            cv::line(annotated, pixelPoint(corners[k]), pixelPoint(corners[(k + 1) % 4]),
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::circle(annotated, pixelPoint(corners[k]), 5, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        }
        cv::putText(annotated, "Marker Detected", pixelPoint(corners[0]) + cv::Point(0, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        result.armor_count = 1;

        // 2. PnP 位姿估计（必须放在 if(detected) 里面！）
        std::vector<cv::Point2f> corners_vec(corners.begin(), corners.end());
        cv::Mat rvec, tvec;
        bool pnp_success = cv::solvePnP(object_points, corners_vec, camera_matrix, dist_coeffs, rvec, tvec);
        
        if (pnp_success) {
            double z_distance = std::abs(tvec.at<double>(2,0));
            if (z_distance > 100 && z_distance < 5000){
                // 画坐标轴
                std::vector<cv::Point3f> axis_points = {{0, 0, 0}, {50, 0, 0}, {0, 50, 0}, {0, 0, 50}};
                std::vector<cv::Point2f> image_points_axis;
                cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, image_points_axis);
                
                cv::line(annotated, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[1]), cv::Scalar(0, 0, 255), 3);
                cv::line(annotated, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[2]), cv::Scalar(0, 255, 0), 3);
                cv::line(annotated, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[3]), cv::Scalar(255, 0, 0), 3);
                
                std::string dist_str = "Z: " + std::to_string(tvec.at<double>(2,0)).substr(0, 5) + "mm";
                cv::putText(annotated, dist_str, pixelPoint(corners[0]) + cv::Point(0, 20), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

                // ===== 新增：计算偏差并发送给电控 =====
                tx_data.is_detected = 1;
                tx_data.x_err = tvec.at<double>(0, 0); // 横向偏差 (mm)
                tx_data.y_err = tvec.at<double>(1, 0); // 纵向偏差 (mm)
                tx_data.z_dist = tvec.at<double>(2, 0); // 距离 (mm)
                
                // 计算偏航角 Yaw (将旋转向量转为旋转矩阵，再求欧拉角)
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                tx_data.yaw = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * (180.0 / CV_PI);
                
                // 计算校验和
                uint8_t *p = (uint8_t*)&tx_data;
                tx_data.checksum = 0;
                for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) {
                    tx_data.checksum += p[i];
                }
                // 发送数据
                serial.sendData(tx_data);
            }
        }
    } 
    // ================= 核心修改位置：如果没识别到目标 =================
    else {
        cv::putText(annotated, "Not Detected", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
                    1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        result.armor_count = 0;

        // ===== 新增：告诉电控目标丢失 =====
        tx_data.is_detected = 0;
        tx_data.x_err = 0;
        tx_data.y_err = 0;
        tx_data.z_dist = 0;
        tx_data.yaw = 0;
        
        uint8_t *p = (uint8_t*)&tx_data;
        tx_data.checksum = 0;
        for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) {
            tx_data.checksum += p[i];
        }
        serial.sendData(tx_data);
    }

    result.annotated = annotated;
    return result;
}
    //删除了学长模板中的中间步展示绘制部分，保留并返回最终标识视频
    void openVideoWriter(cv::VideoWriter& writer, const fs::path& path,
                     double fps, const cv::Size& size) {
    // 直接用 OpenCV 写入 mp4v 编码的 MP4，帧率和尺寸由调用方传入。
    writer.open(path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                fps, size, true);
    if (!writer.isOpened()) {
        throw std::runtime_error("无法创建视频: " + path.string());
    }
}



}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <输入视频.mp4> <输出目录>\n";
        return 1;
    }

    try {
        const fs::path input_path(argv[1]);
        const fs::path output_directory(argv[2]);
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
        if (!std::isfinite(fps) || fps <= 0.0) {
            fps = 30.0;
        }
        cv::VideoWriter annotated_writer;
        cv::Mat frame;
        int frame_index = 0;
        std::size_t total_armors = 0;
        int detected_frames = 0;
        while (capture.read(frame)) {
            if (frame.empty()) {
                throw std::runtime_error("视频中出现空帧");
            }

            FrameResult result = processFrame(frame);
            if (!annotated_writer.isOpened()) {
                openVideoWriter(annotated_writer, video_path, fps, frame.size());
            }
            annotated_writer.write(result.annotated);
            total_armors += result.armor_count;
            if (result.armor_count > 0) {
                ++detected_frames;
            }
            ++frame_index;
        }
        if (frame_index == 0) {
            throw std::runtime_error("视频没有可读取的帧");
        }
        annotated_writer.release();
        capture.release();

        std::cout << input_path.filename().string() << ": " << frame_index
                  << "帧，检测到标志物的帧数: " << detected_frames << "/" << frame_index
                  << " (" << std::fixed << std::setprecision(2)
                  << 100.0 * detected_frames / frame_index << "%)\n"
                  << "标注视频: " << video_path << "\n";

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return 1;
    }
}