#include "vision_core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>
#include <chrono>

// ================= 【性能开关】 =================
// 比赛时取消注释，关闭可视化，极致提升帧率
// #define DISABLE_VISUAL
// ================================================

cv::Point pixelPoint(const cv::Point2f& point) {
    return {cvRound(point.x), cvRound(point.y)};
}

// ================== 坐标转换参数 (请根据实车物理尺寸修改) ==================
/*原点：战车底盘的旋转中心（或者升降投放结构的几何中心）。
X轴:车体正右方
Y轴:车体正下方（沿用视觉习惯）
Z轴:车体正前方*/
const float CAM_OFFSET_X = 0.0f;    // 相机在车体坐标系下的 X 偏移 (mm)
const float CAM_OFFSET_Y = 0.0f;    // 相机在车体坐标系下的 Y 偏移 (mm)
const float CAM_OFFSET_Z = 0.0f;    // 相机在车体坐标系下的 Z 偏移 (mm)
const float CAM_YAW_DEGREE = 0.0f;  // 相机安装的偏航角 (度)（向右偏为正，向左偏为负）。

// 将相机坐标系下的 (x, y, z) 转换到“投放结构中心”坐标系
void transformToDropMechanism(float& x, float& y, float& z) {
    float theta = -CAM_YAW_DEGREE * (CV_PI / 180.0f); // 弧度
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    float x_rot = x * cos_t + z * sin_t;
    float z_rot = -x * sin_t + z * cos_t;

    x = x_rot + CAM_OFFSET_X;
    y = y + CAM_OFFSET_Y;
    z = z_rot + CAM_OFFSET_Z;
}
// ==============================================================================

// 内部函数：找兑换框
bool detectExchangeBoard(cv::Mat& frame, VisionToControl& tx_data, std::array<cv::Point2f, 4>& corners) {
    cv::Mat denoised, gray, binary, morphology;
    cv::medianBlur(frame, denoised, 3);
    cv::cvtColor(denoised, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, morphology, cv::MORPH_CLOSE, close_kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morphology, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> all_points;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > 50.0) {
            all_points.insert(all_points.end(), contour.begin(), contour.end());
        }
    }

    bool detected = false;
    if (all_points.size() > 10) { 
        std::vector<cv::Point> hull, approx;
        cv::convexHull(all_points, hull);
        cv::approxPolyDP(hull, approx, 0.02 * cv::arcLength(hull, true), true);

        if (approx.size() == 4) {
            detected = true;
            std::vector<cv::Point2f> pts;
            for (auto& p : approx) pts.push_back(cv::Point2f((float)p.x, (float)p.y));
            std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
            if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
            if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);
            corners = {pts[0], pts[1], pts[3], pts[2]};
        } else {
            cv::RotatedRect rect = cv::minAreaRect(all_points);
            if (rect.size.area() > 2000.0) {
                detected = true;
                cv::Point2f vertices[4];
                rect.points(vertices);
                std::vector<cv::Point2f> pts(vertices, vertices + 4);
                std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
                if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
                if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);
                corners = {pts[0], pts[1], pts[3], pts[2]};
            }
        }
    }
    if (!detected) return false;

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1820.23, 0, 1236.55, 0, 1773.26, 916.15, 0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.119, 1.043, -0.007, -0.001, -3.627);
    // 【上车前注意】：把 23.0f 改回 40.0f
    std::vector<cv::Point3f> object_points = {{-23.0f, -23.0f, 0}, {23.0f, -23.0f, 0}, {23.0f, 23.0f, 0}, {-23.0f, 23.0f, 0}};
    
    std::vector<cv::Point2f> corners_vec(corners.begin(), corners.end());
    cv::Mat rvec, tvec;
    if (cv::solvePnP(object_points, corners_vec, camera_matrix, dist_coeffs, rvec, tvec)) {
        if (std::abs(tvec.at<double>(2,0)) > 100) {
            tx_data.target_type = 1;
            
            // 1. 提取原始相机坐标
            float cam_x = tvec.at<double>(0, 0);
            float cam_y = tvec.at<double>(1, 0);
            float cam_z = tvec.at<double>(2, 0);

            // ================= 新增：坐标变换 =================
            transformToDropMechanism(cam_x, cam_y, cam_z);
            // ==================================================

            tx_data.x_err = cam_x;
            tx_data.y_err = cam_y;
            tx_data.z_dist = cam_z;
            
            cv::Mat R; cv::Rodrigues(rvec, R);
            float raw_yaw = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * (180.0 / CV_PI);
            // 【新增】：补偿相机的物理安装偏角
            tx_data.yaw = raw_yaw - CAM_YAW_DEGREE; 
            
            return true;
        }
    }
    return false;
}

// 内部函数：找退出 Tag
bool detectExitTag(cv::Mat& frame, VisionToControl& tx_data) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids);
    
    if (!ids.empty()) {
        tx_data.target_type = 2;
        cv::Point2f center(0, 0);
        for (const auto& p : corners[0]) center += p;
        center /= 4.0f;
        // 这里发送的是基于图像中心的像素偏差，电控只做对正，不需要物理毫米距离
        tx_data.x_err = center.x - (frame.cols / 2.0f);
        tx_data.y_err = center.y - (frame.rows / 2.0f);
        tx_data.z_dist = 100000.0f / cv::contourArea(corners[0]);
        tx_data.yaw = 0; tx_data.v_x = 0;
        return true;
    }
    return false;
}

// ================= 主处理函数 =================
FrameResult processFrame(cv::Mat& frame, SerialPort& serial, int current_mode) {
    static uint32_t frame_id_counter = 0;       
    static double last_x_err = 0.0;             
    static bool has_last_x = false;             
    static double filtered_v_x = 0.0;           
    static auto last_time = std::chrono::high_resolution_clock::now(); 

    frame_id_counter++; 

    FrameResult result;
    result.armor_count = 0;

    VisionToControl tx_data;
    tx_data.header[0] = 0xAA; tx_data.header[1] = 0x55;
    tx_data.frame_id = frame_id_counter;
    tx_data.is_detected = 0; tx_data.target_type = 0;
    tx_data.v_x = 0;
    
    std::array<cv::Point2f, 4> corners;
    bool detected = false;
    
    if (current_mode == 1) {
        detected = detectExchangeBoard(frame, tx_data, corners);
    } else if (current_mode == 2) {
        detected = detectExitTag(frame, tx_data);
    }

    // ================= 【可视化准备】 =================
#ifndef DISABLE_VISUAL
    cv::Mat debug_frame;
    cv::resize(frame, debug_frame, cv::Size(640, 360)); 
    float scale = 640.0f / frame.cols; 
#endif
    // ==================================================

    if (detected) {
        result.armor_count = 1;
        tx_data.is_detected = 1;

        // 计算速度 (只针对兑换框，因为 Tag 是静止的)
        if (tx_data.target_type == 1) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_time).count();
            if (has_last_x && dt > 0.005 && dt < 0.5) {
                // 注意：这里算的是转换到车体坐标系后的相对速度
                double instant_v_x = (tx_data.x_err - last_x_err) / dt;
                if (instant_v_x * filtered_v_x < 0 && std::abs(instant_v_x) > 100.0) {
                    filtered_v_x = instant_v_x; 
                } else {
                    double alpha = (std::abs(instant_v_x - filtered_v_x) > 300.0) ? 0.5 : 0.2;
                    filtered_v_x = (1.0 - alpha) * filtered_v_x + alpha * instant_v_x;
                }
            } else {
                filtered_v_x = 0.0; 
            }
            tx_data.v_x = filtered_v_x;
            last_x_err = tx_data.x_err;
            has_last_x = true;
            last_time = current_time;
        }

#ifndef DISABLE_VISUAL
        // 画框可视化
        if (tx_data.target_type == 1) {
            for (int k = 0; k < 4; ++k) {
                cv::line(debug_frame, pixelPoint(corners[k] * scale), pixelPoint(corners[(k+1)%4] * scale), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        // ================= 【可视化：XYZ + Yaw + v_x】 =================
            char text_buffer[128];
            
            // 1. 打印 XYZ 数据
            snprintf(text_buffer, sizeof(text_buffer), "X: %.1f mm  Y: %.1f mm  Z: %.1f mm", 
                     tx_data.x_err, tx_data.y_err, tx_data.z_dist);
            cv::putText(debug_frame, text_buffer, cv::Point(20, 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2); // 青色
            
            // 2. 打印 Yaw 数据
            snprintf(text_buffer, sizeof(text_buffer), "Yaw: %.2f deg", tx_data.yaw);
            cv::putText(debug_frame, text_buffer, cv::Point(20, 70), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2); // 黄色
            
            // 3. 打印 v_x 速度数据
            snprintf(text_buffer, sizeof(text_buffer), "v_x: %.2f mm/s", tx_data.v_x);
            cv::putText(debug_frame, text_buffer, cv::Point(20, 100), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2); // 紫色
            
            // 4. Yaw 异常警告
            if (std::abs(tx_data.yaw) > 3.0f) {
                cv::putText(debug_frame, "WARNING: Yaw Drift!", cv::Point(20, 130), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2); // 红色
            }
            // ==========================================================   
        } else if (tx_data.target_type == 2) {
            cv::putText(debug_frame, "EXIT TAG FOUND", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
        }
#endif

    } else {
        // 目标丢失
        has_last_x = false;
        filtered_v_x = 0.0;
#ifndef DISABLE_VISUAL
        cv::putText(debug_frame, "Not Detected", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
#endif
    }

#ifndef DISABLE_VISUAL
    result.annotated = debug_frame;
#else
    result.annotated = cv::Mat(1, 1, CV_8UC3);
#endif

    // 发送串口
    uint8_t *p = (uint8_t*)&tx_data;
    tx_data.checksum = 0;
    for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) tx_data.checksum += p[i];
    serial.sendData(tx_data);

    return result;
}