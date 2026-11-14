#include "vision_core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <chrono> // 必须引入，用于计算高精度时间差

cv::Point pixelPoint(const cv::Point2f& point) {
    return {cvRound(point.x), cvRound(point.y)};
}

FrameResult processFrame(cv::Mat& frame, SerialPort& serial) {
    // ================= 跨帧状态保存 =================
    static uint32_t frame_id_counter = 0;       // 帧号
    static double last_x_err = 0.0;             // 上一帧的 X 坐标
    static bool has_last_x = false;             // 是否有有效的历史坐标
    static double filtered_v_x = 0.0;           // 滤波后的横向速度
    static auto last_time = std::chrono::high_resolution_clock::now(); // 上一帧的时间戳
    // ================================================

    frame_id_counter++; // 帧号自增

    // 1. 相机内参和物理尺寸
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        1820.233722818336, 0, 1236.55574552644,
        0, 1773.261200756997, 916.1541888473997,
        0, 0, 1);

    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 
        -0.1192465801181609, 1.043492593771526, 
        -0.007001639753345667, -0.001851319401215841,
         -3.627406272769521);

    // 【注意】：上车或比赛前，记得把这里的 23.0f 全部改回 40.0f
    std::vector<cv::Point3f> object_points = {
        {-23.0f, -23.0f, 0.0f}, { 23.0f, -23.0f, 0.0f}, 
        { 23.0f,  23.0f, 0.0f}, {-23.0f,  23.0f, 0.0f}
    };

    FrameResult result;
    result.armor_count = 0;

    // 2. 图像处理 (灰度、二值化、滤波等)
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
    std::array<cv::Point2f, 4> corners;

    // 3. 拟合轮廓（凸包或最小外接矩形）
    if (all_points.size() > 10) { 
        std::vector<cv::Point> hull;
        cv::convexHull(all_points, hull);
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(hull, true);
        cv::approxPolyDP(hull, approx, epsilon, true);

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

    // 4. 准备串口数据
    VisionToControl tx_data;
    tx_data.header[0] = 0xAA;
    tx_data.header[1] = 0x55;
    tx_data.frame_id = frame_id_counter;

    if (detected) {
        result.armor_count = 1;
        // 画框
        for (int k = 0; k < 4; ++k) {
            cv::line(frame, pixelPoint(corners[k]), pixelPoint(corners[(k + 1) % 4]),
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::circle(frame, pixelPoint(corners[k]), 5, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        }

        // 5. PnP 位姿解算
        std::vector<cv::Point2f> corners_vec(corners.begin(), corners.end());
        cv::Mat rvec, tvec;
        bool pnp_success = cv::solvePnP(object_points, corners_vec, camera_matrix, dist_coeffs, rvec, tvec);

        if (pnp_success) {
            double z_distance = std::abs(tvec.at<double>(2,0));
            if (z_distance > 100 && z_distance < 5000) {
                // 画坐标轴
                std::vector<cv::Point3f> axis_points = {{0, 0, 0}, {50, 0, 0}, {0, 50, 0}, {0, 0, 50}};
                std::vector<cv::Point2f> image_points_axis;
                cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, image_points_axis);
                cv::line(frame, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[1]), cv::Scalar(0, 0, 255), 3);
                cv::line(frame, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[2]), cv::Scalar(0, 255, 0), 3);
                cv::line(frame, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[3]), cv::Scalar(255, 0, 0), 3);

                // 填充坐标数据
                tx_data.is_detected = 1;
                tx_data.x_err = tvec.at<double>(0, 0);
                tx_data.y_err = tvec.at<double>(1, 0);
                tx_data.z_dist = tvec.at<double>(2, 0);

                cv::Mat R;
                cv::Rodrigues(rvec, R);
                tx_data.yaw = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * (180.0 / CV_PI);

                // ================= 动态速度估算（核心优化） =================
                auto current_time = std::chrono::high_resolution_clock::now();
                double dt = std::chrono::duration<double>(current_time - last_time).count();

                // 条件：有上一帧数据，且时间差合理（防止卡顿时 dt 突变）
                if (has_last_x && dt > 0.005 && dt < 0.5) {
                    double instant_v_x = (tx_data.x_err - last_x_err) / dt;

                    // 【逻辑 1：突变检测（应对折返或随机快速反向）】
                    // 如果瞬时速度和上一帧滤波后的速度方向相反，且绝对值都比较大，说明目标正在折返
                    if (instant_v_x * filtered_v_x < 0 && std::abs(instant_v_x) > 100.0) {
                        filtered_v_x = instant_v_x; // 抛弃历史滤波，瞬间跟上新方向
                    } 
                    // 【逻辑 2：自适应低通滤波（应对随机运动靶）】
                    // 如果目标速度变化剧烈，降低滤波权重（响应快）；如果目标很平稳，提高滤波权重（更平滑）
                    else {
                        double diff = std::abs(instant_v_x - filtered_v_x);
                        double alpha = 0.2; // 基础权重
                        if (diff > 300.0) alpha = 0.5; // 速度突变大，提高新数据权重，避免滞后
                        
                        filtered_v_x = (1.0 - alpha) * filtered_v_x + alpha * instant_v_x;
                    }
                } else {
                    // 刚刚重新捕捉到目标，先不给速度，防止暴冲
                    filtered_v_x = 0.0; 
                }

                tx_data.v_x = filtered_v_x;
                
                // 记录本帧状态，留给下一帧用
                last_x_err = tx_data.x_err;
                has_last_x = true;
                last_time = current_time;
                // ============================================================

                // 计算校验和并发送
                uint8_t *p = (uint8_t*)&tx_data;
                tx_data.checksum = 0;
                for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) {
                    tx_data.checksum += p[i];
                }
                serial.sendData(tx_data);

                result.annotated = frame;
                return result; // 成功处理，提前返回
            }
        }
    } 
    
    // ================= 目标丢失处理 =================
    cv::putText(frame, "Not Detected", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    
    tx_data.is_detected = 0;
    tx_data.x_err = 0; tx_data.y_err = 0; tx_data.z_dist = 0; tx_data.yaw = 0;
    tx_data.v_x = 0; 

    // 【极其重要】：丢失目标时，清空所有历史记忆！
    // 否则下次再看到目标，因为时间差非常大，会算出极高的速度导致翻车。
    has_last_x = false;
    filtered_v_x = 0.0;

    uint8_t *p = (uint8_t*)&tx_data;
    tx_data.checksum = 0;
    for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) {
        tx_data.checksum += p[i];
    }
    serial.sendData(tx_data);

    result.annotated = frame;
    return result;
}