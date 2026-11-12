#include "vision_core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

cv::Point pixelPoint(const cv::Point2f& point) {
    return {cvRound(point.x), cvRound(point.y)};
}

FrameResult processFrame(cv::Mat& frame, SerialPort& serial) {
    // 1. 内参与物理尺寸
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        1820.233722818336, 0, 1236.55574552644,
        0, 1773.261200756997, 916.1541888473997,
        0, 0, 1);

    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 
        -0.1192465801181609, 1.043492593771526, 
        -0.007001639753345667, -0.001851319401215841,
         -3.627406272769521);

    // 【极其重要】上车前将 23.0f 改回 40.0f
    std::vector<cv::Point3f> object_points = {
        {-23.0f, -23.0f, 0.0f}, { 23.0f, -23.0f, 0.0f}, 
        { 23.0f,  23.0f, 0.0f}, {-23.0f,  23.0f, 0.0f}
    };

    FrameResult result;
    result.armor_count = 0;

    // 2. 图像处理（不 clone，直接处理）
    cv::Mat denoised;
    cv::medianBlur(frame, denoised, 3);

    cv::Mat gray;
    cv::cvtColor(denoised, gray, cv::COLOR_BGR2GRAY);

    cv::Mat binary;
    cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

    cv::Mat morphology;
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, morphology, cv::MORPH_CLOSE, close_kernel);

    // findContours 会修改输入图像，所以传 clone() 或 morphology 均可
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

    // 3. 凸包与拟合
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
            std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.y < b.y;
            });
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
                std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                    return a.y < b.y;
                });
                if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
                if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);
                corners = {pts[0], pts[1], pts[3], pts[2]};
            }
        }
    }

    // 4. 组装串口数据
    VisionToControl tx_data;
    tx_data.header[0] = 0xAA;
    tx_data.header[1] = 0x55;

    if (detected) {
        result.armor_count = 1;
        // 【优化】直接画在原图 frame 上，不 clone
        for (int k = 0; k < 4; ++k) {
            cv::line(frame, pixelPoint(corners[k]), pixelPoint(corners[(k + 1) % 4]),
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::circle(frame, pixelPoint(corners[k]), 5, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        }
        cv::putText(frame, "Marker Detected", pixelPoint(corners[0]) + cv::Point(0, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // 5. PnP 解算
        std::vector<cv::Point2f> corners_vec(corners.begin(), corners.end());
        cv::Mat rvec, tvec;
        bool pnp_success = cv::solvePnP(object_points, corners_vec, camera_matrix, dist_coeffs, rvec, tvec);
        
        if (pnp_success) {
            double z_distance = std::abs(tvec.at<double>(2,0));
            if (z_distance > 100 && z_distance < 5000){
                std::vector<cv::Point3f> axis_points = {{0, 0, 0}, {50, 0, 0}, {0, 50, 0}, {0, 0, 50}};
                std::vector<cv::Point2f> image_points_axis;
                cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, image_points_axis);
                
                cv::line(frame, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[1]), cv::Scalar(0, 0, 255), 3);
                cv::line(frame, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[2]), cv::Scalar(0, 255, 0), 3);
                cv::line(frame, pixelPoint(image_points_axis[0]), pixelPoint(image_points_axis[3]), cv::Scalar(255, 0, 0), 3);
                
                std::string dist_str = "Z: " + std::to_string(tvec.at<double>(2,0)).substr(0, 5) + "mm";
                cv::putText(frame, dist_str, pixelPoint(corners[0]) + cv::Point(0, 20), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

                // 计算偏差并发送给电控
                tx_data.is_detected = 1;
                tx_data.x_err = tvec.at<double>(0, 0);
                tx_data.y_err = tvec.at<double>(1, 0);
                tx_data.z_dist = tvec.at<double>(2, 0);
                
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                tx_data.yaw = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2)) * (180.0 / CV_PI);
                
                uint8_t *p = (uint8_t*)&tx_data;
                tx_data.checksum = 0;
                for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) {
                    tx_data.checksum += p[i];
                }
                serial.sendData(tx_data);
                
                result.annotated = frame;
                return result; // 提前结束
            }
        }
    } 
    
    // 未识别到目标
    cv::putText(frame, "Not Detected", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    
    tx_data.is_detected = 0;
    tx_data.x_err = 0; tx_data.y_err = 0; tx_data.z_dist = 0; tx_data.yaw = 0;
    
    uint8_t *p = (uint8_t*)&tx_data;
    tx_data.checksum = 0;
    for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) {
        tx_data.checksum += p[i];
    }
    serial.sendData(tx_data);

    result.annotated = frame;
    return result;
}