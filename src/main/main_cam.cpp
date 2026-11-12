#include "maix_camera.hpp"
#include "maix_display.hpp"
#include "maix_image.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "vision_core.hpp"
#include "comm/uart.hpp"

int main() {
    // 实例化串口
    SerialPort serial("/dev/ttyS0", 115200);

    // 初始化相机（注意分辨率必须与你标定内参时一致）
    maix::camera::Camera cam(2560, 1440);
    maix::display::Display disp("", maix::display::DISPLAY_TYPE_BUFFERED);

    while (true) {
        maix::image::Image *img = cam.read();
        if (!img) continue;

        // 将 Maix 图像转为 OpenCV 的 Mat（RGB）
        cv::Mat rgb_mat(img->height(), img->width(), CV_8UC3, img->data());
        cv::Mat bgr_frame;
        cv::cvtColor(rgb_mat, bgr_frame, cv::COLOR_RGB2BGR);

        // 调用核心函数，传入 frame 和 serial
        FrameResult result = processFrame(bgr_frame, serial);

        // 将结果转回 RGB 用于屏幕显示
        cv::Mat show_rgb;
        cv::cvtColor(result.annotated, show_rgb, cv::COLOR_BGR2RGB);
        maix::image::Image show_img(show_rgb.cols, show_rgb.rows, maix::image::Format::FMT_RGB888, show_rgb.data);
        disp.show(&show_img);

        delete img; // 极其重要：释放 Maix 图像内存，防内存泄漏
    }
    return 0;
}