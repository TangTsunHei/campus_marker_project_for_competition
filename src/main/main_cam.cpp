#include "vision_core.hpp"
#include "comm/uart.hpp"
#include "comm/protocol.hpp"
#include "maix_camera.hpp"
#include "maix_display.hpp"
#include "maix_image.hpp"
#include <opencv2/imgproc.hpp>

int main() {
    SerialPort serial("/dev/ttyS0", 115200);
    maix::camera::Camera cam(2560, 1440);
    maix::display::Display disp("", maix::display::DISPLAY_TYPE_BUFFERED);

    int current_mode = 1; // 默认寻找兑换框
    ControlToVision rx_cmd;

    while (true) {
        // 轮询电控指令
        if (serial.receiveData(rx_cmd)) {
            current_mode = rx_cmd.mode;
        }

        maix::image::Image *img = cam.read();
        if (!img) continue;

        cv::Mat rgb_mat(img->height(), img->width(), CV_8UC3, img->data());
        cv::Mat bgr_frame;
        cv::cvtColor(rgb_mat, bgr_frame, cv::COLOR_RGB2BGR);

        FrameResult result = processFrame(bgr_frame, serial, current_mode);

        // 【避坑】只有当返回的图片尺寸大于 10x10 时才显示，防止宏定义关闭可视化后(1x1)导致显示崩溃
        if (result.annotated.cols > 10 && result.annotated.rows > 10) {
            cv::Mat show_rgb;
            cv::cvtColor(result.annotated, show_rgb, cv::COLOR_BGR2RGB);
            maix::image::Image show_img(show_rgb.cols, show_rgb.rows, maix::image::Format::FMT_RGB888, show_rgb.data);
            disp.show(&show_img);
        }

        delete img;
    }
    return 0;
}