#pragma once
#include <stdint.h>

#pragma pack(1)
struct VisionToControl {
    uint8_t  header[2];    // 0xAA, 0x55
    uint8_t  is_detected;  // 1=识别到, 0=未识别
    //以下数据均以相机光心为原点，单位为毫米(mm)
    float    x_err;        // mm 目标板在画面中的左右偏移量 ；方向：画面右侧是正方向（+X），画面左侧是负方向（-X）
    float    y_err;        // mm 目标板在画面中的上下偏移量 ；方向：画面下方是正方向（+Y），画面上方是负方向（-Y）
    float    z_dist;       // mm 目标板距离相机正前方的距离（深度）；方向：镜头正前方是正方向（+Z）
    float    yaw;          // 度(右手坐标系绕Z轴旋转，顺时针为正)
    uint8_t  checksum;
};
#pragma pack()