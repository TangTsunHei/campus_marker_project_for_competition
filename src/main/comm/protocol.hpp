#pragma once
#include <stdint.h>
#include <cstddef>

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
// 供电控直接调用的校验函数，确保与视觉端计算逻辑一致
inline uint8_t calculate_checksum(const VisionToControl& data) {
    uint8_t sum = 0;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&data);
    for (size_t i = 0; i < sizeof(VisionToControl) - 1; ++i) {
        sum += p[i];
    }
    return sum;
}