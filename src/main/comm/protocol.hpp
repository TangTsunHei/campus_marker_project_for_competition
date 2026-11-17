#pragma once
#include <stdint.h>
#include <cstddef>

#pragma pack(1)
struct VisionToControl {
    uint8_t  header[2];    // 0xAA, 0x55
    uint8_t  is_detected;  // 1=识别到, 0=未识别
    uint8_t  target_type;  // 【新增】0=无，1=视觉兑换框，2=退出AprilTag
    
    //以下数据均以相机光心为原点，单位为毫米(mm)
    uint32_t frame_id;     // 帧号，电控可借此计算丢包率和延迟
    float    x_err;        // mm 目标在画面中的左右偏移量 ；方向：画面右侧是正方向（+X），画面左侧是负方向（-X）
    float    y_err;        // mm 目标在画面中的上下偏移量 ；方向：画面下方是正方向（+Y），画面上方是负方向（-Y）
    float    z_dist;       // mm 目标距离相机正前方的距离（深度）；方向：镜头正前方是正方向（+Z）
    float    yaw;          // 度(右手坐标系绕y轴旋转，顺时针为正,用于控制小车转向角度，0度表示小车正对着目标)
    float    v_x;          // 目标横向速度 (mm/s)(相对速度)，主要用于追踪运动靶
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

// =====================================================================
// 【新增】电控发给视觉的控制指令（极简协议）
// =====================================================================
#pragma pack(1)
struct ControlToVision {
    uint8_t header[2];     // 0xBB, 0x66 (与视觉发的帧头区分开)
    uint8_t mode;          // 1=寻找兑换框 (默认)， 2=寻找退出Tag， 0=待机
    uint8_t checksum;      // 前3个字节的累加和
};
#pragma pack()

inline uint8_t calculate_ctrl_checksum(const ControlToVision& data) {
    uint8_t sum = 0;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&data);
    for (size_t i = 0; i < sizeof(ControlToVision) - 1; ++i) {
        sum += p[i];
    }
    return sum;
}