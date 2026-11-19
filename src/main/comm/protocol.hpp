#pragma once
#include <stdint.h>
#include <cstddef>

#pragma pack(1)
struct VisionToControl {
    uint8_t  header[2];    // 0xAA, 0x55
    uint8_t  is_detected;  // 1=识别到, 0=未识别
    uint8_t  target_type;  // 【新增】0=无，1=视觉兑换框，2=退出AprilTag
    
    //以下数据均以相机光心为原点，单位为毫米(mm)
   //现在这些数据已经通过坐标系转换，转换为了以升降投放装置未升起时的几何中心为原心的数据了
   //复杂数据的说明：
   //yaw:目标板（视觉兑换靶）相对于相机光轴（战车正前方）的偏转角。它代表了战车车头与目标板法线之间的夹角
   //正数（+）：顺时针偏转。在画面中，表现为目标板左侧离你更近，右侧更远（或者车头向右偏了）。
   //负数（-）：逆时针偏转。在画面中，表现为目标板右侧离你更近，左侧更远（或者车头向左偏了）
   //v_x:正数（+）：目标在战车视野中向右移动。
   //负数（-）：目标在战车视野中向左移动。
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