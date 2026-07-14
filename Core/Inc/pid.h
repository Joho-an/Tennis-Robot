#ifndef __PID_H
#define __PID_H

#include "main.h"

// ============================================================
//  循迹 PID (外环: 偏移量 → 左右轮目标速度)
//
//  OpenMV ROI: y=80~120, 近场 40px
//  offset 范围: -80 ~ +80 像素
//
//  调参: 直线看Kp小误差是否摆 → 弯道看Kd是否过冲 → 最后调Ki
// ============================================================

// ---- PID 增益 (x100) ----
extern int32_t Track_Kp_100;      // 初始 200
extern int32_t Track_Ki_100;      // 初始 0
extern int32_t Track_Kd_100;      // 初始 10
extern int16_t Base_Speed;        // 直线基础速度, 初始 160

// ---- 死区与弯道降速 ----
#define TRACK_DEADBAND      5     // 死区: |offset|≤5直行不调节
#define CURVE_SLOW_THRESH   20    // |offset|≥20开始降速, 稍晚介入避免直线误减速
#define CURVE_MIN_SPEED     100    // 急弯最低速度

// ---- 分段 P 增益: 小误差柔和/大误差激进 ----
#define KP_SPLIT_THRESH     12    // 分界点 (像素)
#define KP_SMALL_SCALE      100   // 小误差比例系数

// ---- 微分滤波 (值越小滤波越强, 但响应越滞后) ----
#define D_FILTER_ALPHA      30    // 加强滤波, 减少D项高频噪声

// ---- 输出平滑 (值越小越丝滑, 但响应越滞后) ----
#define OUT_SMOOTH_ALPHA    60    // 输出平滑系数

// ---- 丢线保护 ----
#define LINE_LOST_TIMEOUT_MS 2000  // 丢线超过2秒 → 减速到 LOST_SPEED
#define LOST_SPEED          60    // 丢线后低速搜索速度

// ---- 输出限幅 ----
#define TRACK_MAX_SPEED     350
#define TRACK_MAX_REVERSE   (-100)

void Track_Line_Process(void);
void Track_Line_Reset(void);
uint32_t Track_Line_GetLostMs(void);  // 获取丢线持续时间 (ms)

#endif /* __PID_H */
