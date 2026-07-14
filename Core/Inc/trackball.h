#ifndef __TRACKBALL_H
#define __TRACKBALL_H

#include "main.h"

// ============================================================
//  网球追踪速度曲线参数
//
//  核心思路:
//    distance 决定前进基础速度 (越远越快, 越近越慢)
//    X 偏移   决定左右轮差值   (偏右→右转, 偏左→左转)
//
//  画面坐标系: 160x120, 中心 X=80
// ============================================================

// ---- 距离→速度映射 (无距离补偿, 直接使用 OpenMV 距离) ----
// 三段速度曲线:
//   0~16cm → 0 (停车, 速度归零)
//  16~30cm → LOW~LOW  (低速区: 统一低速缓慢逼近, 保证精准停车)
//  30~84cm → MID~MAX  (正常追踪, 线性加速)
//  >84cm   → MAX      (全速追赶)
#define BALL_STOP_DIST      16   // 停车距离 (cm): ≤16cm速度归零
#define BALL_SLOW_DIST      30   // 低速区上界 (cm): 16~30cm全程低速
#define BALL_FAST_DIST      84   // 快速区起点 (cm)

#define BALL_SPEED_LOW      20   // 低速区统一速度 mm/s (16~30cm缓慢逼近)
#define BALL_ALIGN_SPEED    10   // X/距离未对准时微调速度 mm/s (前进/后退/转向)
#define BALL_SPEED_MID      180  // 中距离速度 mm/s
#define BALL_SPEED_MAX      350  // 远距离最高速度 mm/s

// ---- X 偏移→差速映射 (PD 控制) ----
#define BALL_X_GAIN         2    // X 偏移 P 增益 (mm/s per pixel, 降低防摆)
#define BALL_X_KD           3    // X 偏移 D 增益 (mm/s per pixel/frame, 阻尼振荡)
#define BALL_X_DEADBAND     8    // X 死区: |offset|≤8不调节, 防止中心附近微摆
#define BALL_DIFF_MAX       200  // 差速上限, 防止单轮急转

// ---- 抓取就绪判定 ----
// 可抓取距离区间: 14~16cm (太远够不到, 太近碰撞风险)
// 目标微调值: 15cm
#define BALL_GRAB_DIST_MAX  16   // 抓取距离上界 (cm)
#define BALL_GRAB_DIST_MIN  14   // 抓取距离下界 (cm)
#define BALL_GRAB_X_MIN     115  // X 坐标下限 (0~255): 球在105~135区间
#define BALL_GRAB_X_MAX     145  // X 坐标上限
#define BALL_GRAB_READY_MS  400  // 稳定时间 (ms): 加长确认时间防止误触发

// ---- 丢球保护 ----
#define BALL_LOST_FRAMES    15   // 连续丢球帧数, 超过则减速旋转搜寻
#define BALL_SEARCH_SPEED   80   // 丢球后搜寻速度 mm/s

// ---- 输出平滑 ----
#define BALL_SMOOTH_ALPHA   25   // 输出平滑系数 (x100): 越小越平滑, 25=强滤波

// ---- 输出限幅 ----
#define BALL_MAX_SPEED      400  // 单轮最高速度 (降低防急转)
#define BALL_MAX_REVERSE    (-150)

// ---- 微调卡死检测 ----
// 微调阶段 distance 和 ball_x 在 STUCK_TIMEOUT_MS 内无变化 → 判定卡死
#define BALL_STUCK_TIMEOUT_MS   200  // 数据无变化超时 (ms)
#define BALL_BACKOUT_SPEED      200  // 卡死后后退速度 mm/s
#define BALL_BACKOUT_DURATION_MS 500 // 卡死后后退持续时间 (ms)

// ============================================================
//  函数声明
// ============================================================
void TrackBall_Init(void);             // 重置追踪状态
void TrackBall_Process(void);          // 每帧调用, 处理网球追踪
uint8_t TrackBall_IsGrabReady(void);   // 球已就位, 可以抓取
uint8_t TrackBall_IsStuck(void);       // 微调卡死检测 (distance/X 无变化超时)

#endif /* __TRACKBALL_H */
