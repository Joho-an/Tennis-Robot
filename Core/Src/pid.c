#include "pid.h"
#include "motor.h"
#include "Openmv.h"

// 循迹 PID 参数 (x100 缩放)
// 降低Kp减少过冲, 增强Kd阻尼振荡, 降低基础速度
int32_t Track_Kp_100 = 200;
int32_t Track_Ki_100 = 0;
int32_t Track_Kd_100 = 50;

int16_t Base_Speed = 250;

// PID 内部状态
static int32_t last_error    = 0;
static int32_t integral      = 0;
static int32_t filtered_d    = 0;
static int16_t smooth_left   = 0;
static int16_t smooth_right  = 0;
static uint8_t first_run     = 1;
static uint32_t last_line_tick = 0;   // 最后一次看到线的时间戳

// ============================================================
//  内部辅助函数
// ============================================================

/**
 * @brief 分段 P 增益: 微偏柔和修正, 大偏激进回正
 *
 * offset 范围 -80~80 (OpenMV QQVGA 160x120)
 * |offset| ≤ 15: 直道微调 → 缩小 Kp 防过冲
 * |offset| > 15: 弯道偏离 → 完整 Kp 快速回正
 */
static inline int32_t apply_kp(int32_t error) {
    int32_t abs_err = error < 0 ? -error : error;
    int32_t kp = Track_Kp_100;
    if (abs_err <= KP_SPLIT_THRESH) {
        kp = (Track_Kp_100 * KP_SMALL_SCALE) / 100;
    }
    return (kp * error) / 100;
}

/**
 * @brief 弯道自适应基础速度
 *
 * |offset| ≤ DEADBAND   → Base_Speed (全速直行)
 * |offset| ≥ THRESH     → CURVE_MIN_SPEED (急弯最低速)
 * 中间线性过渡
 */
static int16_t calc_curve_speed(int32_t abs_err) {
    if (abs_err <= TRACK_DEADBAND) {
        return Base_Speed;
    }
    if (abs_err >= CURVE_SLOW_THRESH) {
        return CURVE_MIN_SPEED;
    }
    int32_t range = CURVE_SLOW_THRESH - TRACK_DEADBAND;
    int32_t drop  = (Base_Speed - CURVE_MIN_SPEED) *
                    (abs_err - TRACK_DEADBAND) / range;
    return (int16_t)(Base_Speed - drop);
}

// ============================================================
//  循迹 PID (外环: offset → 左右轮目标速度)
//
//  串级控制链路:
//    OpenMV 30fps → Track_Line_Process → Motor_SetTargetSpeed
//                  → Motor_PID_Control @10Hz → PWM → 电机
// ============================================================

void Track_Line_Process(void) {
    // ---- 丢线检测 (基于真实时间) + 超时强制停车 ----
    {
        static uint32_t no_data_start = 0;
        if (g_openmv_data.is_updated == 0) {
            if (no_data_start == 0) no_data_start = HAL_GetTick();
            if (HAL_GetTick() - no_data_start > 3000) {
                Motor_SetTargetSpeed(0, 0);  // 3秒无数据 → 强制停车
            }
            return;
        }
        no_data_start = 0;  // 有数据, 重置
    }

    last_line_tick = HAL_GetTick();  // 有数据时更新丢线时间戳
    int32_t error = g_openmv_data.offset;
    g_openmv_data.is_updated = 0;

    int32_t abs_err = error < 0 ? -error : error;

    // ---- 死区: 微小偏移不调节 ----
    if (abs_err <= TRACK_DEADBAND) {
        error    = 0;
        integral = 0;
    }

    // ---- P 项 (分段增益) ----
    int32_t p_out = apply_kp(error);

    // ---- I 项 (条件积分, 抗饱和) ----
    if (Track_Ki_100 != 0 && abs_err > TRACK_DEADBAND) {
        integral += error;
        if (integral > 300) integral = 300;
        if (integral < -300) integral = -300;
    } else {
        integral = 0;
    }
    int32_t i_out = (Track_Ki_100 * integral) / 100;

    // ---- D 项 (一阶低通滤波) ----
    int32_t raw_d = error - last_error;
    last_error = error;
    filtered_d = (D_FILTER_ALPHA * raw_d +
                  (100 - D_FILTER_ALPHA) * filtered_d) / 100;
    int32_t d_out = (Track_Kd_100 * filtered_d) / 100;

    // ---- PID 总输出 ----
    int32_t pid_out = p_out + i_out + d_out;

    // ---- 弯道自适应速度 ----
    int16_t curve_speed = calc_curve_speed(abs_err);

    // ---- 左右轮速度分配 ----
    // offset > 0 (线偏右) → pid_out > 0 → 左轮加速, 右轮减速 → 车体右转
    int16_t target_left  = curve_speed + (int16_t)pid_out;
    int16_t target_right = curve_speed - (int16_t)pid_out;

    // ---- 输出限幅 ----
    if (target_left  > TRACK_MAX_SPEED)    target_left  = TRACK_MAX_SPEED;
    if (target_left  < TRACK_MAX_REVERSE)  target_left  = TRACK_MAX_REVERSE;
    if (target_right > TRACK_MAX_SPEED)    target_right = TRACK_MAX_SPEED;
    if (target_right < TRACK_MAX_REVERSE)  target_right = TRACK_MAX_REVERSE;

    // ---- 输出平滑 (避免目标速度跳变冲击电机内环 PID) ----
    if (first_run) {
        smooth_left  = target_left;
        smooth_right = target_right;
        first_run = 0;
    } else {
        smooth_left  = (int16_t)((OUT_SMOOTH_ALPHA * (int32_t)target_left +
                                  (100 - OUT_SMOOTH_ALPHA) * (int32_t)smooth_left) / 100);
        smooth_right = (int16_t)((OUT_SMOOTH_ALPHA * (int32_t)target_right +
                                  (100 - OUT_SMOOTH_ALPHA) * (int32_t)smooth_right) / 100);
    }

    Motor_SetTargetSpeed(smooth_left, smooth_right);
}

/**
 * @brief 重置 PID 内部状态 (模式切换或停车后调用)
 */
void Track_Line_Reset(void) {
    last_error   = 0;
    integral     = 0;
    filtered_d   = 0;
    smooth_left  = 0;
    smooth_right = 0;
    first_run    = 1;
    last_line_tick = HAL_GetTick();
}

/**
 * @brief 获取丢线持续时间 (ms)
 * @return 当前时刻距离最后一次看到线的毫秒数
 */
uint32_t Track_Line_GetLostMs(void) {
    return HAL_GetTick() - last_line_tick;
}
