#include "trackball.h"
#include "motor.h"
#include "Openmv.h"

// ============================================================
//  网球追踪状态
// ============================================================
static uint16_t lost_frames    = 0;   // 连续丢球帧数
static uint32_t ready_tick     = 0;   // 进入抓取区域的时间戳
static uint8_t  was_ready      = 0;   // 上一帧是否在抓取区域
static uint8_t  first_run      = 1;
static int8_t   last_x_error   = 0;   // PD 控制: 上一帧 X 误差

// ---- 微调卡死检测 ----
static uint8_t  stuck_flag     = 0;   // 1=已触发卡死
static uint8_t  last_stuck_x   = 0;   // 上次记录的 ball_x
static uint8_t  last_stuck_dist = 0;  // 上次记录的 distance
static uint32_t stuck_tick     = 0;   // 数据无变化开始计时

// ============================================================
//  内部辅助函数
// ============================================================

/**
 * @brief 距离 → 基础速度 (三段线性映射, 无距离补偿)
 *
 * 入参 distance 为 OpenMV 原始距离, 直接查表:
 *
 * distance ≤ STOP_DIST   → 0       (停车)
 * STOP_DIST < dist ≤ LOW → LOW     (低速区: 统一低速缓慢逼近)
 * LOW       < dist ≤ FAST → MID~MAX (正常追踪, 线性加速)
 * distance > FAST_DIST   → MAX     (全速追赶)
 */
static int16_t calc_ball_base_speed(uint8_t distance) {
    if (distance <= BALL_STOP_DIST) {
        return 0;
    }
    // 低速区: 16~30cm 统一低速 BALL_SPEED_LOW, 确保缓慢精准逼近
    if (distance <= BALL_SLOW_DIST) {
        return BALL_SPEED_LOW;
    }
    if (distance <= BALL_FAST_DIST) {
        int32_t range = BALL_FAST_DIST - BALL_SLOW_DIST;
        int32_t inc   = (BALL_SPEED_MAX - BALL_SPEED_MID) *
                        (distance - BALL_SLOW_DIST) / range;
        return (int16_t)(BALL_SPEED_MID + inc);
    }
    return BALL_SPEED_MAX;
}

// ============================================================
//  公开函数
// ============================================================

/**
 * @brief 重置追踪状态 (状态机进入时调用)
 */
void TrackBall_Init(void) {
    lost_frames     = 0;
    ready_tick      = 0;
    was_ready       = 0;
    first_run       = 1;
    last_x_error    = 0;
    stuck_flag      = 0;
    last_stuck_x    = 0;
    last_stuck_dist = 0;
    stuck_tick      = 0;
}

/**
 * @brief 网球追踪主处理 (每帧调用)
 *
 * 根据 OpenMV 发来的网球 X 坐标和距离,
 * 计算左右轮目标速度并输出到电机内环 PID.
 */
void TrackBall_Process(void) {
    // ---- 丢球检测 ----
    // 连续无 OpenMV 数据超时保护 (防止 UART 断连后电机卡死)
    {
        static uint32_t no_data_start = 0;
        if (g_openmv_data.is_updated == 0) {
            if (no_data_start == 0) no_data_start = HAL_GetTick();
            if (HAL_GetTick() - no_data_start > 3000) {
                Motor_SetTargetSpeed(0, 0);  // 3秒无数据 → 强制停车
            }
            return;
        }
        no_data_start = 0;  // 有数据, 重置计时器
    }

    uint8_t ball_found = (g_openmv_data.distance > 0);
    g_openmv_data.is_updated = 0;

    if (!ball_found) {
        lost_frames++;
        if (lost_frames >= BALL_LOST_FRAMES) {
            Motor_SetTargetSpeed(-BALL_SEARCH_SPEED, BALL_SEARCH_SPEED);
        } else {
            // 丢球帧数不够时, 保持上帧微调速度 (防止 distance=0 时急停)
            // 不做任何更改, was_ready 清零即可
        }
        was_ready = 0;
        return;
    }

    lost_frames = 0;

    // ---- 有球: PD 控制计算追踪速度 ----
    uint8_t ball_x = g_openmv_data.ball_x;
    int8_t  x_error  = (int8_t)ball_x - 140;  // 将微调目标改为 X=140
    uint8_t distance = g_openmv_data.distance;

    int16_t base = calc_ball_base_speed(distance);

    // 如果球 X 在抓取接受区间 125~155，则认为 X 已对准，停止横向微调
    if (ball_x >= BALL_GRAB_X_MIN && ball_x <= BALL_GRAB_X_MAX) {
        x_error = 0;
    }

    // 防死锁 + 精细对准: 距离≤16cm 且 (X未对准 或 距离过近)
    // 策略:
    //   距离 <13  → 后退(10mm/s) + 转向, 回到13~16区间
    //   距离13~16 + X对准 → base=0 停车抓取(正常流程)
    //   距离13~16 + X未对准 → 原地转向(10mm/s base + diff)
    //   距离 >16  → 正常追踪(走 calc_ball_base_speed)
    if (distance <= BALL_STOP_DIST) {
        uint8_t in_grab_x = (g_openmv_data.ball_x >= BALL_GRAB_X_MIN) &&
                            (g_openmv_data.ball_x <= BALL_GRAB_X_MAX);
        uint8_t in_grab_dist = (distance >= BALL_GRAB_DIST_MIN) &&
                               (distance <= BALL_GRAB_DIST_MAX);

        if (!in_grab_x || !in_grab_dist) {
            // 未完全对准, 需要微调
            // 微调目标: distance=15cm；当距离小于目标时后退，大于目标时前进
            base = BALL_ALIGN_SPEED;  // 10mm/s 基础

            // distance < 15: 太近, 后退微调
            if (distance < 15) {
                base = -BALL_ALIGN_SPEED;  // 负值 = 后退
            }
            // X 不在抓取区间则差速转向, 抓取区间内已在上面清零 x_error

            // ---- 微调卡死检测: distance 和 ball_x 在 BALL_STUCK_TIMEOUT_MS 内无变化 ----
            if (distance == last_stuck_dist && ball_x == last_stuck_x) {
                if (stuck_tick == 0) {
                    stuck_tick = HAL_GetTick();
                } else if (HAL_GetTick() - stuck_tick >= BALL_STUCK_TIMEOUT_MS) {
                    stuck_flag = 1;
                    stuck_tick = 0;
                }
            } else {
                last_stuck_dist = distance;
                last_stuck_x    = ball_x;
                stuck_tick      = 0;
            }
        } else {
            // X对准 + 距离对准 → 彻底停车, 禁止方向修正, 静等抓取触发
            base   = 0;
            x_error = 0;
            last_x_error = 0;
        }
    }

    // PD 控制: diff = Kp * error + Kd * (error - last_error)
    int16_t diff = (int16_t)(x_error * BALL_X_GAIN + (x_error - last_x_error) * BALL_X_KD);
    last_x_error = x_error;

    // 差速限幅: 防止单轮急转导致摆头
    if (diff > BALL_DIFF_MAX)   diff = BALL_DIFF_MAX;
    if (diff < -BALL_DIFF_MAX)  diff = -BALL_DIFF_MAX;

    int16_t target_left  = base + diff;
    int16_t target_right = base - diff;

    // 限幅
    if (target_left  > BALL_MAX_SPEED)   target_left  = BALL_MAX_SPEED;
    if (target_left  < BALL_MAX_REVERSE) target_left  = BALL_MAX_REVERSE;
    if (target_right > BALL_MAX_SPEED)   target_right = BALL_MAX_SPEED;
    if (target_right < BALL_MAX_REVERSE) target_right = BALL_MAX_REVERSE;

    // 输出平滑 (一阶低通, 强滤波避免单帧噪点导致剧烈摆头)
    {
        static int16_t smooth_left = 0, smooth_right = 0;
        if (first_run) {
            smooth_left  = target_left;
            smooth_right = target_right;
            first_run = 0;
        } else {
            smooth_left  = (int16_t)((BALL_SMOOTH_ALPHA * (int32_t)target_left  +
                                      (100 - BALL_SMOOTH_ALPHA) * (int32_t)smooth_left) / 100);
            smooth_right = (int16_t)((BALL_SMOOTH_ALPHA * (int32_t)target_right +
                                      (100 - BALL_SMOOTH_ALPHA) * (int32_t)smooth_right) / 100);
        }
        target_left  = smooth_left;
        target_right = smooth_right;
    }

    Motor_SetTargetSpeed(target_left, target_right);

    // ---- 抓取就绪判定 ----
    // 使用原始 distance (不加补偿), 可抓取区间: 14~16cm
    uint8_t in_zone = (distance >= BALL_GRAB_DIST_MIN) &&
                      (distance <= BALL_GRAB_DIST_MAX) &&
                      (g_openmv_data.ball_x >= BALL_GRAB_X_MIN) &&
                      (g_openmv_data.ball_x <= BALL_GRAB_X_MAX);

    if (in_zone) {
        if (!was_ready) {
            ready_tick = HAL_GetTick();
        }
        was_ready = 1;
    } else {
        was_ready = 0;
    }
}

uint8_t TrackBall_IsGrabReady(void) {
    if (!was_ready) return 0;
    return (HAL_GetTick() - ready_tick >= BALL_GRAB_READY_MS) ? 1 : 0;
}

uint8_t TrackBall_IsStuck(void) {
    return stuck_flag;
}
