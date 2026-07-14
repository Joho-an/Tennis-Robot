#include "auto.h"
#include "motor.h"
#include "servo.h"
#include "pid.h"
#include "trackball.h"
#include "battery.h"
#include "Openmv.h"

// ============================================================
//  状态机内部状态
// ============================================================
static AutoState_t state = AUTO_STATE_LINE_FOLLOW;
static uint8_t ball_count  = 0;
static uint8_t enter_state = 1;  // 首次进入当前状态标志

// ---- 球检测去抖动 ----
static uint8_t  ball_seen_frames = 0;

// ---- 抓取前停车确认 ----
static uint8_t  grab_stop_confirm = 0;  // 1=正在等待电机实际停止
static uint32_t grab_stop_tick    = 0;

// ---- 自转找线 ----
static uint32_t spin_start_tick = 0;
static uint8_t  spin_line_frames = 0;

// ---- 球追踪丢球保护 ----
static uint32_t ball_last_seen_tick = 0;  // 最后一次看到球的时间戳

// ---- 微调卡死后退 ----
static uint8_t  backout_active = 0;       // 1=正在执行后退
static uint32_t backout_start_tick = 0;   // 后退开始时间戳

// ============================================================
//  内部辅助函数
// ============================================================

/**
 * @brief 检查停机条件 (电量过低 / 球数达标)
 */
static uint8_t check_stop_conditions(void) {
    if (ball_count >= BALL_COUNT_MAX) {
        return 1;
    }
    return 0;
}

/**
 * @brief 检测 OpenMV 是否发现了网球 (只在有新数据时判定, 防止残留值误判)
 */
static uint8_t ball_seen(void) {
    // 无新数据时直接跳过, 不计数也不清零
    if (g_openmv_data.is_updated == 0) {
        return 0;
    }
    if (g_openmv_data.distance > 0) {
        ball_seen_frames++;
        if (ball_seen_frames >= BALL_SEEN_FRAMES) {
            ball_seen_frames = 0;
            return 1;
        }
    } else {
        ball_seen_frames = 0;  // distance==0 才清零
    }
    return 0;
}

// ============================================================
//  各状态处理函数
// ============================================================

static void run_line_follow(void) {
    if (enter_state) {
        Track_Line_Reset();
        Motor_Stop();
        enter_state = 0;
    }

    // 先检测球 (必须在 Track_Line_Process 清零 is_updated 之前)
    if (ball_seen()) {
        state = AUTO_STATE_BALL_TRACK;
        enter_state = 1;
        return;
    }

    Track_Line_Process();

    // 丢线超时保护: 超过 LINE_LOST_TIMEOUT_MS 没看到线 → 减速 → 停车
    if (Track_Line_GetLostMs() > LINE_LOST_TIMEOUT_MS) {
        Motor_SetTargetSpeed(LOST_SPEED, LOST_SPEED);
    }
}

static void run_ball_track(void) {
    if (enter_state) {
        TrackBall_Init();
        grab_stop_confirm = 0;
        ball_last_seen_tick = HAL_GetTick();
        backout_active = 0;
        enter_state = 0;
    }

    // ---- 卡死后退中: 以 200mm/s 倒车, 持续 BALL_BACKOUT_DURATION_MS ----
    if (backout_active) {
        Motor_SetTargetSpeed(-BALL_BACKOUT_SPEED, -BALL_BACKOUT_SPEED);
        if (HAL_GetTick() - backout_start_tick >= BALL_BACKOUT_DURATION_MS) {
            Motor_Stop();
            OpenMV_ClearFlag();          // 清除残留数据
            backout_active = 0;
            state = AUTO_STATE_LINE_FOLLOW;
            enter_state = 1;
        }
        return;
    }

    TrackBall_Process();  // 内部会消费 is_updated

    // ---- 微调卡死检测: distance/X 在 BALL_STUCK_TIMEOUT_MS 内无变化 → 后退退出 ----
    if (TrackBall_IsStuck()) {
        OpenMV_ClearFlag();                    // 清除小球数据和状态标志
        backout_active = 1;
        backout_start_tick = HAL_GetTick();
        Motor_SetTargetSpeed(-BALL_BACKOUT_SPEED, -BALL_BACKOUT_SPEED);
        return;
    }

    // 更新最后看到球的时间: 在 TrackBall_Process 消费 is_updated 之前检测
    // 改为用 g_openmv_data.distance 判断 (is_updated 已被清零)
    if (g_openmv_data.distance > 0) {
        ball_last_seen_tick = HAL_GetTick();
    }

    // 丢球超时保护: 超过 BALL_TRACK_TIMEOUT_MS 没看到球 → 退回 LINE_FOLLOW
    if (HAL_GetTick() - ball_last_seen_tick > BALL_TRACK_TIMEOUT_MS) {
        Motor_Stop();
        OpenMV_ClearFlag();
        state = AUTO_STATE_LINE_FOLLOW;
        enter_state = 1;
        return;
    }

    // 球已就位 → 先停车, 等实际速度归零再触发抓取
    if (TrackBall_IsGrabReady()) {
        if (!grab_stop_confirm) {
            // 第一次进入: 目标速度归零, 刹车
            Motor_Stop();
            grab_stop_confirm = 1;
            grab_stop_tick = HAL_GetTick();
        }
        // 等待实际速度归零 (左轮&右轮速度都 < 5mm/s 视为停止)
        MotorStatus_t L = Motor_GetStatus(MOTOR_LEFT);
        MotorStatus_t R = Motor_GetStatus(MOTOR_RIGHT);
        int32_t abs_Lv = L.speed < 0 ? -L.speed : L.speed;
        int32_t abs_Rv = R.speed < 0 ? -R.speed : R.speed;
        if ((abs_Lv < 5 && abs_Rv < 5) ||
            HAL_GetTick() - grab_stop_tick >= 500) {
            OpenMV_ClearFlag();
            Servo_StartGrabCycle(g_openmv_data.distance);
            grab_stop_confirm = 0;
            state = AUTO_STATE_GRAB;
            enter_state = 1;
        }
    } else {
        // 球脱离抓取区域, 重置停车确认状态, 继续追踪对准
        grab_stop_confirm = 0;
    }
}

static void run_grab(void) {
    if (enter_state) {
        // 确保电机完全停止，PID 目标归零
        Motor_Stop();
        OpenMV_ClearFlag();          // 抓取期间不关心视觉数据
        enter_state = 0;
    }

    // 抓取期间持续清除 OpenMV 数据，防止残留数据被后续状态误读
    if (g_openmv_data.is_updated) {
        OpenMV_ClearFlag();
    }

    // 等待舵机抓取完成
    if (Servo_IsGrabDone()) {
        ball_count++;
        state = AUTO_STATE_WAIT_LINE;
        enter_state = 1;
    }
}

static void run_wait_line(void) {
    if (enter_state) {
        Motor_Stop();
        OpenMV_ClearFlag();          // 清除抓取期间残留数据
        spin_start_tick = HAL_GetTick();
        spin_line_frames = 0;
        enter_state = 0;
    }

    // 超时保护: 超时后开始自转找线
    if (HAL_GetTick() - spin_start_tick > WAIT_LINE_TIMEOUT_MS) {
        spin_start_tick = HAL_GetTick();
        spin_line_frames = 0;
        state = AUTO_STATE_SPIN_SEARCH;
        enter_state = 1;
        return;
    }

    // 检测黑色色带: 有 offset 数据 → 可能在线上
    if (g_openmv_data.is_updated) {
        int8_t offset = g_openmv_data.offset;
        int8_t abs_off = offset < 0 ? -offset : offset;
        if (abs_off > SPIN_LINE_THRESH) {
            spin_line_frames++;
            if (spin_line_frames >= WAIT_LINE_FRAMES) {
                state = AUTO_STATE_LINE_FOLLOW;
                enter_state = 1;
                return;
            }
        } else {
            spin_line_frames = 0;
        }
        g_openmv_data.is_updated = 0;
    }
}

static void run_spin_search(void) {
    if (enter_state) {
        OpenMV_ClearFlag();                        // 清除 GRAB 期间累积的旧数据
        Motor_SetTargetSpeed(SPIN_SPEED, -SPIN_SPEED); // 原地右转 (顺时针)=
        enter_state = 0;
    }

    // 超时保护
    if (HAL_GetTick() - spin_start_tick > SPIN_TIMEOUT_MS) {
        Motor_Stop();
        state = AUTO_STATE_STOP;
        enter_state = 1;
        return;
    }

    // 检测白色色带是否出现
    if (g_openmv_data.is_updated) {
        int8_t offset = g_openmv_data.offset;
        int8_t abs_off = offset < 0 ? -offset : offset;
        if (abs_off > SPIN_LINE_THRESH) {
            spin_line_frames++;
            if (spin_line_frames >= SPIN_LINE_FRAMES) {
                Motor_Stop();
                state = AUTO_STATE_LINE_FOLLOW;
                enter_state = 1;
                return;
            }
        } else {
            spin_line_frames = 0;
        }
        g_openmv_data.is_updated = 0;
    }
}

static void run_stop(void) {
    if (enter_state) {
        Motor_Stop();
        // 如果因球满停机, 机械臂恢复初始姿态
        if (ball_count >= BALL_COUNT_MAX) {
            Servo_MoveTo(90, ARM_INIT_ANGLE, CLAW_OPEN_ANGLE);
        }
        enter_state = 0;
    }
}

// ============================================================
//  公开函数
// ============================================================

/**
 * @brief 自动模式主循环 (在 main.c 的自动模式分支中每帧调用)
 */
void AutoMode_Run(void) {
    // 停机条件优先
    if (check_stop_conditions()) {
        state = AUTO_STATE_STOP;
        enter_state = 1;
    }

    switch (state) {
        case AUTO_STATE_LINE_FOLLOW: run_line_follow();  break;
        case AUTO_STATE_BALL_TRACK:  run_ball_track();   break;
        case AUTO_STATE_GRAB:        run_grab();          break;
        case AUTO_STATE_WAIT_LINE:   run_wait_line();     break;
        case AUTO_STATE_SPIN_SEARCH: run_spin_search();   break;
        case AUTO_STATE_STOP:        run_stop();          break;
        default:
            state = AUTO_STATE_LINE_FOLLOW;
            enter_state = 1;
            break;
    }
}

/**
 * @brief 模式切换时重置所有状态
 */
void AutoMode_Reset(void) {
    state       = AUTO_STATE_LINE_FOLLOW;
    enter_state = 1;
    Track_Line_Reset();
    TrackBall_Init();
    ball_seen_frames = 0;
    spin_line_frames = 0;
    // Ensure motors are stopped and vision buffer cleared when entering AUTO
    Motor_Stop();
    OpenMV_ClearFlag();
}


uint8_t AutoMode_GetBallCount(void) {
    return ball_count;
}
