#include "motor.h"

/* 物理换算系数: (轮子周长 / 编码器线数 / 减速比) = 0.152mm/pulse (x1000 = 152) */
static const int32_t PULSE_TO_DIST_MM_X1000 = 152; 

/* 微调辅助: 当目标速度较小时(微调阶段)，强制一个最小 PWM 初值以克服静摩擦 */
// 该阈值应足够小以只在微调时生效 (单位: mm/s)
#define MICRO_ADJUST_SPEED_THRESH  30
// 最小 PWM 初值 (微调阶段防止电机停转)
#define MIN_DRIVE_PWM              50

/* PWM 平滑: 一阶低通滤波系数
 * pwm_smoothed = alpha * pwm_smoothed + (1-alpha) * pwm_out
 * alpha=5/10=0.5: 响应快, 刹车时 PWM 快速衰减
 */
#define PWM_SMOOTH_ALPHA_NUM 5
#define PWM_SMOOTH_ALPHA_DEN 10

/* 状态存储 */
static MotorStatus_t left_data = {0};
static MotorStatus_t right_data = {0};

/* 编码器原始数据记录 */
static int32_t last_left_count = 0;
static int32_t last_right_count = 0;

/* 右电机专用的 16位定时器 (TIM4) 溢出处理与方向补偿 */
volatile int32_t right_total_pulses = 0;
static uint16_t last_tim4_raw = 0;

/* ================== [新增] PID 数据结构与实例 ================== */
typedef struct {
    int32_t target_speed;       // 目标速度 mm/s
    int32_t error_last;         // 上次误差 e(k-1)
    int32_t error_prev;         // 上上次误差 e(k-2)
    int32_t pwm_out;            // 累加计算出的目标 PWM 占空比
    int32_t pwm_smoothed;       // 平滑后的 PWM 输出 (带符号)
    int32_t last_set_target;    // 上一次 Motor_SetTargetSpeed 设置的值 (用于检测目标变化)
} MotorPID_t;

static MotorPID_t pid_left = {0};
static MotorPID_t pid_right = {0};
/* =============================================================== */

/**
 * @brief 底层引脚控制
 */
static void Motor_WriteHardware(MotorID motor, GPIO_PinState in1, GPIO_PinState in2) {
    if (motor == MOTOR_LEFT) {
        HAL_GPIO_WritePin(LIN1_PORT, LIN1_PIN, in1); 
        HAL_GPIO_WritePin(LIN2_PORT, LIN2_PIN, in2); 
    } else {
        HAL_GPIO_WritePin(RIN1_PORT, RIN1_PIN, in1); 
        HAL_GPIO_WritePin(RIN2_PORT, RIN2_PIN, in2); 
    }
}

/**
 * @brief 电机与编码器初始化
 */
void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim3, L_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, R_PWM_CHANNEL);
    HAL_TIM_Encoder_Start(&htim5, TIM_ENCODERMODE_TI12);
    HAL_TIM_Encoder_Start(&htim4, TIM_ENCODERMODE_TI12);
    
    last_tim4_raw = __HAL_TIM_GET_COUNTER(&htim4);
    last_left_count = ((int32_t)__HAL_TIM_GET_COUNTER(&htim5));
    
    Motor_Stop();
}

/* ================== [新增] 增量式 PID 核心计算逻辑 ================== */
/**
 * @brief 计算并输出 PID
 */
static void Motor_PID_Control(MotorID motor, int32_t current_speed) {
    MotorPID_t *pid = (motor == MOTOR_LEFT) ? &pid_left : &pid_right;

    int32_t local_target = pid->target_speed;
    uint8_t is_braking = 0;  // 刹车模式标志

    // 刹车逻辑: 目标为0但仍有速度 → 反向制动 (不进前馈, 避免振荡)
    if (local_target == 0) {
        int32_t abs_speed = current_speed < 0 ? -current_speed : current_speed;
        if (abs_speed > 10) {
            // 反向制动: 反向目标速度 = max(当前速度的60%, 50mm/s)
            int32_t brake_target = (abs_speed * 60) / 100;
            if (brake_target < 50) brake_target = 50;
            local_target = (current_speed > 0) ? -brake_target : brake_target;
            is_braking = 1;
        } else {
            pid->pwm_out = 0;
            pid->error_last = 0;
            pid->error_prev = 0;
            pid->last_set_target = 0;
            pid->pwm_smoothed = 0;
            Motor_SetSpeed(motor, 0);
            return;
        }
    }

    // ---- 前馈 (刹车模式跳过) ----
    if (!is_braking && local_target != pid->last_set_target) {
        pid->last_set_target = local_target;
        int32_t ff_pwm = (local_target * 28) / 10;  // Kff=2.8, 满推350→980PWM
        if (ff_pwm > 0 && ff_pwm < PID_START_PWM) ff_pwm = PID_START_PWM;
        if (ff_pwm < 0 && ff_pwm > -PID_START_PWM) ff_pwm = -PID_START_PWM;
        int32_t pwm_safe_max = (int32_t)(MOTOR_MAX_PWM * 95 / 100);  // 上限95%
        if (ff_pwm > pwm_safe_max) ff_pwm = pwm_safe_max;
        if (ff_pwm < -pwm_safe_max) ff_pwm = -pwm_safe_max;
        pid->pwm_out = ff_pwm;
        pid->pwm_smoothed = ff_pwm;
        pid->error_last = 0;
        pid->error_prev = 0;
        Motor_SetSpeed(motor, (int16_t)pid->pwm_smoothed);
        return;
    }

    // 刹车模式下更新 last_set_target 避免误入前馈
    if (is_braking) {
        pid->last_set_target = local_target;
    }

    // 从静止启动
    if (pid->pwm_out == 0 && local_target != 0) {
        pid->pwm_out = (local_target > 0) ? PID_START_PWM : -PID_START_PWM;
    }

    // PID 计算
    int32_t error = local_target - current_speed;
    int32_t abs_err = error < 0 ? -error : error;
    if (abs_err <= PID_DEADBAND) {
        pid->error_prev = pid->error_last;
        pid->error_last = 0;
        // 不 return, 仍然输出当前 PWM 到硬件
        Motor_SetSpeed(motor, (int16_t)pid->pwm_smoothed);
        return;
    }

    int32_t pwm_inc = (PID_KP * (error - pid->error_last) +
                    PID_KI * error +
                    PID_KD * (error - 2 * pid->error_last + pid->error_prev)) / 100;
    pid->pwm_out += pwm_inc;
    pid->error_prev = pid->error_last;
    pid->error_last = error;

    if (pid->pwm_out > MOTOR_MAX_PWM) pid->pwm_out = MOTOR_MAX_PWM;
    if (pid->pwm_out < -MOTOR_MAX_PWM) pid->pwm_out = -MOTOR_MAX_PWM;

    // 微调阶段最小 PWM
    if (local_target != 0 &&
        local_target <= MICRO_ADJUST_SPEED_THRESH && local_target >= -MICRO_ADJUST_SPEED_THRESH) {
        if (pid->pwm_out > 0 && pid->pwm_out < MIN_DRIVE_PWM) pid->pwm_out = MIN_DRIVE_PWM;
        if (pid->pwm_out < 0 && pid->pwm_out > -MIN_DRIVE_PWM) pid->pwm_out = -MIN_DRIVE_PWM;
    }

    // PWM 平滑
    pid->pwm_smoothed = (pid->pwm_smoothed * PWM_SMOOTH_ALPHA_NUM +
                        pid->pwm_out * (PWM_SMOOTH_ALPHA_DEN - PWM_SMOOTH_ALPHA_NUM)) / PWM_SMOOTH_ALPHA_DEN;
    Motor_SetSpeed(motor, (int16_t)pid->pwm_smoothed);
}
/* ==================================================================== */


/**
 * @brief 更新电机速度与位置 (建议 100ms 调用一次)
 */
void Motor_UpdateStatus(void) {
    // 1. 获取左轮数据
    int32_t curr_left = -((int32_t)__HAL_TIM_GET_COUNTER(&htim5)); 
    
    // 2. 获取右轮数据
    uint16_t curr_tim4_raw = __HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = (int16_t)(curr_tim4_raw - last_tim4_raw); 
    
    right_total_pulses -= (delta); 
    last_tim4_raw = curr_tim4_raw;
    int32_t curr_right = right_total_pulses;

    // 3. 计算速度 (mm/s)
    left_data.speed = (curr_left - last_left_count) * UPDATE_FREQ * PULSE_TO_DIST_MM_X1000 / 1000;
    right_data.speed = (curr_right - last_right_count) * UPDATE_FREQ * PULSE_TO_DIST_MM_X1000 / 1000;

    // 4. 计算总位移 (mm)
    left_data.distance = curr_left * PULSE_TO_DIST_MM_X1000 / 1000;
    right_data.distance = curr_right * PULSE_TO_DIST_MM_X1000 / 1000;

    // 5. 更新历史记录值
    last_left_count = curr_left;
    last_right_count = curr_right;
}

/**
 * @brief 执行 PID 闭环控制 (与 Motor_UpdateStatus 同周期调用)
 *
 * 调用约定:
 *   - Motor_UpdateStatus 先刷新编码器速度 → Motor_RunPID 立即闭环
 *   - 两者在 main 循环中同 50ms 周期执行
 *   - 目标速度变化时通过前馈立即响应, 不等 100ms 反馈
 */
void Motor_RunPID(void) {
    Motor_PID_Control(MOTOR_LEFT, left_data.speed);
    Motor_PID_Control(MOTOR_RIGHT, right_data.speed);
}

/**
 * @brief 设置电机方向
 */
void Motor_SetDirection(MotorID motor, MotorDir dir) {
    if (dir == MOTOR_STOP) {
        Motor_WriteHardware(motor, GPIO_PIN_SET, GPIO_PIN_SET);  // H+H 抱死刹车
    } else if (dir == MOTOR_FORWARD) {
        Motor_WriteHardware(motor, GPIO_PIN_RESET, GPIO_PIN_SET); // 正反转颠倒后
    } else if (dir == MOTOR_BACKWARD) {
        Motor_WriteHardware(motor, GPIO_PIN_SET, GPIO_PIN_RESET); // 正反转颠倒后
    }
}

/**
 * @brief 底层：设置电机占空比 (带限幅)
 * 注意：现在此函数供 PID 内部调用，外部尽量不再直接调用
 */
void Motor_SetSpeed(MotorID motor, int16_t speed) {
    if (speed == 0) {
        Motor_SetDirection(motor, MOTOR_STOP);
        speed = MOTOR_MAX_PWM;  // 抱死刹车: IN1=IN2=HIGH, PWM=100% 实现最大制动力
    } else if (speed > 0) {
        Motor_SetDirection(motor, MOTOR_FORWARD);
    } else {
        Motor_SetDirection(motor, MOTOR_BACKWARD);
        speed = -speed;
    }

    if (speed > MOTOR_MAX_PWM) speed = MOTOR_MAX_PWM;

    if (motor == MOTOR_LEFT)
        __HAL_TIM_SET_COMPARE(&htim3, L_PWM_CHANNEL, (uint32_t)speed);
    else
        __HAL_TIM_SET_COMPARE(&htim3, R_PWM_CHANNEL, (uint32_t)speed);
}

/* ================== [修改] 重构上层控制逻辑 ================== */
/**
 * @brief 设置独立的目标速度 (可用于转向、差速)
 */
void Motor_SetTargetSpeed(int32_t left_speed, int32_t right_speed) {
    pid_left.target_speed = left_speed;
    pid_right.target_speed = right_speed;
    pid_left.last_set_target = left_speed;
    pid_right.last_set_target = right_speed;
}

/**
 * @brief 直线前进 (闭环模式)
 */
void Motor_MoveForward(int32_t speed_mm_s) {
    Motor_SetTargetSpeed(speed_mm_s, speed_mm_s);
}

/**
 * @brief 停止电机 (必须清理 PID 目标)
 */
void Motor_Stop(void) {
    Motor_SetTargetSpeed(0, 0); // 目标归零
    Motor_SetSpeed(MOTOR_LEFT, 0);    // 硬件立即停止
    Motor_SetSpeed(MOTOR_RIGHT, 0);
}
/* ============================================================== */

MotorStatus_t Motor_GetStatus(MotorID motor) {
    return (motor == MOTOR_LEFT) ? left_data : right_data;
}