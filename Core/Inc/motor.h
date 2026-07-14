#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32h7xx_hal.h"

/* --- 电机硬件引脚配置 --- */
#define LIN1_PIN        GPIO_PIN_10
#define LIN1_PORT       GPIOA
#define LIN2_PIN        GPIO_PIN_9
#define LIN2_PORT       GPIOA

#define RIN1_PIN        GPIO_PIN_3
#define RIN1_PORT       GPIOD
#define RIN2_PIN        GPIO_PIN_4
#define RIN2_PORT       GPIOD

/* --- PWM与编码器接口 --- */
extern TIM_HandleTypeDef htim3; // PWM 输出 (CH1:右, CH2:左)
extern TIM_HandleTypeDef htim5; // 左编码器 (32位)
extern TIM_HandleTypeDef htim4; // 右编码器 (16位)

#define L_PWM_CHANNEL    TIM_CHANNEL_2
#define R_PWM_CHANNEL    TIM_CHANNEL_1

/* --- 控制参数 --- */
#define MOTOR_MAX_PWM    1000   // 最大占空比限制
#define UPDATE_FREQ      20     // 状态更新频率 (50ms一次则为20Hz)

/* ================== [优化] 速度 PID 参数 ================== */
#define PID_SCALE        100    // 缩放倍数，替代原代码中硬编码的 100
#define PID_KP           80     // 比例系数
#define PID_KI           7      // 积分系数
#define PID_KD           1      // 微分系数

#define PID_DEADBAND     5      // 死区(mm/s): 误差小于此值时不调节PWM，防止静止或匀速时电机抖动
#define PID_START_PWM    100    // 静止启动最小PWM: 从0启动时直接跳到此值
/* ========================================================== */

/* --- 极性配置 (防正反馈暴走) --- */
// 提示：因为电机方向被修正，编码器极性可能也需要对应乘 -1。
// 若烧录后电机疯转不停，请将这里的 -1 改为 1！
#define ENC_POLARITY_L   (-1)   
#define ENC_POLARITY_R   (-1)   

/* --- 类型定义 --- */
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} MotorID;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} MotorDir;

typedef struct {
    volatile int32_t speed;        // 当前速度 (mm/s)
    volatile int32_t distance;     // 累加距离 (mm)
    volatile int32_t total_pulses; // 累加总脉冲
} MotorStatus_t;

/* --- 函数接口 --- */
void Motor_Init(void);
void Motor_UpdateStatus(void);
MotorStatus_t Motor_GetStatus(MotorID motor);

void Motor_SetSpeed(MotorID motor, int16_t speed);
void Motor_SetDirection(MotorID motor, MotorDir dir);

void Motor_SetTargetSpeed(int32_t left_speed, int32_t right_speed);
void Motor_RunPID(void);               // PID 闭环 (与 Motor_UpdateStatus 同周期调用)
void Motor_MoveForward(int32_t speed_mm_s);
void Motor_Stop(void);

#endif /* __MOTOR_H__ */