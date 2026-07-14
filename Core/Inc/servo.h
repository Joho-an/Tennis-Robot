#ifndef __SERVO_H
#define __SERVO_H

#include "stm32h7xx_hal.h"

/* --- 外部引用的 I2C4 句柄 --- */
extern I2C_HandleTypeDef hi2c4;

/* --- I2C 舵机驱动板 硬件地址与寄存器  --- */
#define SERVO_I2C_ADDR          0x80  // 0x40 << 1
#define SERVO_REG_MODE1         0x00
#define SERVO_REG_MODE2         0x01
#define SERVO_REG_PRESCALE      0xFE
#define SERVO_REG_LED0_ON_L     0x06

/* --- 舵机初始化状态标志 (供 main.c/LCD 显示) --- */
extern uint8_t servo_init_ok;

/* --- 外部可访问的目标角度变量 --- */
extern int16_t base_target;
extern int16_t arm_target;
extern int16_t claw_target;

/* --- 外部可访问的当前角度变量（LCD可监控） --- */
extern int16_t base_angle;
extern int16_t arm_angle;
extern int16_t claw_angle;

/* --- 抓取过程状态定义 --- */
typedef enum {
    GRAB_IDLE = 0,      // 空闲
    GRAB_REACH,         // 前伸 + 等待1s夹爪闭合
    GRAB_BACK,          // 收回 + 等待1s夹爪张开
    GRAB_RESET          // 回到初始角度
} GrabState_t;

/* --- 外部可调参数（调整角度提高抓取精度） --- */
#define CLAW_GRAB_ANGLE    165 // DG3 闭合角度（抓取）
#define CLAW_RELEASE_ANGLE 125  // DG3 张开角度

#define REACH_BASE_TARGET  5    // 前伸时的底座角度
#define REACH_ARM_TARGET   160  // 前伸时的中臂角度

#define BACK_BASE_TARGET   90   // 收回时的底座角度
#define BACK_ARM_TARGET    30   // 收回时的中臂角度

// ================== 硬件通道定义 (映射到 I2C 驱动板) ==================
#define BASE_CH     13  // 对应底盘
#define ARM_CH      14  // 对应中臂
#define CLAW_CH     15  // 对应夹爪

// ================== PWM 映射参数 (0-4095 范围) ==================
// 0.5ms~2.5ms 对应 50Hz 下的寄存器值 (安全物理边界)
#define SERVO_MIN_PWM   108    // 约 0.5ms (0度)
#define SERVO_MAX_PWM   498    // 约 2.5ms (180度)

// ================== 初始安全角度  ==================
#define BASE_INIT_ANGLE   90    //直立状态
#define ARM_INIT_ANGLE    20    //收回状态（修改为 20° 初始值）
#define CLAW_OPEN_ANGLE   125   //张爪状态(上电默认)
#define CLAW_CLOSE_ANGLE  165   //闭爪状态

// ================== 运动参数 ==================
#define SERVO_SPEED_MS_DEG_BASE 40  // 底座运动速度：毫秒/度 (越大越慢)
#define SERVO_SPEED_MS_DEG_ARM  20  // 中臂运动速度：毫秒/度
#define SERVO_SPEED_MS_DEG_CLAW 5   // 夹爪运动速度：毫秒/度 (可相对快些)

/* --- 固定修正偏离量 (如果需要机械初始误差修正可在此修改) --- */
#define BASE_OFFSET 0
#define ARM_OFFSET  0
#define CLAW_OFFSET 0

/* --- 函数接口 --- */
void Servo_Init(uint16_t freq_hz);
void Servo_Handle_Tick(void);
void Servo_MoveTo(int16_t b_target, int16_t a_target, int16_t c_target);
void Servo_SetAngleDirect(uint8_t Channel, int16_t Angle);
void Servo_MoveClawInstant(int16_t angle); // 夹爪瞬动 (跳过缓动插补)

/* --- 状态机函数接口 --- */
void Servo_Task_GrabProcess(void);
void Servo_StartGrabCycle(uint8_t distance); // 触发一次抓取流程, 传入球距离
uint8_t Servo_IsGrabDone(void);  // 抓取流程是否已完成 (回到 GRAB_IDLE)

#endif /* __SERVO_H */