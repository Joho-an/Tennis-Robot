#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

// ============================================================
//  舵机指令枚举 (统一由 main.c 消费)
// ============================================================
typedef enum {
    SERVO_CMD_NONE           = 0,   // 无指令
    SERVO_CMD_RESET          = 7,   // 舵机复位 (全部回初始角度)

    // base 固定角度 (单字节, 右手柄Y轴满拉触发)
    SERVO_CMD_BASE_FRONT     = 'b', // base前伸 0°
    SERVO_CMD_BASE_BACK      = 'a', // base后缩 180°

    // arm 步进 (单字节, 右手柄X轴持续发送)
    SERVO_CMD_ARM_STEP_INC   = 'c', // arm 步进 +1°
    SERVO_CMD_ARM_STEP_DEC   = 'd', // arm 步进 -1°

    // arm 固定角度 (单字节, 右手柄Y轴满拉触发)
    SERVO_CMD_ARM_FRONT      = 'g', // arm前伸到固定角度 120°
    SERVO_CMD_ARM_BACK       = 'h', // arm后  缩到固定角度 0°

    SERVO_CMD_CLAW_CLOSE     = 'e', // 夹爪闭合 180°
    SERVO_CMD_CLAW_OPEN      = 'f', // 夹爪张开 110°
} ServoCmd_t;

// ============================================================
//  APP 控制数据结构体
// ============================================================
typedef struct {
    uint8_t  is_connected;  // 连接状态: 0=断开, 1=已连接
    uint32_t last_rx_tick;  // 上次收到数据时间戳 (断连保护)

    uint8_t  mode;          // 运行模式: 0=自动, 1=手动
    uint8_t  direction;     // 运动方向: 0=停, 1=前, 2=后, 3=左, 4=右
    uint8_t  servo_cmd;     // 舵机指令 (ServoCmd_t 枚举值), 0=无
    uint8_t  has_joystick;  // 1=摇杆在线

    // 摇杆数据 (F103手柄 / APP虚拟摇杆)
    // 数据包: [0xBB, joy_l_y, joy_l_x, 0xEE]
    uint8_t  joy_l_y;       // 左摇杆 Y (0=上推, 128=中位, 255=下拉)
    uint8_t  joy_l_x;       // 左摇杆 X (0=左推, 128=中位, 255=右推)
    uint8_t  joy_r_y;       // 右摇杆 Y
    uint8_t  joy_r_x;       // 右摇杆 X

    // arm 步进控制 (右手柄X轴持续增量)
    uint8_t  arm_step_inc;  // 1=收到步进+指令, main消费后清零
    uint8_t  arm_step_dec;  // 1=收到步进-指令, main消费后清零

    /* Debug */
    uint16_t last_rx_size;  // 最后接收字节数
    uint8_t  raw_data[8];   // 原始数据前8字节 (LCD显示用)
} AppCtrl_t;

extern AppCtrl_t g_app_ctrl;

// ============================================================
//  函数声明
// ============================================================
void Bluetooth_Init(void);
void Bluetooth_CheckStatus(void);
void Bluetooth_HandleRxEvent(uint16_t Size);

// ============================================================
//  参数宏
// ============================================================
#define BT_TIMEOUT_MS   150   // 蓝牙断连超时 (ms)
#define JOY_MAX_SPEED   350   // 摇杆满推对应速度 mm/s (前进/后退)
#define JOY_MAX_TURN    250   // 摇杆满推对应转弯速度 mm/s (左转/右转)
#define JOY_DEADBAND    6     // 摇杆中位死区: |val-128|≤6 视为中位 (约5%, 6/128≈4.7%)

#endif /* __BLUETOOTH_H */