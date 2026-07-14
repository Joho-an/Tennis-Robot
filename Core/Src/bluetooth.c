#include "bluetooth.h"
#include "usart.h"

// 舵机目标角度变量 (定义在 servo.c 中)
extern int16_t base_target;
extern int16_t arm_target;
extern int16_t claw_target;

AppCtrl_t g_app_ctrl = {0};

// UART5 (蓝牙) DMA 接收缓冲区，32字节对齐以适配 H7 Cache
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    __attribute__((section(".RAM_D2"))) ALIGN_32BYTES(uint8_t bt_rx_buf[32]);
#else
    ALIGN_32BYTES(uint8_t bt_rx_buf[32]);
#endif

/**
 * @brief 初始化蓝牙接收
 */
void Bluetooth_Init(void) {
    // 启动 UART5 的空闲中断接收
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, bt_rx_buf, 32);

    g_app_ctrl.mode = 1;  // 默认手动模式
    g_app_ctrl.joy_l_y = 128;  // 摇杆初始中位
    g_app_ctrl.joy_l_x = 128;
    g_app_ctrl.joy_r_y = 128;
    g_app_ctrl.joy_r_x = 128;
    g_app_ctrl.last_rx_tick = HAL_GetTick();
}

/**
 * @brief UART5 接收事件处理
 * @param Size 接收字节数
 *
 * 协议定义:
 *   [摇杆包]  0xBB + joy_l_y + joy_l_x + 0xEE (4字节)
 *   [单字节]  '0'=急停  '1'=前进  '2'=后退  '3'=左转  '4'=右转
 *             '7'=舵机复位  '8'=自动模式  '9'=手动模式
 *             'a'=base后缩180°  'b'=base前伸0°
 *             'c'=arm步进+1°  'd'=arm步进-1°
 *             'e'=夹爪闭合180°  'f'=夹爪张开110°
 *             'g'=arm前伸到固定120°  'h'=arm后缩到固定0°
 */
void Bluetooth_HandleRxEvent(uint16_t Size) {
    if (Size == 0) {
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, bt_rx_buf, 32);
        return;
    }

    g_app_ctrl.last_rx_tick = HAL_GetTick();
    g_app_ctrl.is_connected = 1;

    // 缓存原始数据供 LCD 调试显示
    g_app_ctrl.last_rx_size = Size;
    for (int i = 0; i < 8; i++) {
        g_app_ctrl.raw_data[i] = (i < Size) ? bt_rx_buf[i] : 0;
    }

    // 逐字节扫描: 支持摇杆包 + 单字符指令混合粘连
    int i = 0;
    while (i < Size) {
        // ============================================================
        //  协议1: 摇杆数据包 (4字节帧)
        //  [0xBB, joy_l_y, joy_l_x, 0xEE]
        // ============================================================
        if (i + 4 <= Size && bt_rx_buf[i] == 0xBB && bt_rx_buf[i + 3] == 0xEE) {
            g_app_ctrl.joy_l_y      = bt_rx_buf[i + 1];
            g_app_ctrl.joy_l_x      = bt_rx_buf[i + 2];
            g_app_ctrl.has_joystick = 1;
            i += 4;
            continue;
        }

        uint8_t val = bt_rx_buf[i];

        // ============================================================
        //  协议2: 单字节指令
        // ============================================================

        // ---- 数字指令 '0'~'9' ----
        if (val >= '0' && val <= '9') {
            switch (val) {
                case '0':  // 急停: 电机速度=0, 摇杆回中
                    g_app_ctrl.direction   = 0;
                    g_app_ctrl.joy_l_y     = 128;
                    g_app_ctrl.joy_l_x     = 128;
                    g_app_ctrl.has_joystick = 1;
                    break;
                case '1':  // 前进: 摇杆上推
                    g_app_ctrl.direction   = 1;
                    g_app_ctrl.joy_l_y     = 0;
                    g_app_ctrl.joy_l_x     = 128;
                    g_app_ctrl.has_joystick = 1;
                    break;
                case '2':  // 后退: 摇杆下拉
                    g_app_ctrl.direction   = 2;
                    g_app_ctrl.joy_l_y     = 255;
                    g_app_ctrl.joy_l_x     = 128;
                    g_app_ctrl.has_joystick = 1;
                    break;
                case '3':  // 左转: 摇杆左推
                    g_app_ctrl.direction   = 3;
                    g_app_ctrl.joy_l_y     = 128;
                    g_app_ctrl.joy_l_x     = 0;
                    g_app_ctrl.has_joystick = 1;
                    break;
                case '4':  // 右转: 摇杆右推
                    g_app_ctrl.direction   = 4;
                    g_app_ctrl.joy_l_y     = 128;
                    g_app_ctrl.joy_l_x     = 255;
                    g_app_ctrl.has_joystick = 1;
                    break;
                case '7':  // 舵机复位: 全部回初始角度
                    g_app_ctrl.servo_cmd = SERVO_CMD_RESET;
                    break;
                case '8':  // 自动模式
                    g_app_ctrl.mode = 0;
                    break;
                case '9':  // 手动模式
                    g_app_ctrl.mode = 1;
                    break;
                default:
                    break;
            }
            i++;
            continue;
        }

        // ---- 舵机指令 'a'~'h' ----
        if ((val >= 'a' && val <= 'h')) {
            // 仅手动模式响应舵机指令
            if (g_app_ctrl.mode != 0) {
                switch (val) {
                    case 'a': g_app_ctrl.servo_cmd = SERVO_CMD_BASE_BACK;    break; // base后缩180°
                    case 'b': g_app_ctrl.servo_cmd = SERVO_CMD_BASE_FRONT;   break; // base前伸0°
                    case 'c': g_app_ctrl.arm_step_inc = 1;                   break; // arm步进+0.5°
                    case 'd': g_app_ctrl.arm_step_dec = 1;                   break; // arm步进-0.5°
                    case 'e': g_app_ctrl.servo_cmd = SERVO_CMD_CLAW_CLOSE;   break; // 夹爪闭合180°
                    case 'f': g_app_ctrl.servo_cmd = SERVO_CMD_CLAW_OPEN;    break; // 夹爪张开110°
                    case 'g': g_app_ctrl.servo_cmd = SERVO_CMD_ARM_FRONT;    break; // arm前伸到固定120°
                    case 'h': g_app_ctrl.servo_cmd = SERVO_CMD_ARM_BACK;     break; // arm后缩到固定0°
                    default: break;
                }
            }
            i++;
            continue;
        }

        i++; // 未识别字节, 跳过
    }

    HAL_UARTEx_ReceiveToIdle_IT(&huart5, bt_rx_buf, 32);
}

/**
 * @brief 蓝牙信号看门狗
 *
 * 超过 BT_TIMEOUT_MS 无数据 → 断连 → 急停保护
 */
void Bluetooth_CheckStatus(void) {
    if (HAL_GetTick() - g_app_ctrl.last_rx_tick > BT_TIMEOUT_MS) {
        if (g_app_ctrl.is_connected == 1) {
            g_app_ctrl.is_connected = 0;
            g_app_ctrl.direction    = 0;
            g_app_ctrl.servo_cmd    = SERVO_CMD_NONE;
            g_app_ctrl.has_joystick = 0;
            g_app_ctrl.arm_step_inc = 0;
            g_app_ctrl.arm_step_dec = 0;
            // 摇杆回中 → 松手即停
            g_app_ctrl.joy_l_y = 128;
            g_app_ctrl.joy_l_x = 128;
            g_app_ctrl.joy_r_y = 128;
            g_app_ctrl.joy_r_x = 128;
        }
    }
}