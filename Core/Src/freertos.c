/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LCD.h"
#include "battery.h"
#include "servo.h"
#include "motor.h"
#include "Openmv.h"
#include "bluetooth.h"
#include "pid.h"
#include "auto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for BluetoothTask */
osThreadId_t BluetoothTaskHandle;
const osThreadAttr_t BluetoothTask_attributes = {
  .name = "BluetoothTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(StartTask02, NULL, &ServoTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartTask03, NULL, &MotorTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartTask04, NULL, &DisplayTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartTask05, NULL, &ControlTask_attributes);

  /* creation of BluetoothTask */
  BluetoothTaskHandle = osThreadNew(StartTask06, NULL, &BluetoothTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Idle / health-check task — kept minimal */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* ServoTask — 舵机平滑运动 + 抓取状态机, 10ms 周期 */
  for(;;)
  {
    Servo_Handle_Tick();
    Servo_Task_GrabProcess();
    osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* MotorTask — 编码器刷新 + PID 闭环, 100ms 周期 */
  for(;;)
  {
    Motor_UpdateStatus();
    Motor_RunPID();
    osDelay(100);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* DisplayTask — LCD 信息刷新, 500ms 周期 */
  for(;;)
  {
    char buf[32];

    MotorStatus_t L = Motor_GetStatus(MOTOR_LEFT);
    MotorStatus_t R = Motor_GetStatus(MOTOR_RIGHT);

    char sign_Lv = L.speed < 0 ? '-' : ' ';
    int32_t abs_Lv = L.speed < 0 ? -L.speed : L.speed;
    char sign_Ld = L.distance < 0 ? '-' : ' ';
    int32_t abs_Ld = L.distance < 0 ? -L.distance : L.distance;

    char sign_Rv = R.speed < 0 ? '-' : ' ';
    int32_t abs_Rv = R.speed < 0 ? -R.speed : R.speed;
    char sign_Rd = R.distance < 0 ? '-' : ' ';
    int32_t abs_Rd = R.distance < 0 ? -R.distance : R.distance;

    // 左电机
    sprintf(buf, "L_v:%c%ld.%01ld cm/s   ", sign_Lv, abs_Lv / 10, abs_Lv % 10);
    ST7735_DrawString(5, 5, buf, WHITE, BLACK);
    sprintf(buf, "L_D:%c%ld.%01ld cm   ", sign_Ld, abs_Ld / 10, abs_Ld % 10);
    ST7735_DrawString(5, 15, buf, YELLOW, BLACK);

    // 右电机
    sprintf(buf, "R_v:%c%ld.%01ld cm/s   ", sign_Rv, abs_Rv / 10, abs_Rv % 10);
    ST7735_DrawString(5, 35, buf, WHITE, BLACK);
    sprintf(buf, "R_D:%c%ld.%01ld cm   ", sign_Rd, abs_Rd / 10, abs_Rd % 10);
    ST7735_DrawString(5, 45, buf, YELLOW, BLACK);

    // base / arm 当前角度
    sprintf(buf, "Base:%3d Arm:%3d    ", base_angle, arm_angle);
    ST7735_DrawString(5, 55, buf, WHITE, BLACK);

    // claw 当前/目标角度
    sprintf(buf, "Claw:%3d    ", claw_angle);
    ST7735_DrawString(5, 65, buf, WHITE, BLACK);

    // OpenMV 数据
    {
        static uint32_t last_openmv_tick = 0;
        static uint32_t last_rx_count = 0;
        uint32_t now = HAL_GetTick();
        if (uart4_rx_count != last_rx_count) {
            last_rx_count = uart4_rx_count;
            last_openmv_tick = now;
        }
        if (now - last_openmv_tick > 1000) {
            sprintf(buf, "OpenMV Error!!    ");
            ST7735_DrawString(5, 88, buf, RED, BLACK);
        } else {
            sprintf(buf, "RX:%lu            ", uart4_rx_count);
            ST7735_DrawString(5, 88, buf, WHITE, BLACK);
        }
        sprintf(buf, "TF:%d DS:%d ", g_openmv_data.offset, g_openmv_data.distance);
        ST7735_DrawString(5, 78, buf, YELLOW, BLACK);
    }

    // 电池
    Battery_Show();

    // 蓝牙状态
    sprintf(buf, "Mode:%s Dir:%d ", g_app_ctrl.mode == 0 ? "AUTO  " : "MANUAL", g_app_ctrl.direction);
    ST7735_DrawString(5, 100, buf, YELLOW, BLACK);
    sprintf(buf, "Rx: %02X %02X %02X %02X  ", g_app_ctrl.raw_data[0], g_app_ctrl.raw_data[1], g_app_ctrl.raw_data[2], g_app_ctrl.raw_data[3]);
    ST7735_DrawString(5, 110, buf, WHITE, BLACK);

    // 舵机状态 + 自动模式球数
    if (g_app_ctrl.mode == 0) {
        sprintf(buf, "S:%s B:%d    ", servo_init_ok ? "OK" : "ER",
                AutoMode_GetBallCount());
    } else {
        sprintf(buf, "S:%s    ", servo_init_ok ? "OK" : "ER");
    }
    ST7735_DrawString(5, 120, buf, servo_init_ok ? GREEN : RED, BLACK);

    osDelay(500);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* ControlTask — 模式切换检测 + 自动/手动模式控制逻辑 */
  static uint8_t last_mode = 255;

  for(;;)
  {
    // ---- 模式切换检测 ----
    if (g_app_ctrl.mode != last_mode) {
        last_mode = g_app_ctrl.mode;
        if (g_app_ctrl.mode == 0) {
            AutoMode_Reset();
        } else {
            Motor_Stop();
        }
    }

    if (g_app_ctrl.mode == 0) {
        // ========== 自动模式 ==========
        AutoMode_Run();
    } else {
        // ========== 手动模式 ==========

        // ---- 运动方向控制 (摇杆全向公式) ----
        {
            int32_t raw_y = g_app_ctrl.joy_l_y;
            int32_t raw_x = g_app_ctrl.joy_l_x;

            int32_t diff_y = raw_y - 128;
            int32_t diff_x = raw_x - 128;
            if (diff_y < 0) diff_y = -diff_y;
            if (diff_x < 0) diff_x = -diff_x;
            if (diff_y <= JOY_DEADBAND) raw_y = 128;
            if (diff_x <= JOY_DEADBAND) raw_x = 128;

            int32_t jy = (int32_t)128 - raw_y;
            int32_t jx = (int32_t)raw_x - 128;

            int32_t abs_jy = jy < 0 ? -jy : jy;
            int32_t abs_jx = jx < 0 ? -jx : jx;
            int32_t base = (abs_jy * abs_jy / 128) * JOY_MAX_SPEED / 128;
            int32_t diff = (abs_jx * abs_jx / 128) * JOY_MAX_TURN / 128;
            if (jy < 0) base = -base;
            if (jx < 0) diff = -diff;

            int32_t left  = base + diff;
            int32_t right = base - diff;
            if (left  > JOY_MAX_SPEED)  left  = JOY_MAX_SPEED;
            if (left  < -JOY_MAX_SPEED) left  = -JOY_MAX_SPEED;
            if (right > JOY_MAX_SPEED)  right = JOY_MAX_SPEED;
            if (right < -JOY_MAX_SPEED) right = -JOY_MAX_SPEED;
            Motor_SetTargetSpeed((int16_t)left, (int16_t)right);
        }

        // ---- 舵机指令 (统一消费 servo_cmd) ----
        if (g_app_ctrl.servo_cmd != SERVO_CMD_NONE) {
            switch (g_app_ctrl.servo_cmd) {
                case SERVO_CMD_RESET:
                    Servo_MoveTo(BASE_INIT_ANGLE, ARM_INIT_ANGLE, CLAW_OPEN_ANGLE);
                    break;
                case SERVO_CMD_BASE_FRONT:
                    Servo_MoveTo(REACH_BASE_TARGET, arm_target, claw_target);
                    break;
                case SERVO_CMD_BASE_BACK:
                    Servo_MoveTo(BACK_BASE_TARGET, arm_target, claw_target);
                    break;
                case SERVO_CMD_ARM_FRONT:
                    Servo_MoveTo(base_target, REACH_ARM_TARGET, claw_target);
                    break;
                case SERVO_CMD_ARM_BACK:
                    Servo_MoveTo(base_target, BACK_ARM_TARGET, claw_target);
                    break;
                case SERVO_CMD_CLAW_CLOSE:
                    Servo_MoveTo(base_target, arm_target, CLAW_GRAB_ANGLE);
                    break;
                case SERVO_CMD_CLAW_OPEN:
                    Servo_MoveTo(base_target, arm_target, CLAW_RELEASE_ANGLE);
                    break;
                default:
                    break;
            }
            g_app_ctrl.servo_cmd = SERVO_CMD_NONE;
        }

        // ---- arm 步进控制 ----
        {
            int16_t step = 0;
            if (g_app_ctrl.arm_step_inc) { step =  1; g_app_ctrl.arm_step_inc = 0; }
            if (g_app_ctrl.arm_step_dec) { step = -1; g_app_ctrl.arm_step_dec = 0; }
            if (step != 0) {
                int16_t new_target = arm_target + step;
                if (new_target < 0)   new_target = 0;
                if (new_target > 180) new_target = 180;
                Servo_MoveTo(base_target, new_target, claw_target);
            }
        }
    }

    osDelay(10);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the BluetoothTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* BluetoothTask — 蓝牙断连检测 + OpenMV 数据转发 */
  for(;;)
  {
    Bluetooth_CheckStatus();
    OpenMV_RelayToUART5();
    osDelay(10);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

