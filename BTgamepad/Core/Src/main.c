/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t l_y;  // 左Y (0上推,128中,255下拉)
    uint8_t l_x;  // 左X (0左推,128中,255右推)
    uint8_t r_y;  // 右Y
    uint8_t r_x;  // 右X
} Joystick_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JOY_INTERVAL_MS 20    // 摇杆包发送间隔
#define BTN_DEBOUNCE_MS 20    // 按键去抖时间
#define JOY_TRIGGER_THRESH  15       // 右摇杆拉满阈值: <15 或 >240
#define JOY_TRIGGER_COOLDOWN_MS 5000 // 同一方向触发冷却 5s
#define JOY_CENTER       128        // 中位值
#define JOY_CENTER_DB    10         // 中位死区 10/255≈4%, 在[118,138]内视为中位
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;

// 摇杆
static Joystick_t joy = {128, 128, 128, 128};
static uint32_t last_joy_tick = 0;

// 按键: 0=未按, 1=按下
static uint8_t  btn_state[8];       // 当前去抖后状态
static uint32_t btn_last_tick[8];   // 上次变化时间戳

// 右摇杆触发: -1=负向拉满, 0=中间, +1=正向拉满
static int8_t   rx_zone_prev = 0;
static int8_t   ry_zone_prev = 0;
static uint32_t last_trigger_tick[4]; // [0]='a',[1]='b',[2]='c',[3]='d'
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t ADC_Read(uint32_t channel);
static void     Joystick_Update(void);
static void     Joystick_Send(void);
static void     Joystick_Trigger_Send(void);
static void     Buttons_Scan(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 按键GPIO: {port, pin, 按下发送指令(待填)}
static const struct {
    GPIO_TypeDef *port; uint16_t pin;
    char cmd;  // 蓝牙发送指令, 后面自己填
} btns[8] = {
    {GPIOB, Key1_Pin, '8'},  // BTN1 (PB14)--AUTO MODE
    {GPIOB, Key2_Pin, 'g'},  // BTN2 (PB13)--前伸
    {GPIOB, Key3_Pin, 'e'},  // BTN3 (PB12)--爪子张开
    {GPIOB, Key4_Pin, '9'},  // BTN4 (PB4)--MANUAL MODE
    {GPIOB, Key5_Pin, 'h'},  // BTN5 (PB5)--收回
    {GPIOB, Key6_Pin, 'f'},  // BTN6 (PB6)--爪子闭合
    {GPIOA, LSW_Pin,  '0'},  // L_SW (PA11) 左手柄按键--电机急停
    {GPIOB, RSW_Pin,  '7'},  // R_SW (PB3)  右手柄按键--机械臂复位
};

// ADC 通道: PA0~PA3 → ADC1_IN0~IN3
#define ADC_CH_LX  ADC_CHANNEL_2  // PA2 → LX → 左X → 转向
#define ADC_CH_LY  ADC_CHANNEL_3  // PA3 → LY → 左Y → 前进后退
#define ADC_CH_RX  ADC_CHANNEL_0  // PA0 → RX → 右X
#define ADC_CH_RY  ADC_CHANNEL_1  // PA1 → RY → 右Y

static uint16_t ADC_Read(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

static void Joystick_Update(void) {
    uint16_t adc;
    int16_t  val;

    // 左X (取反, 中位死区: [118,138] → 归为128)
    adc = ADC_Read(ADC_CH_LX);
    val = JOY_CENTER - (int16_t)(adc >> 4) + JOY_CENTER;  // val = 256 - (adc>>4)
    if (val < 0) val = 0; else if (val > 255) val = 255;
    if (val >= JOY_CENTER - JOY_CENTER_DB && val <= JOY_CENTER + JOY_CENTER_DB) val = JOY_CENTER;
    joy.l_x = (uint8_t)val;

    // 左Y (取反, 中位死区)
    adc = ADC_Read(ADC_CH_LY);
    val = JOY_CENTER - (int16_t)(adc >> 4) + JOY_CENTER;
    if (val < 0) val = 0; else if (val > 255) val = 255;
    if (val >= JOY_CENTER - JOY_CENTER_DB && val <= JOY_CENTER + JOY_CENTER_DB) val = JOY_CENTER;
    joy.l_y = (uint8_t)val;

    // 右X
    adc = ADC_Read(ADC_CH_RX);
    val = (int16_t)(adc >> 4);  // 不取反, val = adc>>4 直接
    if (val < 0) val = 0; else if (val > 255) val = 255;
    if (val >= JOY_CENTER - JOY_CENTER_DB && val <= JOY_CENTER + JOY_CENTER_DB) val = JOY_CENTER;
    joy.r_x = (uint8_t)val;

    // 右Y
    adc = ADC_Read(ADC_CH_RY);
    val = (int16_t)(adc >> 4);
    if (val < 0) val = 0; else if (val > 255) val = 255;
    if (val >= JOY_CENTER - JOY_CENTER_DB && val <= JOY_CENTER + JOY_CENTER_DB) val = JOY_CENTER;
    joy.r_y = (uint8_t)val;
}

// 协议: [0xBB, l_y, l_x, 0xEE] 匹配 H7 bluetooth.c 解析
// 仅左摇杆: 左Y(前进后退) + 左X(转向)
// 全部在中位时不发送; 任意轴偏离中位则恢复20ms持续发送
static void Joystick_Send(void) {
    if (joy.l_y == JOY_CENTER && joy.l_x == JOY_CENTER) {
        return;  // 左摇杆中位, 静默
    }
    uint8_t buf[4];
    buf[0] = 0xBB;
    buf[1] = joy.l_y;
    buf[2] = joy.l_x;
    buf[3] = 0xEE;
    HAL_UART_Transmit(&huart1, buf, 4, 10);
}

// 右摇杆四向触发:
//   RX左(c)/RX右(d): 偏离中位时每20ms持续发送, 中位死区[118,138]停发
//   RY上(b)/RY下(a): 边缘触发 + 5s冷却 (保持不变)
static void Joystick_Trigger_Send(void) {
    uint32_t now = HAL_GetTick();

    int8_t rx_zone, ry_zone;
    if      (joy.r_x < JOY_TRIGGER_THRESH)           rx_zone = -1;
    else if (joy.r_x > 255 - JOY_TRIGGER_THRESH)     rx_zone = +1;
    else                                             rx_zone =  0;

    if      (joy.r_y < JOY_TRIGGER_THRESH)           ry_zone = -1;
    else if (joy.r_y > 255 - JOY_TRIGGER_THRESH)     ry_zone = +1;
    else                                             ry_zone =  0;

    // ---- c/d: 持续发送, 只要偏离中位就发, 无冷却 ----
    if (joy.r_x != JOY_CENTER) {
        if (rx_zone == -1) {
            uint8_t c = 'c'; HAL_UART_Transmit(&huart1, &c, 1, 10);
        } else if (rx_zone == +1) {
            uint8_t c = 'd'; HAL_UART_Transmit(&huart1, &c, 1, 10);
        }
    }

    // ---- a/b: 边缘触发 + 5s冷却 (保持不变) ----
    if (ry_zone == -1 && ry_zone_prev != -1 &&
        (now - last_trigger_tick[0] >= JOY_TRIGGER_COOLDOWN_MS)) {
        uint8_t c = 'b'; HAL_UART_Transmit(&huart1, &c, 1, 10);
        last_trigger_tick[0] = now;
        HAL_Delay(2);
    }
    if (ry_zone == +1 && ry_zone_prev != +1 &&
        (now - last_trigger_tick[1] >= JOY_TRIGGER_COOLDOWN_MS)) {
        uint8_t c = 'a'; HAL_UART_Transmit(&huart1, &c, 1, 10);
        last_trigger_tick[1] = now;
        HAL_Delay(2);
    }

    rx_zone_prev = rx_zone;
    ry_zone_prev = ry_zone;
}

static void Buttons_Scan(void) {
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < 8; i++) {
        // 有上拉, 按下=低电平
        uint8_t raw = (HAL_GPIO_ReadPin(btns[i].port, btns[i].pin) == GPIO_PIN_RESET) ? 1 : 0;
        if (raw != btn_state[i] && (now - btn_last_tick[i] >= BTN_DEBOUNCE_MS)) {
            btn_state[i] = raw;
            btn_last_tick[i] = now;
            if (raw && btns[i].cmd != ' ') {
                // 按下时发送指令 (cmd != ' ' 才发)
                uint8_t c = (uint8_t)btns[i].cmd;
                HAL_UART_Transmit(&huart1, &c, 1, 10);
            }
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    // ---- 摇杆: 20ms 采集+发送 ----
    if (now - last_joy_tick >= JOY_INTERVAL_MS) {
        last_joy_tick = now;
        Joystick_Update();
        Joystick_Send();
        HAL_Delay(2);  // 间隔确保蓝牙模块分包, 避免合并 Joystick_Trigger_Send
        Joystick_Trigger_Send();
        HAL_Delay(2);  // 间隔确保蓝牙模块分包, 避免合并 Buttons_Scan
    }

    // ---- 按键扫描 ----
    Buttons_Scan();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
