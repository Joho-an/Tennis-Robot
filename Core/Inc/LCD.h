#ifndef __LCD_H
#define __LCD_H

#include "main.h" // 包含HAL库生成的定义，确保能识别GPIO_Pin

/* --- 屏幕参数 --- */
// 注意：你提到的实际x:0~155, y:0~115 说明物理偏移已经包含在逻辑内部
#define LCD_WIDTH   160
#define LCD_HEIGHT  130

/* --- 颜色定义 (RGB565格式) --- */
#define WHITE   0xFFFF
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define YELLOW  0xFFE0
#define GRAY    0X8430
#define BROWN   0XBC40

/* --- 引脚控制封装 --- */
/* 这些宏直接映射到你在 CubeMX 中为引脚设置的 User Label */
#define LCD_RES_CLR()  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RES_SET()  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)

#define LCD_DC_CLR()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_SET()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)

#define LCD_CS_CLR()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_SET()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)

#define LCD_BLK_SET()  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET)
#define LCD_BLK_CLR()  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET)

/* --- 基础功能函数声明 --- */
void ST7735_Init(SPI_HandleTypeDef *hspi);
void ST7735_Clear(uint16_t color);
void ST7735_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/* --- 图形与图像绘制 --- */
void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pImage);

/* --- 字符与字符串显示 --- */
void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t textColor, uint16_t bgColor);
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t textColor, uint16_t bgColor);

/* --- 中文显示功能 --- */
// 注意：s 对应字库中的索引或编码，取决于你的 font.h 实现
void ST7735_DrawChinese16(uint16_t x, uint16_t y, char *s, uint16_t textColor, uint16_t bgColor);
void ST7735_ShowChineseString(uint16_t x, uint16_t y, char *str, uint16_t textColor, uint16_t bgColor);

#endif /* __LCD_ST7735_H */