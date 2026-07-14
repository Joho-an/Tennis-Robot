#include "LCD.h"
#include "font.h"
#include "spi.h"
#include <string.h>

/* 私有句柄 */
static SPI_HandleTypeDef *p_hspi_lcd;

/* H750 适配：根据你的 LCD 物理特性设置偏移 */
#define X_OFFSET 0
#define Y_OFFSET 0

/**
 * @brief 底层写命令
 */
static void LCD_WriteCmd(uint8_t cmd) {
    LCD_DC_CLR();
    LCD_CS_CLR();
    HAL_SPI_Transmit(p_hspi_lcd, &cmd, 1, 10);
    LCD_CS_SET();
}

/**
 * @brief 底层写数据 (8位)
 */
static void LCD_WriteData8(uint8_t data) {
    LCD_DC_SET();
    LCD_CS_CLR();
    HAL_SPI_Transmit(p_hspi_lcd, &data, 1, 10);
    LCD_CS_SET();
}

/**
 * @brief 设置显示窗口
 */
void ST7735_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    x1 += X_OFFSET; x2 += X_OFFSET;
    y1 += Y_OFFSET; y2 += Y_OFFSET;

    LCD_WriteCmd(0x2A); // Column Address Set
    LCD_WriteData8(x1 >> 8); LCD_WriteData8(x1 & 0xFF);
    LCD_WriteData8(x2 >> 8); LCD_WriteData8(x2 & 0xFF);

    LCD_WriteCmd(0x2B); // Row Address Set
    LCD_WriteData8(y1 >> 8); LCD_WriteData8(y1 & 0xFF);
    LCD_WriteData8(y2 >> 8); LCD_WriteData8(y2 & 0xFF);

    LCD_WriteCmd(0x2C); // Memory Write
}

/**
 * @brief 初始化 ST7735
 */
void ST7735_Init(SPI_HandleTypeDef *hspi) {
    p_hspi_lcd = hspi;

    // 硬件复位
    LCD_RES_CLR();
    HAL_Delay(50);
    LCD_RES_SET();
    HAL_Delay(120);

    // 软复位与唤醒
    LCD_WriteCmd(0x11); // Sleep Out
    HAL_Delay(120);

    // 像素格式设置 (RGB565)
    LCD_WriteCmd(0x3A);
    LCD_WriteData8(0x05);

    // 屏幕方向设置 (0x78 为横屏)
    LCD_WriteCmd(0x36);
    LCD_WriteData8(0x70);

    // 开启显示
    LCD_WriteCmd(0x29);
    LCD_BLK_SET(); // 开启背光

    ST7735_Clear(0x0000); // 初始黑屏
}

/**
 * @brief 填充矩形 (高效填充)
 */
void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    uint8_t color_data[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFF) };
    ST7735_SetAddress(x, y, x + w - 1, y + h - 1);
    
    LCD_DC_SET();
    LCD_CS_CLR();
    
    // 对于 H7，如果是大面积填充，使用循环发送
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        HAL_SPI_Transmit(p_hspi_lcd, color_data, 2, 10);
    }
    LCD_CS_SET();
}

/**
 * @brief 清屏
 */
void ST7735_Clear(uint16_t color) {
    ST7735_DrawRect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

/**
 * @brief 显示图片 (注意 H7 的 Cache 一致性)
 */
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pImage) {
    ST7735_SetAddress(x, y, x + w - 1, y + h - 1);
    LCD_DC_SET();
    LCD_CS_CLR();

    // H7 性能极高，这里直接发送整块数据
    HAL_SPI_Transmit(p_hspi_lcd, (uint8_t *)pImage, w * h * 2, HAL_MAX_DELAY);

    LCD_CS_SET();
}

/**
 * @brief 显示单个字符 (5x8)
 */
void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t textColor, uint16_t bgColor) {
    if (c < 32 || c > 126) return;
    uint8_t index = c - 32;
    uint8_t t_color[2] = { textColor >> 8, textColor & 0xFF };
    uint8_t b_color[2] = { bgColor >> 8, bgColor & 0xFF };

    ST7735_SetAddress(x, y, x + 4, y + 7);
    LCD_DC_SET();
    LCD_CS_CLR();

    for (int j = 0; j < 8; j++) {
        for (int i = 0; i < 5; i++) {
            if (ASCII_5x8[index][i] & (0x01 << j)) {
                HAL_SPI_Transmit(p_hspi_lcd, t_color, 2, 10);
            } else {
                HAL_SPI_Transmit(p_hspi_lcd, b_color, 2, 10);
            }
        }
    }
    LCD_CS_SET();
}

/**
 * @brief 显示字符串
 */
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t textColor, uint16_t bgColor) {
    while (*str) {
        if (x + 6 > LCD_WIDTH) { x = 0; y += 10; }
        if (y + 8 > LCD_HEIGHT) break;
        ST7735_DrawChar(x, y, *str, textColor, bgColor);
        x += 6;
        str++;
    }
}