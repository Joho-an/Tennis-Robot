#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "main.h"

/* ==================== OpenMV 数据结构 ==================== */
// Openmv 数据包: [0x55, BALL_X, DIST, LINE_DEV, 0x00, 0xAA] 共6字节
typedef struct {
    uint8_t  ball_x;        // 网球 X 坐标 (0~255, 320等比映射)
    uint8_t  distance;      // 网球距离 (cm)
    int8_t   offset;        // 色带偏离值 (-128~127)
    uint8_t  reserved;      // 保留
    uint8_t  is_updated;
    uint8_t  relay_ready;

    uint8_t  raw_len;
    uint8_t  raw_buf[6];    // 6字节数据包
} OpenMVData_t;

/* ==================== 外部全局变量 ==================== */
extern OpenMVData_t g_openmv_data;
extern volatile uint32_t uart4_rx_count;   // UART4 回调计数 (调试)

/* ==================== 函数声明 ==================== */
void OpenMV_Init(void);                 // 初始化 OpenMV 的 UART4 接收
uint8_t OpenMV_HasNewData(void);        // 检查是否有新数据
void OpenMV_ClearFlag(void);            // 清除新数据标志
void OpenMV_RelayToUART5(void);         // 将最新 OpenMV 数据转发到蓝牙主机

#endif /* __OPENMV_H__ */