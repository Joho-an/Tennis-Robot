#include "Openmv.h"
#include "usart.h"
#include "bluetooth.h"

/* ==================== 全局变量 ==================== */
// OpenMV 数据结构体
OpenMVData_t g_openmv_data = {0};
volatile uint32_t uart4_rx_count = 0;  // UART4 回调计数 (调试用)

// UART4 DMA 接收缓冲区, 必须放在 D2 SRAM (DTCM 不可被 DMA1 访问!)
static uint8_t uart4_rx_buf[32] __attribute__((section(".RAM_D2"))) __attribute__((aligned(32)));

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化 OpenMV 的 UART4 DMA 接收
 *
 * OpenMV 通过 UART4 发送 6 字节二进制数据包:
 * [0x55, BALL_X, DIST, LINE_DEV, 0x00, 0xAA]
 */
void OpenMV_Init(void) {
    // 关闭 FIFO (避免 H7 DMA+空闲中断+FIFO 组合问题)
    HAL_UARTEx_DisableFifoMode(&huart4);

    // 启动 UART4 的 DMA 空闲中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_buf, 32);

    // 禁用过半中断，只关注空闲中断（数据接收完毕）
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
}

/**
 * @brief 检查是否有新数据
 * @return 1 - 有新数据，0 - 无新数据
 */
uint8_t OpenMV_HasNewData(void) {
    return g_openmv_data.is_updated;
}

/**
 * @brief 清除新数据标志位
 * 
 * 应在应用程序处理完数据后调用此函数
 */
void OpenMV_ClearFlag(void) {
    g_openmv_data.is_updated = 0;
}

/**
 * @brief UART4 DMA 接收完成中断回调函数
 * 
 * 当 UART4 接收到数据包或超时时触发
 * 负责解析 OpenMV 发送的数据包并更新 g_openmv_data
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART4) {
        // 只在收到有效数据时计数 (Size>0 才是真数据)
        if (Size > 0) uart4_rx_count++;

        // H7 必须先使 D-Cache 失效，再读 DMA 缓冲区
        SCB_InvalidateDCache_by_Addr((uint32_t *)uart4_rx_buf, 32);

        // 保存前6字节供 LCD 调试
        g_openmv_data.raw_len = (uint8_t)(Size > 6 ? 6 : Size);
        for (int j = 0; j < 6; j++) {
            g_openmv_data.raw_buf[j] = (j < Size) ? uart4_rx_buf[j] : 0xFF;
        }

        // 协议: 0x55 + BALL_X + DIST + LINE_DEV + 0x00 + 0xAA 共6字节
        // 原始二进制, 从后往前扫描, 取最后一帧(最新数据)
        for (int i = (int)Size - 6; i >= 0; i--) {
            if (uart4_rx_buf[i] == 0x55 && uart4_rx_buf[i + 5] == 0xAA) {
                g_openmv_data.ball_x    = uart4_rx_buf[i + 1];  // 网球X坐标
                g_openmv_data.distance  = uart4_rx_buf[i + 2];  // 距离
                g_openmv_data.offset    = (int8_t)uart4_rx_buf[i + 3];  // 色带偏离(有符号)
                g_openmv_data.reserved  = uart4_rx_buf[i + 4];  // 保留
                g_openmv_data.is_updated  = 1;
                g_openmv_data.relay_ready = 1;
                break;
            }
        }

        // 重新开启 UART4 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_buf, 32);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    } else if (huart->Instance == UART5) {
        Bluetooth_HandleRxEvent(Size);
    }
}

/**
 * @brief 将最新 OpenMV 数据转发到蓝牙主机
 */
void OpenMV_RelayToUART5(void) {
    if (!g_openmv_data.relay_ready) return;
    g_openmv_data.relay_ready = 0;

    extern UART_HandleTypeDef huart5;
    uint8_t pkt[6];
    pkt[0] = 0xDD;
    pkt[1] = (uint8_t)g_openmv_data.offset;
    pkt[2] = g_openmv_data.distance;
    pkt[3] = g_openmv_data.ball_x;
    pkt[4] = 0;
    pkt[5] = 0xEE;
    HAL_UART_Transmit(&huart5, pkt, 6, 10);
}

/**
 * @brief UART 错误回调函数，处理溢出等错误导致的死机，进行串口恢复
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // 发生错误时，先恢复错误状态，然后重新启动接收
    if (huart->Instance == UART5) {
        // 清除错误标志
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        
        // 尝试重新开启接收
        extern uint8_t bt_rx_buf[32];
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_IT(huart, bt_rx_buf, 32);
    } else if (huart->Instance == UART4) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_buf, 32);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }
}

