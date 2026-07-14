# Tennis-Robot
---main中主要包含OpenMV视觉相关代码和硬件连接手册，其中Main.py可直接放入OpenMV SD卡后直接执行使用(不包含训练后的轻量化模型)。其余分支分别包含了H750主控.c和.h代码，以及自制手柄代码，host网页可用文件直接放在main中--
以下是简要的项目介绍：
FreeRTOS+轻量化视觉的网球收集机器人
开发环境：VS Code+CMake STM32Cubemx OpenMV IDE Python
技术栈：FreeRTOS/CMSIS-RTOSv2、C、Python、OpenMV、PID控制、I2C/SPI/UART/DMA/TIM

系统架构：整机功能划分为5个独立任务，由RTOS抢占式内核统一调度
任务            周期          优先级          功能
MotorTask       100ms         最高          编码器采样、增量式PID闭环
BluetoothTask   10ms          高            蓝牙透传、OpenMV数据转发
ServoTask       10ms          中            PCA9685舵机控制
ControlTask     20ms          中            双模式转换控制
DisplayTask     500ms         最低          LCD全屏信息更新

通信链路：
STM32F103手柄 → UART5(蓝牙) → STM32H750主控 ← UART4(DMA+IDLE) ↔ OpenMV Cam
                                    ↕ UART5(蓝牙转发)
                                PC网页实时监控

算法与控制部分：
电机控制： 增量式PID闭环速度控制，含死区抑制（±5mm/s）、输出平滑滤波、编码器极性自适应修正
循迹控制： 基于OpenMV色带偏移数据的PID控制器，分段P增益（±12像素分界）、D项低通滤波、弯道自适应降速、丢线超时保护
球追踪控制： PD偏移控制 + 三段距速度映射曲线，200ms数据无变化卡死检测后退恢复
舵机控制： 五次多项式缓动插补，起止零加速度 + 抓取动作状态机（前伸→闭合→收回→复位）

OpenMV视觉：通过LAB颜色空间识别网球(面积/圆度/密度筛选、距离估算)，随后将候选目标经过MobileNetV2(INT8量化TFLite)二次确认，最后通过打包6字节数据包发送。

硬件设计：
独立完成四层贴片主控板原理图与Layout，多路分级电源系统。但因为硬件部分增加了一下部件，修正了引脚占用问题，所以四层板目前和代码部分存在部分引脚冲突，比如：右电机极性引脚更换、OpenMV串口通信引脚更换。新增USB供电口并加入放浪涌设计、新增SD卡槽后续有更多功能拓展空间
