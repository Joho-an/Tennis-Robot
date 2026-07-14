#include "servo.h"
#include <math.h>

// 实际角度偏离值--修正
int16_t base_angle = BASE_INIT_ANGLE;
int16_t arm_angle  = ARM_INIT_ANGLE;
int16_t claw_angle = CLAW_OPEN_ANGLE;

// 目标角度
int16_t base_target = BASE_INIT_ANGLE;
int16_t arm_target  = ARM_INIT_ANGLE;
int16_t claw_target = CLAW_OPEN_ANGLE;

// 舵机初始化状态 (供外部检查)
uint8_t servo_init_ok = 0;

// 内部变量：抓取状态机
static GrabState_t grab_step = GRAB_IDLE;
static uint32_t state_timer = 0;
static uint8_t  grab_distance = 0;  // 触发抓取时的球距离, 用于查表确定角度

/* --- 缓动控制结构体 --- */
typedef struct {
    int16_t last_target;     // 记录上次的目标位
    float start_angle;       // 运动起点的实际角度
    float current_float;     // 运动过程中的高精度当前角度
    uint32_t start_time;     // 运动开始的时间限制
    uint32_t duration;       // 运动期望耗时
    uint16_t speed_ms_deg;   // ms/度 的速度基准
} ServoMotion_t;

static ServoMotion_t sm_base = {BASE_INIT_ANGLE, BASE_INIT_ANGLE, BASE_INIT_ANGLE, 0, 0, SERVO_SPEED_MS_DEG_BASE};
static ServoMotion_t sm_arm  = {ARM_INIT_ANGLE, ARM_INIT_ANGLE, ARM_INIT_ANGLE, 0, 0, SERVO_SPEED_MS_DEG_ARM};
static ServoMotion_t sm_claw = {CLAW_OPEN_ANGLE, CLAW_OPEN_ANGLE, CLAW_OPEN_ANGLE, 0, 0, SERVO_SPEED_MS_DEG_CLAW};


/* ================= 底层 I2C 驱动 ================= */

static HAL_StatusTypeDef Servo_I2C_WriteReg(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c4, SERVO_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef Servo_I2C_ReadReg(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(&hi2c4, SERVO_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef Servo_I2C_SetPWM(uint8_t num, uint16_t on, uint16_t off) {
    uint8_t reg_base = SERVO_REG_LED0_ON_L + 4 * num;
    HAL_StatusTypeDef sts;
    uint8_t retry;

    for (retry = 0; retry < 3; retry++) {
        sts = Servo_I2C_WriteReg(reg_base, on & 0xFF);
        if (sts != HAL_OK) continue;
        sts = Servo_I2C_WriteReg(reg_base + 1, on >> 8);
        if (sts != HAL_OK) continue;
        sts = Servo_I2C_WriteReg(reg_base + 2, off & 0xFF);
        if (sts != HAL_OK) continue;
        sts = Servo_I2C_WriteReg(reg_base + 3, off >> 8);
        if (sts != HAL_OK) continue;
        break;  // 成功
    }
    return sts;
}

/* ================= 舵机应用层逻辑 ================= */

/**
 * @brief 初始化舵机控制板 并在上电瞬间进入安全姿态
 * @param freq_hz 目标 PWM 频率 (典型值 50)
 *
 * PCA9685 初始化标准流程 (遵循 NXP 数据手册 §7.3):
 *   1. 配置 MODE2 (OUTDRV / OUTNE 等输出特性)
 *   2. 写 MODE1 进入 SLEEP
 *   3. 写 PRESCALE
 *   4. 写 MODE1 退出 SLEEP
 *   5. 等待 ≥500μs 振荡器稳定
 *   6. 写 MODE1 使能 RESTART + 自动增量
 */
void Servo_Init(uint16_t freq_hz) {
    HAL_StatusTypeDef sts;
    uint8_t reg_val;
    servo_init_ok = 0;

    // --- 第0步：软件复位 (SWRST, bit7) ---
    // MODE2 必须在 MODE1 之前配置
    sts = Servo_I2C_WriteReg(SERVO_REG_MODE2, 0x04); // OUTDRV=1 推挽输出
    if (sts != HAL_OK) return;

    // --- 第1步：进入 SLEEP (bit4) 以允许修改预分频 ---
    // 内部 25MHz 振荡器，允许改 Prescale
    sts = Servo_I2C_WriteReg(SERVO_REG_MODE1, 0x10); // SLEEP=1
    if (sts != HAL_OK) return;
    HAL_Delay(1);

    // --- 第2步：设置预分频器 ---
    // prescale = round(osc_clock / (4096 * update_rate)) - 1
    // osc_clock = 25MHz, update_rate = 50Hz → prescale = 121
    uint8_t prescale = (uint8_t)(25000000UL / (4096UL * (uint32_t)freq_hz) - 1);
    sts = Servo_I2C_WriteReg(SERVO_REG_PRESCALE, prescale);
    if (sts != HAL_OK) return;

    // --- 第3步：退出 SLEEP，唤醒内部振荡器 ---
    sts = Servo_I2C_WriteReg(SERVO_REG_MODE1, 0x00);
    if (sts != HAL_OK) return;
    HAL_Delay(1); // 数据手册要求 ≥500μs，这里给 1ms 安全余量

    // --- 第4步：使能 RESTART + 自动增量 ---
    // RESTART(bit7)=1 清零内部计数器，AI(bit5)=1 允许连续写寄存器
    sts = Servo_I2C_WriteReg(SERVO_REG_MODE1, 0xA0);
    if (sts != HAL_OK) return;
    HAL_Delay(1);

    // --- 第5步：回读 MODE1 验证通信正常 ---
    sts = Servo_I2C_ReadReg(SERVO_REG_MODE1, &reg_val);
    if (sts != HAL_OK || reg_val == 0x00 || reg_val == 0xFF) {
        // 通信失败或读到异常值 (全0/全1说明I2C器件无应答)
        return;
    }

    // --- 第6步：输出初始安全角度 ---
    servo_init_ok = 1; // 必须在 SetAngleDirect 之前置位，否则会被守卫拦截

    base_angle = BASE_INIT_ANGLE;
    arm_angle  = ARM_INIT_ANGLE;
    claw_angle = CLAW_OPEN_ANGLE;

    base_target = BASE_INIT_ANGLE;
    arm_target  = ARM_INIT_ANGLE;
    claw_target = CLAW_OPEN_ANGLE;

    Servo_SetAngleDirect(BASE_CH, base_angle);
    Servo_SetAngleDirect(ARM_CH, arm_angle);
    Servo_SetAngleDirect(CLAW_CH, claw_angle);
}

/**
 * @brief 将角度映射到 I2C 寄存器值并输出
 */
void Servo_SetAngleDirect(uint8_t Channel, int16_t Angle) {
    if (!servo_init_ok) return;

    // 包含物理调整偏置补偿
    int16_t real_angle = Angle;
    if (Channel == BASE_CH) real_angle += BASE_OFFSET;
    else if (Channel == ARM_CH) real_angle += ARM_OFFSET;
    else if (Channel == CLAW_CH) real_angle += CLAW_OFFSET;

    // 限幅处理
    if (real_angle < 0)   real_angle = 0;
    if (real_angle > 180) real_angle = 180;

    // 换算公式：PWM = MIN + (Angle/180) * (MAX - MIN)
    int32_t pwm_range = SERVO_MAX_PWM - SERVO_MIN_PWM;
    uint16_t off_val = (uint16_t)(SERVO_MIN_PWM + (real_angle * pwm_range) / 180);

    Servo_I2C_SetPWM(Channel, 0, off_val);
}

/**
 * @brief 设置新目标角度
 */
void Servo_MoveTo(int16_t b_target, int16_t a_target, int16_t c_target) {
    base_target = b_target;
    arm_target  = a_target;
    claw_target = c_target;
}

/**
 * @brief 夹爪瞬动 (跳过缓动插补，直接到位)
 */
void Servo_MoveClawInstant(int16_t angle) {
    if (!servo_init_ok) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    claw_target = angle;
    claw_angle  = angle;

    sm_claw.last_target   = angle;
    sm_claw.start_angle   = (float)angle;
    sm_claw.current_float = (float)angle;

    Servo_SetAngleDirect(CLAW_CH, angle);
}

/**
 * @brief 三次多项式单轴插补运算 (自适应策略)
 *
 * 解决蓝牙连续按键时的抖动震颤:
 *   - 运动已完成 → 重置曲线，以当前位置为起点开始新缓动
 *   - 运动进行中 && 目标变化 ≤ 15° → 仅顺滑终点，不重置曲线
 *   - 运动进行中 && 目标变化 > 15° → 重置曲线重新规划
 *   - 最小运动时长 60ms，防止 1° 小步进退化为瞬时阶跃
 */
static uint8_t Update_Servo_Motion(ServoMotion_t *sm, int16_t *current_int, int16_t target, uint32_t now) {
    uint8_t moved = 0;

    if (target != sm->last_target) {
        int16_t delta = target - sm->last_target;
        int16_t abs_delta = delta < 0 ? -delta : delta;
        int16_t curr = (int16_t)(sm->current_float + 0.5f);
        uint8_t motion_complete = (curr == sm->last_target);

        if (motion_complete || abs_delta > 15) {
            sm->start_angle = sm->current_float;
            sm->start_time = now;

            float diff = (float)target - sm->start_angle;
            if (diff < 0) diff = -diff;
            sm->duration = (uint32_t)(diff * sm->speed_ms_deg);
            if (sm->duration < 60) sm->duration = 60;
        }
        sm->last_target = target;
    }

    if ((int16_t)sm->current_float != target || sm->current_float != (float)target) {
        uint32_t elapsed = now - sm->start_time;
        if (elapsed >= sm->duration) {
            sm->current_float = (float)target;
        } else {
            float t = (float)elapsed / (float)sm->duration;
            // 五次多项式: t³(6t² - 15t + 10), 起点和终点加速度均为0, 极其平滑
            float ease = t * t * t * (6.0f * t * t - 15.0f * t + 10.0f);
            sm->current_float = sm->start_angle + ((float)target - sm->start_angle) * ease;
        }

        int16_t final_angle = (int16_t)(sm->current_float + 0.5f);
        if (*current_int != final_angle) {
            *current_int = final_angle;
            moved = 1;
        }
    }
    return moved;
}

/**
 * @brief 非阻塞平滑运动核心 (在 main while 中直接调用)
 * 10ms 执行一次步进更新--五次多项式平滑插补 (100Hz)
 */
void Servo_Handle_Tick(void) {
    if (!servo_init_ok) return;

    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 10) return; // 100Hz 更新率, 步进更细腻
    last_tick = now;

    // base 使用瞬动模式, 跳过缓动直接输出
    static int16_t last_base_target = BASE_INIT_ANGLE;
    if (base_target != last_base_target) {
        last_base_target = base_target;
        base_angle = base_target;
        sm_base.last_target   = base_target;
        sm_base.start_angle   = (float)base_target;
        sm_base.current_float = (float)base_target;
        Servo_SetAngleDirect(BASE_CH, base_target);
    }

    // claw 同样使用瞬动模式, 与 Servo_MoveClawInstant 保持一致
    static int16_t last_claw_target = CLAW_OPEN_ANGLE;
    if (claw_target != last_claw_target) {
        last_claw_target = claw_target;
        claw_angle = claw_target;
        sm_claw.last_target   = claw_target;
        sm_claw.start_angle   = (float)claw_target;
        sm_claw.current_float = (float)claw_target;
        Servo_SetAngleDirect(CLAW_CH, claw_target);
    }

    uint8_t moved_arm = Update_Servo_Motion(&sm_arm, &arm_angle, arm_target, now);

    // 输出 arm
    if (moved_arm) Servo_SetAngleDirect(ARM_CH, arm_angle);
}

/**
 * @brief 触发抓取流程的接口
 * @param distance 触发时球的距离 (cm), 用于查表确定 base/arm 前伸角度
 */
void Servo_StartGrabCycle(uint8_t distance) {
    if (grab_step == GRAB_IDLE) {
        grab_distance = distance;
        grab_step = GRAB_REACH;
    }
}

uint8_t Servo_IsGrabDone(void) {
    return (grab_step == GRAB_IDLE) ? 1 : 0;
}

/**
 * @brief 机械臂抓取逻辑状态机
 * 该函数应在 main 的 while(1) 中紧随 Servo_Handle_Tick 调用
 *
 * 修复要点:
 *   1. 每个状态只在首次进入 (enter) 时设定一次目标角度，避免每帧覆盖导致缓动状态机混乱
 *   2. 夹爪的闭合/张开动作在子阶段中单独控制，不与底座/中臂的目标设定混在一起
 */
void Servo_Task_GrabProcess(void) {
    if (!servo_init_ok) return;
    if (grab_step == GRAB_IDLE) return;

    // 内部状态子阶段 (用于区分 "运动到位前" vs "等待1s期间")
    // 0 = 正在运动到位; 1 = 到位后等待中
    static uint8_t reach_sub_phase = 0;
    static uint8_t back_sub_phase  = 0;
    static uint8_t first_enter_reach = 1;  // GRAB_REACH 首次进入标志
    static uint8_t first_enter_back  = 1;  // GRAB_BACK 首次进入标志

    // ============================================================
    //  阶段1: GRAB_REACH → 中臂前伸 + 底座前伸, 夹爪张开接近球
    //  子阶段0: 底座+中臂向前运动到位 → 等待1s → 夹爪闭合 → 再等1s稳定
    // ============================================================
    if (grab_step == GRAB_REACH) {
        if (first_enter_reach) {
            first_enter_reach = 0;
            reach_sub_phase = 0;
        }

        if (reach_sub_phase == 0) {
            // 根据触发距离线性插值, 确定底座和中臂前伸角度
            // 距离越小(球越近), base↑前伸 / arm↑前伸
            //
            // 映射 (可抓取距离 13~15cm):
            //   distance=15 → base=0,  arm=170
            //   distance=13 → base=5,  arm=172
            //   ≥15 → 钳位 base=0, arm=170 (兜底)
            int16_t reach_base, reach_arm;
            int16_t dist = (int16_t)grab_distance;
            // 映射 (可抓取距离 14~16cm):
            //   distance=16 → base=5,  arm=165
            //   distance=15 → base=0,  arm=165
            //   distance=14 → base=(5*(15-14))=5->0 映射, arm=175
            if (dist >= 16) {
                reach_base = 0;    // 16cm 对应 0° 
                reach_arm  = 165;  // 16cm 对应 165°
            } else if (dist >= 15) {
                reach_base = 5;    // 15cm 对应 5° (不变)
                reach_arm  = 173;  // 15cm 对应 173° (不变)
            } else if (dist >= 14) {
                reach_base = (int16_t)(5 * (15 - dist));
                reach_arm  = 176;  // 14cm 对应 176° (不变)
            } else {    
                // dist < 14: 钳位为最大前伸保护
                reach_base = 10;
                reach_arm  = 180;
            }

            base_target = reach_base;
            arm_target  = reach_arm;
            claw_target = CLAW_RELEASE_ANGLE;

            int16_t db = base_angle - base_target; if (db < 0) db = -db;
            int16_t da = arm_angle  - arm_target;  if (da < 0) da = -da;
            if (db <= 1 && da <= 1) {
                // 底座+中臂已到位 → 先等待1s稳定, 再闭合夹爪
                reach_sub_phase = 1;
                state_timer = HAL_GetTick();
                // 不立即闭合夹爪, 先等待
            }
        } else if (reach_sub_phase == 1) {
            // 子阶段1: 等待 1s 让臂稳定, 然后夹爪闭合
            if (HAL_GetTick() - state_timer >= 1000) {
                reach_sub_phase = 2;
                state_timer = HAL_GetTick();
                // 使用瞬动接口，跳过缓动直接输出闭合 PWM
                Servo_MoveClawInstant(CLAW_GRAB_ANGLE);
            }
        } else {
            // 子阶段2: 夹爪闭合后等待 1s 确保抓稳
            if (HAL_GetTick() - state_timer >= 1000) {
                reach_sub_phase = 0;
                state_timer = 0;
                grab_step = GRAB_BACK;
                first_enter_back = 1;
            }
        }
    }
    // ============================================================
    //  阶段2: GRAB_BACK → 中臂收回 + 底座收回, 夹爪保持闭合
    //  子阶段0: 底座+中臂向后运动到位
    //  子阶段1: 到位后等待 1s, 期间夹爪张开释放球
    // ============================================================
    else if (grab_step == GRAB_BACK) {
        if (first_enter_back) {
            first_enter_back = 0;
            back_sub_phase = 0;
        }

        if (back_sub_phase == 0) {
            // 运动中: 底座和中臂收回，夹爪保持闭合 (抓着球)
            base_target = BACK_BASE_TARGET;
            arm_target  = BACK_ARM_TARGET;
            claw_target = CLAW_GRAB_ANGLE;

            int16_t db = base_angle - base_target; if (db < 0) db = -db;
            int16_t da = arm_angle  - arm_target;  if (da < 0) da = -da;
            if (db <= 1 && da <= 1) {
                // 底座+中臂已到位 → 进入子阶段1，夹爪立即张开释放
                back_sub_phase = 1;
                state_timer = HAL_GetTick();
                Servo_MoveClawInstant(CLAW_RELEASE_ANGLE);
            }
        } else {
            // 子阶段1: 等待 1s, 夹爪保持张开
            if (HAL_GetTick() - state_timer >= 1000) {
                back_sub_phase = 0;
                state_timer = 0;
                grab_step = GRAB_RESET;
            }
        }
    }
    // ============================================================
    //  阶段3: GRAB_RESET → 所有舵机回到初始安全姿态
    // ============================================================
    else if (grab_step == GRAB_RESET) {
        base_target = BASE_INIT_ANGLE;
        arm_target  = ARM_INIT_ANGLE;
        claw_target = CLAW_OPEN_ANGLE;

        int16_t db = base_angle - base_target; if (db < 0) db = -db;
        int16_t da = arm_angle  - arm_target;  if (da < 0) da = -da;
        int16_t dc = claw_angle - claw_target; if (dc < 0) dc = -dc;
        if (db <= 1 && da <= 1 && dc <= 1) {
            first_enter_reach = 1;  // 为下次抓取准备
            first_enter_back  = 1;
            grab_step = GRAB_IDLE;
        }
    }
}
