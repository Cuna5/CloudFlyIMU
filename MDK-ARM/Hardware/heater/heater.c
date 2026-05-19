/**
 * @file    heater.c
 * @brief   加热 PWM 驱动（TIM3_CH1，PC6）实现。
 *
 * - PWM 启动及频率自动修正为约 1 Hz。
 * - 占空比夹紧到 [0, 1] 并更新比较器。
 * - 急停/故障锁存状态机。
 * - 过温保护阈值检查。
 *
 * 本驱动不依赖 FreeRTOS——定时器寄存器访问足够原子，无需互斥锁。
 * Heater_ApplyDuty（位于 common/hardware.c）负责与 SensorData_t 互斥锁的耦合。
 */
#include "hardware.h"
#include "tim.h"

/* ================================================================== */
/* 模块私有状态                                                        */
/* ================================================================== */

/** Heater_Init 成功完成后置 true。 */
static bool  s_initialized   = false;

/** 故障锁存标志。由 Heater_EmergencyStop 和 Heater_OverheatCheck 置位，
 *  仅由 Heater_ClearFault 清除。 */
static bool  s_fault_latched = false;

/** 最近一次应用到 TIM3_CH1 CCR 寄存器的占空比。 */
static float s_current_duty  = 0.0f;

/* ================================================================== */
/* 1 Hz 频率自动修正常量                                               */
/* ================================================================== */

/** APB1 定时器内核时钟（STM32H743 默认 RCC 配置下为 240 MHz）。 */
#define HEATER_TIM3_CLOCK_HZ    240000000.0f

/** 可接受的 PWM 频率范围 [0.9, 1.1] Hz。 */
#define HEATER_FREQ_LO_HZ       0.9f
#define HEATER_FREQ_HI_HZ       1.1f

/** 强制 1 Hz 配置：24000 * 10000 = 2.4e8 个时钟周期。 */
#define HEATER_FORCE_PSC        (24000U - 1U)
#define HEATER_FORCE_ARR        (10000U - 1U)

/* ================================================================== */
/* 内部辅助函数                                                        */
/* ================================================================== */

/**
 * @brief 将 duty 夹紧到 [0, 1]。NaN 输入通过 out_nan 报告。
 */
static float heater_clamp_duty(float duty, bool *out_nan)
{
    /* IEEE-754：NaN 与自身不相等 */
    if (duty != duty) {
        if (out_nan) *out_nan = true;
        return 0.0f;
    }
    if (out_nan) *out_nan = false;

    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 1.0f) {
        return 1.0f;
    }
    return duty;
}

/**
 * @brief EmergencyStop 的内部版本，跳过 s_initialized 检查。
 * 由 Heater_OverheatCheck 在公共检查通过后调用，避免返回误导性状态。
 */
static void heater_emergency_stop_locked(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0u);
    s_current_duty  = 0.0f;
    s_fault_latched = true;
}

/* ================================================================== */
/* 公共 API                                                            */
/* ================================================================== */

Driver_Status Heater_Init(void)
{
    /* --- 1. 频率自动修正 -------------------------------------------- */
    const uint32_t psc_plus_1 = htim3.Init.Prescaler + 1U;
    const uint32_t arr_plus_1 = htim3.Init.Period    + 1U;

    /* 防御性检查：(PSC, ARR) = (0, 0) 时 f = 240 MHz，远超范围，由下方 if 处理 */
    bool needs_force = false;
    if (psc_plus_1 == 0U || arr_plus_1 == 0U) {
        needs_force = true;
    } else {
        const float ticks_per_period = (float)psc_plus_1 * (float)arr_plus_1;
        const float f_hz = HEATER_TIM3_CLOCK_HZ / ticks_per_period;
        if (f_hz < HEATER_FREQ_LO_HZ || f_hz > HEATER_FREQ_HI_HZ) {
            needs_force = true;
        }
    }

    if (needs_force) {
        __HAL_TIM_SET_PRESCALER (&htim3, HEATER_FORCE_PSC);
        __HAL_TIM_SET_AUTORELOAD(&htim3, HEATER_FORCE_ARR);
        /* HAL 宏写入寄存器；同步更新 Init 缓存字段，供 Heater_SetDuty 读取 */
        htim3.Init.Prescaler = HEATER_FORCE_PSC;
        htim3.Init.Period    = HEATER_FORCE_ARR;
    }

    /* --- 2. 使能输出前将占空比重置为 0 -------------------------------- */
    s_current_duty = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0u);

    /* --- 3. 启动 PWM -------------------------------------------------- */
    HAL_StatusTypeDef hs = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    if (hs != HAL_OK) {
        return Driver_MapHalStatus(hs);
    }

    /* --- 4. 标记就绪 -------------------------------------------------- */
    s_fault_latched = false;
    s_initialized   = true;
    return DRV_OK;
}

Driver_Status Heater_SetDuty(float duty)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    /* 故障锁存时拒绝操作，不写比较器 */
    if (s_fault_latched) {
        return DRV_ERR_NOT_INIT;
    }

    bool is_nan = false;
    const float clamped = heater_clamp_duty(duty, &is_nan);
    if (is_nan) {
        return DRV_ERR_PARAM;
    }

    /* CCR = duty * (ARR + 1)；duty=1 时 CCR=ARR+1，输出全开 */
    const uint32_t ccr = (uint32_t)(clamped * (float)(htim3.Init.Period + 1U));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    s_current_duty = clamped;
    return DRV_OK;
}

Driver_Status Heater_GetDuty(float *duty)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (duty == NULL) {
        return DRV_ERR_PARAM;
    }
    *duty = s_current_duty;
    return DRV_OK;
}

Driver_Status Heater_EmergencyStop(void)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    heater_emergency_stop_locked();
    return DRV_OK;
}

Driver_Status Heater_ClearFault(void)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    s_fault_latched = false;
    return DRV_OK;
}

Driver_Status Heater_OverheatCheck(float temperature_c)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (temperature_c >= HEATER_OVERHEAT_THRESHOLD_C) {
        heater_emergency_stop_locked();
        return DRV_ERR_PARAM;
    }
    return DRV_OK;
}

bool Heater_IsFaultLatched(void)
{
    /* 只读访问；Init 前查询安全（静态变量零初始化，返回 false） */
    if (!s_initialized) {
        return false;
    }
    return s_fault_latched;
}
