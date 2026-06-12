/**
 * @file    rgb_led.c
 * @brief   RGB LED PWM 驱动（TIM5_CH1/2/3）实现。
 *
 * TIM5 ARR=255，三路 PWM 分别对应 R/G/B 通道。
 * 共阳接法（RGB_COMMON_ANODE=1）：CCR 越大亮度越低，内部取反。
 * 共阴接法（RGB_COMMON_ANODE=0）：CCR 越大亮度越高，直接映射。
 */
#include "hardware.h"
#include "tim.h"

/* TIM5 的 ARR 值，与 CubeMX 配置保持一致 */
#define RGB_TIM_ARR     255U

/* 根据实际接线修改此宏：共阳=1，共阴=0 */
#define RGB_COMMON_ANODE  1

#if RGB_COMMON_ANODE
#define RGB_MAP(val)  (RGB_TIM_ARR - (uint32_t)(val))
#else
#define RGB_MAP(val)  ((uint32_t)(val))
#endif

static bool s_initialized = false;

/* ------------------------------------------------------------------ */
/* 内部辅助                                                             */
/* ------------------------------------------------------------------ */

/** @brief 同步更新三路 CCR 寄存器。 */
static void set_channels(uint8_t r, uint8_t g, uint8_t b)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, RGB_MAP(r));
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, RGB_MAP(g));
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, RGB_MAP(b));
}

/* ------------------------------------------------------------------ */
/* 初始化                                                               */
/* ------------------------------------------------------------------ */

Driver_Status RGBLED_Init(void)
{
    HAL_StatusTypeDef ret;

    ret = HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    if (ret != HAL_OK) return Driver_MapHalStatus(ret);

    ret = HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    if (ret != HAL_OK) return Driver_MapHalStatus(ret);

    ret = HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    if (ret != HAL_OK) return Driver_MapHalStatus(ret);

    s_initialized = true;
    set_channels(0, 0, 0); /* 初始关灯 */
    return DRV_OK;
}

/* ------------------------------------------------------------------ */
/* 基础控制                                                             */
/* ------------------------------------------------------------------ */

Driver_Status RGBLED_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_initialized) return DRV_ERR_NOT_INIT;
    set_channels(r, g, b);
    return DRV_OK;
}

Driver_Status RGBLED_Off(void)
{
    if (!s_initialized) return DRV_ERR_NOT_INIT;
    set_channels(0, 0, 0);
    return DRV_OK;
}

/* ------------------------------------------------------------------ */
/* 预设颜色                                                             */
/* ------------------------------------------------------------------ */

Driver_Status RGBLED_SetRed(void)     { return RGBLED_SetColor(255,   0,   0); }
Driver_Status RGBLED_SetGreen(void)   { return RGBLED_SetColor(  0, 255,   0); }
Driver_Status RGBLED_SetBlue(void)    { return RGBLED_SetColor(  0,   0, 255); }
Driver_Status RGBLED_SetWhite(void)   { return RGBLED_SetColor(255, 255, 255); }
Driver_Status RGBLED_SetYellow(void)  { return RGBLED_SetColor(255, 255,   0); }
Driver_Status RGBLED_SetCyan(void)    { return RGBLED_SetColor(  0, 255, 255); }
Driver_Status RGBLED_SetMagenta(void) { return RGBLED_SetColor(255,   0, 255); }
