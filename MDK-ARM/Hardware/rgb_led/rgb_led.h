/**
 * @file    rgb_led.h
 * @brief   RGB LED PWM 驱动（TIM5_CH1/2/3）公共 API。
 *
 * 通过 TIM5 三路 PWM 独立控制 R/G/B 通道亮度（0-255）。
 * 支持共阳/共阴接法，由 rgb_led.c 中 RGB_COMMON_ANODE 宏配置。
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 */
/* Do not include directly — use hardware.h */
#ifndef RGB_LED_H
#define RGB_LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 初始化                                                               */
/* ------------------------------------------------------------------ */

/** @brief 启动 TIM5 三路 PWM，初始关灯。 */
Driver_Status RGBLED_Init(void);

/* ------------------------------------------------------------------ */
/* 基础控制                                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief 设置 RGB 颜色（各通道 0-255）。
 *
 * 共阳接法时内部取反（CCR = ARR - val），共阴接法直接映射。
 */
Driver_Status RGBLED_SetColor(uint8_t r, uint8_t g, uint8_t b);

/** @brief 关闭 RGB 灯（三通道全 0）。 */
Driver_Status RGBLED_Off(void);

/* ------------------------------------------------------------------ */
/* 预设颜色                                                             */
/* ------------------------------------------------------------------ */
Driver_Status RGBLED_SetRed(void);
Driver_Status RGBLED_SetGreen(void);
Driver_Status RGBLED_SetBlue(void);
Driver_Status RGBLED_SetWhite(void);
Driver_Status RGBLED_SetYellow(void);
Driver_Status RGBLED_SetCyan(void);
Driver_Status RGBLED_SetMagenta(void);

#ifdef __cplusplus
}
#endif

#endif /* RGB_LED_H */
