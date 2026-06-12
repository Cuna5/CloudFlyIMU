/**
 * @file    led.h
 * @brief   单色状态 LED 驱动（共阳，高电平灭灯）公共 API。
 *
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 * 实现位于 led/led.c。
 */
/* Do not include directly — use hardware.h */
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化 LED GPIO，初始状态为灭灯。 */
Driver_Status LED_Init(void);

/** @brief 点亮 LED（低电平有效）。 */
void LED_On(void);

/** @brief 熄灭 LED（高电平）。 */
void LED_Off(void);

/** @brief 翻转 LED 状态。 */
void LED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
