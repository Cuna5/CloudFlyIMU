/**
 * @file    led.c
 * @brief   单色状态 LED 驱动（共阳，高电平灭灯）实现。
 *
 * LED 接 LED_GPIO_Port/LED_Pin，共阳接法：
 *   GPIO_PIN_SET   → 灭灯
 *   GPIO_PIN_RESET → 亮灯
 */
#include "hardware.h"
#include "main.h"

static bool s_initialized = false;

Driver_Status LED_Init(void)
{
    s_initialized = true;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); /* 高电平灭灯（共阳） */
    return DRV_OK;
}

void LED_On(void)
{
    if (!s_initialized) return;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void LED_Off(void)
{
    if (!s_initialized) return;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_Toggle(void)
{
    if (!s_initialized) return;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
