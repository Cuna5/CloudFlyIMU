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
