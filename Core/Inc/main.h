/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_CS_Pin GPIO_PIN_0
#define OLED_CS_GPIO_Port GPIOC
#define OLED_MOSI_Pin GPIO_PIN_1
#define OLED_MOSI_GPIO_Port GPIOC
#define OLED_MISO_Pin GPIO_PIN_2
#define OLED_MISO_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_3
#define OLED_DC_GPIO_Port GPIOC
#define BMI_SCK_Pin GPIO_PIN_5
#define BMI_SCK_GPIO_Port GPIOA
#define BMI_MISO_Pin GPIO_PIN_6
#define BMI_MISO_GPIO_Port GPIOA
#define BMI_MOSI_Pin GPIO_PIN_7
#define BMI_MOSI_GPIO_Port GPIOA
#define BMI_ACC_CS_Pin GPIO_PIN_4
#define BMI_ACC_CS_GPIO_Port GPIOC
#define BMI_GYRO_CS_Pin GPIO_PIN_5
#define BMI_GYRO_CS_GPIO_Port GPIOC
#define OLED_SCK_Pin GPIO_PIN_10
#define OLED_SCK_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOH
#define RGB_R_Pin GPIO_PIN_10
#define RGB_R_GPIO_Port GPIOH
#define RGB_G_Pin GPIO_PIN_11
#define RGB_G_GPIO_Port GPIOH
#define RGB_B_Pin GPIO_PIN_12
#define RGB_B_GPIO_Port GPIOH
#define OLED_RST_Pin GPIO_PIN_12
#define OLED_RST_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_14
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_15
#define DEBUG_RX_GPIO_Port GPIOB
#define HEAT_PWM_Pin GPIO_PIN_6
#define HEAT_PWM_GPIO_Port GPIOC
#define IST_SCL_Pin GPIO_PIN_6
#define IST_SCL_GPIO_Port GPIOB
#define IST_SDA_Pin GPIO_PIN_7
#define IST_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
