/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hardware.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_TASK_PERIOD_MS          1U
#define SENSOR_PRESSURE_DIV            100U
#define HEAT_TASK_PERIOD_MS            100U
#define DEBUG_TASK_PERIOD_MS           1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for HeatTask */
osThreadId_t HeatTaskHandle;
const osThreadAttr_t HeatTask_attributes = {
  .name = "HeatTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FusionTask */
osThreadId_t FusionTaskHandle;
const osThreadAttr_t FusionTask_attributes = {
  .name = "FusionTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I2CMutex */
osMutexId_t I2CMutexHandle;
const osMutexAttr_t I2CMutex_attributes = {
  .name = "I2CMutex"
};
/* Definitions for SPIMutex */
osMutexId_t SPIMutexHandle;
const osMutexAttr_t SPIMutex_attributes = {
  .name = "SPIMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static int32_t App_FloatToMilli(float value);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of I2CMutex */
  I2CMutexHandle = osMutexNew(&I2CMutex_attributes);

  /* creation of SPIMutex */
  SPIMutexHandle = osMutexNew(&SPIMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartTask02, NULL, &SensorTask_attributes);

  /* creation of HeatTask */
  HeatTaskHandle = osThreadNew(StartTask03, NULL, &HeatTask_attributes);

  /* creation of FusionTask */
  FusionTaskHandle = osThreadNew(StartTask04, NULL, &FusionTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartTask05, NULL, &DebugTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  (void)argument;

  SensorData_t sample = {0};
  uint32_t pressure_div = 0U;
  uint32_t last_warn_ms = 0U;
  bool first_warn = true;

  /* Infinite loop */
  for(;;)
  {
    Driver_Status st = Sensor_SampleOnce(&sample);

    pressure_div++;
    if (pressure_div >= SENSOR_PRESSURE_DIV)
    {
      pressure_div = 0U;
      float pressure_pa = sample.pressure;
      Driver_Status p_st = BMP280_ReadPressure(&pressure_pa);
      if (p_st == DRV_OK)
      {
        sample.pressure = pressure_pa;
      }
      else if (st == DRV_OK)
      {
        st = p_st;
      }
    }

    (void)Heater_GetDuty(&sample.pwm_duty);
    (void)SensorData_Set(&sample);

    if (st != DRV_OK)
    {
      uint32_t now_ms = HAL_GetTick();
      if (first_warn || ((now_ms - last_warn_ms) >= 1000U))
      {
        first_warn = false;
        last_warn_ms = now_ms;
        Debug_Log_Level(DBG_WARN, "Sensor sample err=%d", (int)st);
      }
    }

    osDelay(SENSOR_TASK_PERIOD_MS);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the HeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  (void)argument;

  uint32_t last_bmp_errors = 0U;
  bool overheat_logged = false;

  /* Infinite loop */
  for(;;)
  {
    SensorData_t snap;
    if (SensorData_Get(&snap) == DRV_OK)
    {
      Driver_Status heat_st = Heater_OverheatCheck(snap.temperature);
      if (heat_st == DRV_ERR_PARAM)
      {
        if (!overheat_logged)
        {
          overheat_logged = true;
          Debug_Log_Level(DBG_ERR, "Heater overheat temp_mC=%ld",
                          (long)App_FloatToMilli(snap.temperature));
        }
      }
      else if (heat_st == DRV_OK)
      {
        overheat_logged = false;
      }
    }

    uint32_t bmp_errors = 0U;
    Sensor_GetErrorCounters(NULL, NULL, &bmp_errors);
    if (bmp_errors != last_bmp_errors)
    {
      last_bmp_errors = bmp_errors;
      (void)Heater_EmergencyStop();
      Debug_Log_Level(DBG_ERR, "BMP280 read fault, heater stopped");
    }

    osDelay(HEAT_TASK_PERIOD_MS);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the FusionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    SensorData_t snap;
    uint32_t bmi_errors = 0U;
    uint32_t ist_errors = 0U;
    uint32_t bmp_errors = 0U;

    if (SensorData_Get(&snap) == DRV_OK)
    {
      Sensor_GetErrorCounters(&bmi_errors, &ist_errors, &bmp_errors);

      Debug_Log_Level(DBG_INFO,
                      "A_milli=%ld,%ld,%ld G_milli=%ld,%ld,%ld",
                      (long)App_FloatToMilli(snap.ax),
                      (long)App_FloatToMilli(snap.ay),
                      (long)App_FloatToMilli(snap.az),
                      (long)App_FloatToMilli(snap.gx),
                      (long)App_FloatToMilli(snap.gy),
                      (long)App_FloatToMilli(snap.gz));

      Debug_Log_Level(DBG_INFO,
                      "M_milli=%ld,%ld,%ld T_mC=%ld P_Pa=%ld PWM_permille=%ld ERR=%lu,%lu,%lu",
                      (long)App_FloatToMilli(snap.mx),
                      (long)App_FloatToMilli(snap.my),
                      (long)App_FloatToMilli(snap.mz),
                      (long)App_FloatToMilli(snap.temperature),
                      (long)(App_FloatToMilli(snap.pressure) / 1000),
                      (long)App_FloatToMilli(snap.pwm_duty),
                      (unsigned long)bmi_errors,
                      (unsigned long)ist_errors,
                      (unsigned long)bmp_errors);
    }

    osDelay(DEBUG_TASK_PERIOD_MS);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static int32_t App_FloatToMilli(float value)
{
  if (value != value)
  {
    return 0;
  }
  if (value > 2147483.0f)
  {
    return INT32_MAX;
  }
  if (value < -2147483.0f)
  {
    return INT32_MIN;
  }
  return (int32_t)(value * 1000.0f);
}

/* USER CODE END Application */

