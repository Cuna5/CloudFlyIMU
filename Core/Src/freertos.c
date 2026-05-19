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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hardware.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_TASK_PERIOD_MS          1U
#define SENSOR_PRESSURE_DIV            100U
#define HEAT_TASK_PERIOD_MS            100U
#define FUSION_TASK_PERIOD_MS          1U
#define DEBUG_TASK_PERIOD_MS           1000U

/* Mahony 增益，可通过宏覆盖 */
#ifndef MAHONY_KP
#define MAHONY_KP   2.0f
#endif
#ifndef MAHONY_KI
#define MAHONY_KI   0.005f
#endif

/* 磁场可靠性判断阈值（微特斯拉） */
#ifndef MAG_NORM_MIN
#define MAG_NORM_MIN   20.0f
#endif
#ifndef MAG_NORM_MAX
#define MAG_NORM_MAX   80.0f
#endif

#ifndef HEAT_TARGET_C
#define HEAT_TARGET_C   40.0f
#endif
#ifndef HEAT_PID_KP
#define HEAT_PID_KP     5.0f
#endif
#ifndef HEAT_PID_KI
#define HEAT_PID_KI     0.1f
#endif
#ifndef HEAT_PID_KD
#define HEAT_PID_KD     0.5f
#endif
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

  /* 从 Flash 加载 PID 参数；失败时写入编译期默认值到 Flash */
  AppParams_t params = {
      .heat_kp       = HEAT_PID_KP,
      .heat_ki       = HEAT_PID_KI,
      .heat_kd       = HEAT_PID_KD,
      .heat_target_c = HEAT_TARGET_C,
  };
  Driver_Status load_st = Params_Load(&params);
  if (load_st != DRV_OK)
  {
    Debug_Log_Level(DBG_WARN, "Params_Load err=%d, saving defaults", (int)load_st);
    (void)Params_Save(&params);
  }
  else
  {
    Debug_Log_Level(DBG_INFO, "Params loaded: kp=%ld ki=%ld kd=%ld target_mC=%ld",
                    (long)App_FloatToMilli(params.heat_kp),
                    (long)App_FloatToMilli(params.heat_ki),
                    (long)App_FloatToMilli(params.heat_kd),
                    (long)App_FloatToMilli(params.heat_target_c));
  }

  PID_t heat_pid;
  PID_Init(&heat_pid, params.heat_kp, params.heat_ki, params.heat_kd,
           1.0f, 0.0f, 1.0f);
  float heat_target = params.heat_target_c;

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
          PID_Reset(&heat_pid);
          Debug_Log_Level(DBG_ERR, "Heater overheat temp_mC=%ld",
                          (long)App_FloatToMilli(snap.temperature));
        }
      }
      else if (heat_st == DRV_OK)
      {
        overheat_logged = false;
        const float dt = HEAT_TASK_PERIOD_MS / 1000.0f;
        const float duty = PID_Update(&heat_pid, heat_target,
                                      snap.temperature, dt);
        (void)Heater_ApplyDuty(duty);
      }
    }

    uint32_t bmp_errors = 0U;
    Sensor_GetErrorCounters(NULL, NULL, &bmp_errors);
    if (bmp_errors != last_bmp_errors)
    {
      last_bmp_errors = bmp_errors;
      PID_Reset(&heat_pid);
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

  Mahony_t mahony;
  Mahony_Init(&mahony, MAHONY_KP, MAHONY_KI);

  const float dt = FUSION_TASK_PERIOD_MS / 1000.0f;

  /* Infinite loop */
  for(;;)
  {
    SensorData_t s;
    if (SensorData_Get(&s) == DRV_OK)
    {
      /* 磁场模长可靠性判断 */
      float mag_norm = sqrtf(s.mx*s.mx + s.my*s.my + s.mz*s.mz);
      float mag_rel = 0.0f;
      if (mag_norm >= MAG_NORM_MIN && mag_norm <= MAG_NORM_MAX)
      {
        /* 线性映射到 [0,1]，中心点最高 */
        float center = (MAG_NORM_MIN + MAG_NORM_MAX) * 0.5f;
        float half   = (MAG_NORM_MAX - MAG_NORM_MIN) * 0.5f;
        mag_rel = 1.0f - fabsf(mag_norm - center) / half;
      }

      Mahony_Update(&mahony,
                    s.gx, s.gy, s.gz,
                    s.ax, s.ay, s.az,
                    s.mx, s.my, s.mz,
                    mag_rel, dt);

      AttitudeData_t att = {0};
      att.timestamp_ms   = s.timestamp_ms;
      att.mag_reliability = mag_rel;
      att.ekf_mode        = (mag_rel > 0.1f) ? 2U : 1U;  /* 2=9轴, 1=6轴 */
      Mahony_GetEuler(&mahony, &att.roll, &att.pitch, &att.yaw);
      (void)AttitudeData_Set(&att);
    }

    osDelay(FUSION_TASK_PERIOD_MS);
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

    AttitudeData_t att;
    if (AttitudeData_Get(&att) == DRV_OK)
    {
      Debug_Log_Level(DBG_INFO,
                      "RPY_milli=%ld,%ld,%ld MagRel_permille=%ld Mode=%u",
                      (long)App_FloatToMilli(att.roll),
                      (long)App_FloatToMilli(att.pitch),
                      (long)App_FloatToMilli(att.yaw),
                      (long)App_FloatToMilli(att.mag_reliability),
                      (unsigned)att.ekf_mode);
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

