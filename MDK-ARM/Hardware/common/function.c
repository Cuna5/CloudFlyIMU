#include "function.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <math.h>

#define SENSOR_TASK_PERIOD_MS          5U
#define SENSOR_PRESSURE_DIV            100U
#define MAHONY_TASK_PERIOD_MS          5U
#define OLED_TASK_PERIOD_MS            50U
#define OLED_INIT_RETRY_MS             1000U
#define OLED_ATTITUDE_STALE_MS         500U
#define OLED_RAD_TO_DEG                57.2957795f

#ifndef MAHONY_KP
#define MAHONY_KP   2.0f
#endif
#ifndef MAHONY_KI
#define MAHONY_KI   0.005f
#endif
#ifndef MAG_NORM_MIN
#define MAG_NORM_MIN   20.0f
#endif
#ifndef MAG_NORM_MAX
#define MAG_NORM_MAX   80.0f
#endif
#ifndef MAG_RELIABILITY_MODE_THRESHOLD
#define MAG_RELIABILITY_MODE_THRESHOLD   0.10f
#endif

static float Mahony_CalcDt(uint32_t now_ms, uint32_t *last_ms)
{
  uint32_t elapsed_ms = now_ms - *last_ms;
  *last_ms = now_ms;
  if ((elapsed_ms == 0U) || (elapsed_ms > 100U))
    elapsed_ms = MAHONY_TASK_PERIOD_MS;
  return (float)elapsed_ms * 0.001f;
}

static float Mahony_Clamp01(float v)
{
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static float Mahony_CalcMagReliability(const SensorData_t *s)
{
  float n = sqrtf(s->mx * s->mx + s->my * s->my + s->mz * s->mz);
  if (n <= 1.0e-6f)     return 0.0f;
  if (n < MAG_NORM_MIN) return Mahony_Clamp01(n / MAG_NORM_MIN);
  if (n > MAG_NORM_MAX) return Mahony_Clamp01(MAG_NORM_MAX / n);
  return 1.0f;
}

static bool Oled_IsFinite(float v)
{
  return (v == v) && (v > -1.0e30f) && (v < 1.0e30f);
}

static uint32_t Cpu_GetUsagePercent(void)
{
  TaskStatus_t tasks[12];
  uint32_t total_time;
  UBaseType_t n = uxTaskGetSystemState(tasks, 12U, &total_time);
  if ((n == 0U) || (total_time == 0U)) return 0U;

  uint32_t idle_time = 0U;
  bool idle_found = false;
  for (UBaseType_t i = 0U; i < n; i++)
  {
    if (tasks[i].xHandle == xTaskGetIdleTaskHandle())
    {
      idle_time = tasks[i].ulRunTimeCounter;
      idle_found = true;
      break;
    }
  }
  if (!idle_found) return 0U;

  static bool prev_valid = false;
  static uint32_t prev_total_time = 0U;
  static uint32_t prev_idle_time = 0U;

  if (!prev_valid)
  {
    prev_valid = true;
    prev_total_time = total_time;
    prev_idle_time = idle_time;
    return 0U;
  }

  uint32_t total_delta = total_time - prev_total_time;
  uint32_t idle_delta = idle_time - prev_idle_time;
  prev_total_time = total_time;
  prev_idle_time = idle_time;

  if (total_delta == 0U) return 0U;
  if (idle_delta > total_delta) idle_delta = total_delta;

  uint32_t busy_delta = total_delta - idle_delta;
  uint32_t busy_pct = (uint32_t)((((uint64_t)busy_delta * 100ULL) +
                                  ((uint64_t)total_delta / 2ULL)) /
                                 (uint64_t)total_delta);
  if (busy_pct > 100U)
  {
    busy_pct = 100U;
  }

  return busy_pct;
}

static int32_t Oled_RadToTenthsDeg(float rad)
{
  if (!Oled_IsFinite(rad)) return 0;
  float t = rad * OLED_RAD_TO_DEG * 10.0f;
  return (t >= 0.0f) ? (int32_t)(t + 0.5f) : (int32_t)(t - 0.5f);
}

static uint32_t Oled_ReliabilityPercent(float v)
{
  if (!Oled_IsFinite(v) || (v <= 0.0f)) return 0U;
  if (v >= 1.0f) return 100U;
  return (uint32_t)((v * 100.0f) + 0.5f);
}

static void Oled_PrintAngle(uint8_t y, const char *label, float rad)
{
  int32_t tenths = Oled_RadToTenthsDeg(rad);
  char sign = '+';
  if (tenths < 0) { sign = '-'; tenths = -tenths; }
  (void)OLED_SetCursor(0U, y);
  (void)OLED_Printf(OLED_COLOR_ON, "%s:%c%3ld.%01ld deg",
                    label, sign, (long)(tenths / 10), (long)(tenths % 10));
}

static Driver_Status Oled_RenderAttitude(const AttitudeData_t *att, bool stale)
{
  Driver_Status st = OLED_Fill(OLED_COLOR_OFF);
  if (st != DRV_OK) return st;

  st = OLED_SetCursor(0U, 0U);
  if (st != DRV_OK) return st;
  st = OLED_WriteString("CloudFlyIMU ATT", OLED_COLOR_ON);
  if (st != DRV_OK) return st;

  if (stale)
  {
    st = OLED_SetCursor(0U, 24U);
    if (st != DRV_OK) return st;
    st = OLED_WriteString("Waiting attitude", OLED_COLOR_ON);
    if (st != DRV_OK) return st;
    st = OLED_SetCursor(0U, 40U);
    if (st != DRV_OK) return st;
    st = OLED_WriteString("Mahony not ready", OLED_COLOR_ON);
    if (st != DRV_OK) return st;
  }
  else
  {
    Oled_PrintAngle(16U, "R", att->roll);
    Oled_PrintAngle(24U, "P", att->pitch);
    Oled_PrintAngle(32U, "Y", att->yaw);

    uint32_t mag_pct = Oled_ReliabilityPercent(att->mag_reliability);
    const char *mode = (att->ekf_mode >= 2U) ? "9AX" :
                       (att->ekf_mode == 1U) ? "6AX" : "IDLE";
    st = OLED_SetCursor(0U, 48U);
    if (st != DRV_OK) return st;
    st = OLED_Printf(OLED_COLOR_ON, "MAG:%3lu%% MODE:%s",
                     (unsigned long)mag_pct, mode);
    if (st != DRV_OK) return st;
  }

  st = OLED_SetCursor(0U, 56U);
  if (st != DRV_OK) return st;
  st = OLED_Printf(OLED_COLOR_ON, "CPU:%3lu%%", (unsigned long)Cpu_GetUsagePercent());
  if (st != DRV_OK) return st;

  return OLED_Update();
}

/* ------------------------------------------------------------------ */

void Task_Sensor(void)
{
  SensorData_t sample = {0};
  uint32_t pressure_div = 0U;
  uint32_t last_warn_ms = 0U;
  bool first_warn = true;

  for (;;)
  {
    Driver_Status st = Sensor_SampleOnce(&sample);

    pressure_div++;
    if (pressure_div >= SENSOR_PRESSURE_DIV)
    {
      pressure_div = 0U;
      float pressure_pa = sample.pressure;
      Driver_Status p_st = BMP280_ReadPressure(&pressure_pa);
      if (p_st == DRV_OK)
        sample.pressure = pressure_pa;
      else if (st == DRV_OK)
        st = p_st;
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
}

void Task_Heat(void)
{
  (void)Heater_SetDuty(0.5f);

  for (;;)
  {
    osDelay(100U);
  }
}

void Task_MahonyFusion(void)
{
  Mahony_t mahony;
  uint32_t last_update_ms = HAL_GetTick();
  uint32_t last_warn_ms = 0U;
  bool first_warn = true;

  Mahony_Init(&mahony, MAHONY_KP, MAHONY_KI);

  for (;;)
  {
    SensorData_t sensor;
    Driver_Status st = SensorData_Get(&sensor);
    if (st == DRV_OK)
    {
      uint32_t now_ms = HAL_GetTick();
      float dt = Mahony_CalcDt(now_ms, &last_update_ms);
      float mag_rel = Mahony_CalcMagReliability(&sensor);

      Mahony_Update(&mahony,
                    sensor.gx, sensor.gy, sensor.gz,
                    sensor.ax, sensor.ay, sensor.az,
                    sensor.mx, sensor.my, sensor.mz,
                    mag_rel, dt);

      AttitudeData_t attitude = {0};
      attitude.timestamp_ms = now_ms;
      Mahony_GetEuler(&mahony, &attitude.roll, &attitude.pitch, &attitude.yaw);
      attitude.bgx = -mahony.ix;
      attitude.bgy = -mahony.iy;
      attitude.bgz = -mahony.iz;
      attitude.mag_reliability = mag_rel;
      attitude.ekf_mode = (mag_rel >= MAG_RELIABILITY_MODE_THRESHOLD) ? 2U : 1U;

      (void)AttitudeData_Set(&attitude);
    }
    else
    {
      uint32_t now_ms = HAL_GetTick();
      if (first_warn || ((now_ms - last_warn_ms) >= 1000U))
      {
        first_warn = false;
        last_warn_ms = now_ms;
        Debug_Log_Level(DBG_WARN, "Mahony sensor get err=%d", (int)st);
      }
    }

    osDelay(MAHONY_TASK_PERIOD_MS);
  }
}

#ifndef EKF_GYRO_NOISE
#define EKF_GYRO_NOISE   0.01f
#endif
#ifndef EKF_BIAS_NOISE
#define EKF_BIAS_NOISE   0.001f
#endif
#ifndef EKF_ACCEL_NOISE
#define EKF_ACCEL_NOISE  0.1f
#endif
#ifndef EKF_MAG_NOISE
#define EKF_MAG_NOISE    0.3f
#endif

void Task_EKFFusion(void)
{
  EKF_t ekf;
  uint32_t last_update_ms = HAL_GetTick();
  uint32_t last_warn_ms = 0U;
  bool first_warn = true;

  EKF_Init(&ekf, EKF_GYRO_NOISE, EKF_BIAS_NOISE, EKF_ACCEL_NOISE, EKF_MAG_NOISE);

  for (;;)
  {
    SensorData_t sensor;
    Driver_Status st = SensorData_Get(&sensor);
    if (st == DRV_OK)
    {
      uint32_t now_ms = HAL_GetTick();
      float dt = Mahony_CalcDt(now_ms, &last_update_ms);
      float mag_rel = Mahony_CalcMagReliability(&sensor);

      EKF_Predict(&ekf, sensor.gx, sensor.gy, sensor.gz, dt);
      EKF_Update(&ekf,
                 sensor.ax, sensor.ay, sensor.az,
                 sensor.mx, sensor.my, sensor.mz,
                 mag_rel);

      AttitudeData_t attitude = {0};
      attitude.timestamp_ms = now_ms;
      EKF_GetEuler(&ekf, &attitude.roll, &attitude.pitch, &attitude.yaw);
      EKF_GetBias(&ekf, &attitude.bgx, &attitude.bgy, &attitude.bgz);
      attitude.mag_reliability = mag_rel;
      attitude.ekf_mode = (mag_rel >= MAG_RELIABILITY_MODE_THRESHOLD) ? 2U : 1U;

      (void)AttitudeData_Set(&attitude);
    }
    else
    {
      uint32_t now_ms = HAL_GetTick();
      if (first_warn || ((now_ms - last_warn_ms) >= 1000U))
      {
        first_warn = false;
        last_warn_ms = now_ms;
        Debug_Log_Level(DBG_WARN, "EKF sensor get err=%d", (int)st);
      }
    }

    osDelay(MAHONY_TASK_PERIOD_MS);
  }
}

#define CPU_STATS_PERIOD_MS            5000U

void Task_Debug(void)
{
  bool oled_ready = false;
  uint32_t last_warn_ms = 0U;
  bool first_warn = true;
  uint32_t last_stats_ms = 0U;

  for (;;)
  {
    if (!oled_ready)
    {
      Driver_Status init_st = OLED_Init();
      if (init_st == DRV_OK)
      {
        oled_ready = true;
      }
      else
      {
        uint32_t now_ms = HAL_GetTick();
        if (first_warn || ((now_ms - last_warn_ms) >= OLED_INIT_RETRY_MS))
        {
          first_warn = false;
          last_warn_ms = now_ms;
          Debug_Log_Level(DBG_WARN, "OLED init err=%d", (int)init_st);
        }
        osDelay(OLED_INIT_RETRY_MS);
        continue;
      }
    }

    AttitudeData_t attitude;
    Driver_Status data_st = AttitudeData_Get(&attitude);
    if (data_st == DRV_OK)
    {
      uint32_t now_ms = HAL_GetTick();
      bool stale = (attitude.timestamp_ms == 0U) ||
                   ((now_ms - attitude.timestamp_ms) > OLED_ATTITUDE_STALE_MS);
      Driver_Status oled_st = Oled_RenderAttitude(&attitude, stale);
      if (oled_st != DRV_OK)
      {
        oled_ready = false;
        uint32_t warn_ms = HAL_GetTick();
        if (first_warn || ((warn_ms - last_warn_ms) >= OLED_INIT_RETRY_MS))
        {
          first_warn = false;
          last_warn_ms = warn_ms;
          Debug_Log_Level(DBG_WARN, "OLED render err=%d", (int)oled_st);
        }
      }
    }
    else
    {
      uint32_t now_ms = HAL_GetTick();
      if (first_warn || ((now_ms - last_warn_ms) >= OLED_INIT_RETRY_MS))
      {
        first_warn = false;
        last_warn_ms = now_ms;
        Debug_Log_Level(DBG_WARN, "OLED attitude get err=%d", (int)data_st);
      }
    }

    uint32_t tick = HAL_GetTick();
    if ((tick - last_stats_ms) >= CPU_STATS_PERIOD_MS)
    {
      last_stats_ms = tick;
      TaskStatus_t tasks[12];
      uint32_t total_time;
      UBaseType_t n = uxTaskGetSystemState(tasks, 12U, &total_time);
      if ((n > 0U) && (total_time > 0U))
      {
        Debug_Log_Level(DBG_INFO, "--- CPU stats (total=%lu) ---", (unsigned long)total_time);
        for (UBaseType_t i = 0U; i < n; i++)
        {
          uint32_t pct = (uint32_t)(((uint64_t)tasks[i].ulRunTimeCounter * 100ULL) /
                                    (uint64_t)total_time);
          Debug_Log_Level(DBG_INFO, "  %-16s %3lu%%", tasks[i].pcTaskName, (unsigned long)pct);
        }
      }
    }

    osDelay(OLED_TASK_PERIOD_MS);
  }
}
