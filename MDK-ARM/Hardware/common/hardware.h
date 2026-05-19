/**
 * @file    hardware.h
 * @brief   CloudFlyIMU Driver_Layer aggregate header.
 *
 * This file is the single public entry point for the base hardware driver
 * layer (Driver_Layer). Application/RTOS code should `#include "hardware.h"`
 * and never include the per-module sub-headers directly.
 *
 * Responsibilities of this header (task 1.2):
 *   - Define the unified driver return code @ref Driver_Status.
 *   - Expose project-wide tunables as macros: @ref HAL_TIMEOUT_MS,
 *     @ref HEATER_OVERHEAT_THRESHOLD_C, @ref DEBUG_LEVEL_MIN.
 *   - Define the shared data structures @ref SensorData_t and
 *     @ref AttitudeData_t plus the debug-level enum @ref DEBUG_LEVEL_t.
 *   - Declare every top-level Driver_Layer prototype (Hardware_Init,
 *     Hardware_SelfTest, Sensor_SampleOnce, Sensor_GetErrorCounters,
 *     Heater_ApplyDuty, SensorData_Set/Get, AttitudeData_Set/Get,
 *     Driver_MapHalStatus).
 *   - Declare the shared data mutex handle @ref DataMutexHandle.
 *   - Re-export each sub-module header so a single include is enough.
 *
 * Implementations live in:
 *   - common/hardware.c         (task 1.3, 9.x, 10.x, 11.x)
 *   - debug_uart/debug_uart.c   (task 2.1)
 *   - bmp280/bmp280.c           (task 3.1)
 *   - ist8310/ist8310.c         (task 4.1)
 *   - bmi088/bmi088.c           (task 5.1)
 *   - heater/heater.c           (task 7.1)
 *
 * Spec: .kiro/specs/hardware-base-drivers/{requirements,design,tasks}.md
 *
 * Validates: Requirements 1.2, 7.1, 7.2, 8.1, 8.2, 8.3.
 */
#ifndef HARDWARE_H
#define HARDWARE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Standard library                                                    */
/* ------------------------------------------------------------------ */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* STM32 HAL (HAL_StatusTypeDef, peripheral handle types)              */
/* ------------------------------------------------------------------ */
#include "stm32h7xx_hal.h"

/* ------------------------------------------------------------------ */
/* CMSIS-RTOS V2 (osMutexId_t, osKernelGetState, osMutexAcquire, ...)  */
/* ------------------------------------------------------------------ */
#include "cmsis_os2.h"


/* ================================================================== */
/* 1. Unified driver return code (Requirement 8.1, 8.2)                */
/* ================================================================== */

/**
 * @brief Unified return code for every public driver API.
 *
 * Every initialisation, read, and write entry point in the Driver_Layer
 * returns a value of this type, allowing upper-layer code to react with a
 * single switch / if-chain regardless of which sub-module produced the
 * error.
 */
typedef enum {
    DRV_OK              = 0,    /**< Success. */
    DRV_ERR_BUS         = 1,    /**< Bus-level failure (HAL_ERROR / HAL_BUSY). */
    DRV_ERR_ID          = 2,    /**< Chip ID mismatch on init / self-test. */
    DRV_ERR_TIMEOUT     = 3,    /**< HAL_TIMEOUT or mutex acquire timeout. */
    DRV_ERR_PARAM       = 4,    /**< NULL pointer, invalid address, over-temperature. */
    DRV_ERR_NOT_INIT    = 5,    /**< Module not initialised, or heater fault latched. */
} Driver_Status;


/* ================================================================== */
/* 2. Project-wide tunables (Requirement 5.10, 6.7, 8.3)               */
/* ================================================================== */

/** Default HAL bus-API timeout in milliseconds. Used for every
 *  HAL_SPI_*, HAL_I2C_*, HAL_UART_* call as well as `osMutexAcquire`. */
#define HAL_TIMEOUT_MS                      100u

/** Heater over-temperature trip threshold, in degrees Celsius. */
#ifndef HEATER_OVERHEAT_THRESHOLD_C
#define HEATER_OVERHEAT_THRESHOLD_C         55.0f
#endif


/* ================================================================== */
/* 3. Debug level enum + minimum-level macro (Requirement 6.6, 6.7)    */
/* ================================================================== */

/**
 * @brief Severity levels accepted by `Debug_Log_Level`.
 *
 * Lower numeric values are higher severity. The runtime filter
 * `DEBUG_LEVEL_MIN` drops messages whose level value is greater
 * (i.e. less severe) than the configured threshold.
 */
typedef enum {
    DBG_ERR     = 0,    /**< Errors that prevent further operation.   "[ERR] "  */
    DBG_WARN    = 1,    /**< Warnings, recoverable conditions.        "[WARN] " */
    DBG_INFO    = 2,    /**< Default informational messages.          "[INF] "  */
    DBG_DEBUG   = 3,    /**< Verbose tracing for debugging.           "[DBG] "  */
} DEBUG_LEVEL_t;

/** Minimum severity level emitted at runtime. Messages with `lvl > DEBUG_LEVEL_MIN`
 *  are silently dropped by `Debug_Log_Level`. */
#ifndef DEBUG_LEVEL_MIN
#define DEBUG_LEVEL_MIN                     DBG_INFO
#endif


/* ================================================================== */
/* 4. Shared data structures (Requirement 7.1, 7.2)                    */
/* ================================================================== */

/**
 * @brief Latest raw sensor frame shared between SensorTask and consumers.
 *
 * Owned by `g_sensor_data` in `common/hardware.c`; read/written exclusively
 * via `SensorData_Set` / `SensorData_Get` while holding `DataMutexHandle`
 * (or directly when the FreeRTOS scheduler is not yet running).
 */
typedef struct {
    uint32_t timestamp_ms;      /**< HAL_GetTick() at the time of capture.   */
    float    ax, ay, az;        /**< Acceleration, m/s^2 (BMI088).           */
    float    gx, gy, gz;        /**< Angular rate,  rad/s (BMI088).          */
    float    mx, my, mz;        /**< Magnetic field, microtesla (IST8310).   */
    float    temperature;       /**< Temperature, deg C (BMP280).            */
    float    pressure;          /**< Pressure, Pa (BMP280).                  */
    float    pwm_duty;          /**< Last heater duty applied, 0.0 .. 1.0.   */
} SensorData_t;

/**
 * @brief Latest attitude / EKF output frame.
 *
 * Defined here so the data contract is owned by the Driver_Layer; the
 * actual fusion logic is delivered by a separate spec. The Driver_Layer
 * only guarantees the struct layout and mutex-protected accessors.
 */
typedef struct {
    uint32_t timestamp_ms;      /**< HAL_GetTick() at the time of update.    */
    float    roll, pitch, yaw;  /**< Euler angles, radians.                  */
    float    bgx, bgy, bgz;     /**< Gyro bias estimate, rad/s.              */
    float    mag_reliability;   /**< Magnetometer trust factor, 0.0 .. 1.0.  */
    uint8_t  ekf_mode;          /**< 0 = idle, 1 = 6-axis, 2 = 9-axis.       */
} AttitudeData_t;


/* ================================================================== */
/* 5. Shared mutex handles (Requirement 7.7)                            */
/* ================================================================== */

/**
 * @brief Mutex protecting `g_sensor_data` and `g_attitude_data`.
 *
 * Created by `Hardware_Init` via `osMutexNew`. Defined (storage) in
 * `common/hardware.c`; declared `extern` here so other Driver_Layer
 * sub-modules (e.g. `Heater_ApplyDuty` in the aggregate layer) can take
 * the lock when writing back into `g_sensor_data`.
 *
 * Ordering rule: bus mutex (SPI/I2C) is always acquired BEFORE this one;
 * never the other way around. This avoids the only potential deadlock.
 */
extern osMutexId_t DataMutexHandle;

/**
 * @brief Idempotently create @ref DataMutexHandle via `osMutexNew`.
 *
 * Helper called by `Hardware_Init` (task 10.1). Safe to call more than
 * once: subsequent calls return `DRV_OK` without creating a second
 * handle. Exposed at API level so tests that drive task 9.1 in
 * isolation (and any future caller that needs the data lock before
 * `Hardware_Init` is wired up) can create it directly.
 *
 * @retval DRV_OK            Mutex created (or already existed).
 * @retval DRV_ERR_NOT_INIT  `osMutexNew` returned NULL.
 *
 * Validates: Requirement 7.7.
 */
Driver_Status DataMutex_Create(void);


/* ================================================================== */
/* 6. Top-level Driver_Layer prototypes                                */
/* ================================================================== */

/**
 * @brief Initialise the entire Driver_Layer in a fixed order.
 *
 * Order: Debug_UART → BMP280 → IST8310 → BMI088 → Heater. Returns the
 * first non-`DRV_OK` status without invoking later sub-modules. If the
 * failing module is NOT Debug_UART_Driver and Debug_UART_Driver has
 * already been initialised, an error log line is emitted via
 * `Debug_Log_Level(DBG_ERR, ...)` before returning.
 *
 * Also creates `DataMutexHandle` via `osMutexNew`.
 *
 * Validates: Requirements 1.3, 1.4, 1.5, 1.6, 7.7.
 */
Driver_Status Hardware_Init(void);

/**
 * @brief Read every sensor's Chip ID and log per-sensor PASS/FAIL lines.
 *
 * Continues through the full list even if some checks fail, so a single
 * run produces a complete diagnosis. Returns `DRV_OK` only when every
 * Chip ID matches its expected value, otherwise `DRV_ERR_ID`.
 *
 * Safe to call before `osKernelStart()`.
 *
 * Validates: Requirements 10.1, 10.2, 10.3, 10.4, 10.5.
 */
Driver_Status Hardware_SelfTest(void);

/**
 * @brief Translate an `HAL_StatusTypeDef` into the Driver_Layer status.
 *
 * Mapping:
 *   - `HAL_OK`       → `DRV_OK`
 *   - `HAL_TIMEOUT`  → `DRV_ERR_TIMEOUT`
 *   - `HAL_ERROR`    → `DRV_ERR_BUS`
 *   - `HAL_BUSY`     → `DRV_ERR_BUS`
 *   - any other      → `DRV_ERR_BUS`
 *
 * Validates: Requirements 8.4, 8.5.
 */
Driver_Status Driver_MapHalStatus(HAL_StatusTypeDef hs);


/* ================================================================== */
/* 7. Shared data accessors (Requirement 7.4 .. 7.6, 7.8, 7.9)         */
/* ================================================================== */

/** Copy `*src` into the global SensorData under `DataMutexHandle`. */
Driver_Status SensorData_Set(const SensorData_t *src);

/** Copy the global SensorData into `*dst` under `DataMutexHandle`. */
Driver_Status SensorData_Get(SensorData_t *dst);

/** Copy `*src` into the global AttitudeData under `DataMutexHandle`. */
Driver_Status AttitudeData_Set(const AttitudeData_t *src);

/** Copy the global AttitudeData into `*dst` under `DataMutexHandle`. */
Driver_Status AttitudeData_Get(AttitudeData_t *dst);


/* ================================================================== */
/* 8. Task integration hooks (Requirement 9)                            */
/* ================================================================== */

/**
 * @brief Acquire one frame of raw sensor data into `*out`.
 *
 * Calls BMI088_ReadAccel + BMI088_ReadGyro every invocation; throttles
 * IST8310_ReadMag to ~50 Hz and BMP280_ReadTemperature to ~10 Hz on the
 * assumption that this routine is invoked at ~1 kHz.
 *
 * On a per-sensor failure, the previous successful value for that sensor
 * is preserved in `*out`, an internal error counter is incremented, and
 * the rest of the read sequence still runs. The return value is the
 * first non-`DRV_OK` status encountered (BMI → IST → BMP order).
 *
 * Validates: Requirements 9.1, 9.2.
 */
Driver_Status Sensor_SampleOnce(SensorData_t *out);

/**
 * @brief Snapshot the per-sensor cumulative error counters.
 *
 * Any output pointer may be `NULL` to skip that counter.
 *
 * Validates: Requirements 9.3.
 */
void Sensor_GetErrorCounters(uint32_t *bmi, uint32_t *ist, uint32_t *bmp);

/**
 * @brief Set heater duty and write the applied value back to SensorData.
 *
 * Internally calls `Heater_SetDuty(duty)`; on success, updates
 * `g_sensor_data.pwm_duty` while holding `DataMutexHandle`.
 *
 * Validates: Requirements 9.4.
 */
Driver_Status Heater_ApplyDuty(float duty);


/* ================================================================== */
/* 9. Sub-module headers (single-include facade)                        */
/*                                                                      */
/* Included LAST so each sub-module header may rely on the shared       */
/* types/macros defined above (Driver_Status, DEBUG_LEVEL_t,            */
/* SensorData_t, HAL_TIMEOUT_MS, ...).                                  */
/* ================================================================== */
#include "debug_uart.h"
#include "bmp280.h"
#include "ist8310.h"
#include "bmi088.h"
#include "heater.h"


#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_H */
