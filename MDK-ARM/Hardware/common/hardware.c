/**
 * @file    hardware.c
 * @brief   Driver_Layer aggregate implementation.
 *
 * Delivers:
 *   - Task 1.3  — `Driver_MapHalStatus` HAL → `Driver_Status` mapping.
 *   - Task 9.1  — Shared data globals, mutex, and four accessors.
 *   - Task 10.1 — `Hardware_Init`.
 *   - Task 10.2 — `Hardware_SelfTest`.
 *   - Task 11.1 — `Sensor_SampleOnce`.
 *   - Task 11.2 — `Sensor_GetErrorCounters`, `Heater_ApplyDuty`.
 *
 * Spec: .kiro/specs/hardware-base-drivers/{requirements,design,tasks}.md
 */
#include "hardware.h"

#include <string.h>     /* memcpy */


/* ================================================================== */
/* Driver_MapHalStatus (task 1.3, Requirements 8.4, 8.5)               */
/* ================================================================== */

/**
 * @brief Translate an `HAL_StatusTypeDef` into the Driver_Layer status.
 *
 * Mapping (matches design.md §Error Handling):
 *   - `HAL_OK`       → `DRV_OK`
 *   - `HAL_TIMEOUT`  → `DRV_ERR_TIMEOUT`
 *   - `HAL_ERROR`    → `DRV_ERR_BUS`  (via `default`)
 *   - `HAL_BUSY`     → `DRV_ERR_BUS`  (via `default`)
 *   - any other      → `DRV_ERR_BUS`  (via `default`)
 *
 * Validates: Requirements 8.4, 8.5.
 */
Driver_Status Driver_MapHalStatus(HAL_StatusTypeDef hs)
{
    switch (hs) {
        case HAL_OK:
            return DRV_OK;
        case HAL_TIMEOUT:
            return DRV_ERR_TIMEOUT;
        case HAL_ERROR:
        case HAL_BUSY:
        default:
            return DRV_ERR_BUS;
    }
}


/* ================================================================== */
/* Shared data + mutex (task 9.1, Requirements 7.3..7.9)               */
/* ================================================================== */

/**
 * @brief Latest raw sensor frame.  Written by `SensorData_Set`, read by
 *        `SensorData_Get`.  Both accessors hold @ref DataMutexHandle.
 *
 * Statically zero-initialised so `_Get` returning before any `_Set`
 * still produces deterministic output (all-zero `SensorData_t`, which
 * upper layers can recognise via `timestamp_ms == 0`).
 */
static SensorData_t   g_sensor_data;

/**
 * @brief Latest attitude frame.  Mirrors `g_sensor_data` w.r.t. locking
 *        and zero-init semantics.
 */
static AttitudeData_t g_attitude_data;

/**
 * @brief Mutex protecting the two globals above.
 *
 * Storage is here; declaration is `extern osMutexId_t DataMutexHandle;`
 * in `hardware.h` so other Driver_Layer sub-modules
 * (e.g. `Heater_ApplyDuty`) can take the lock when they need to write
 * back into `g_sensor_data`.
 *
 * Created exactly once by @ref DataMutex_Create which is invoked from
 * `Hardware_Init` (task 10.1).  Until then `DataMutexHandle == NULL`
 * and any pre-init Set/Get call from a running task observes a
 * `DRV_ERR_TIMEOUT` (osMutexAcquire on NULL handle), while pre-scheduler
 * callers fall through to the no-lock fast path — which is exactly the
 * sequence allowed by Requirement 7.8.
 */
osMutexId_t DataMutexHandle;

/**
 * @brief Mutex attributes kept in static storage so the underlying
 *        CMSIS-RTOS implementation can hold a long-lived pointer to
 *        the `name` string.
 */
static const osMutexAttr_t s_data_mutex_attr = {
    .name      = "DataMutex",
    .attr_bits = 0u,
    .cb_mem    = NULL,
    .cb_size   = 0u,
};

/**
 * @brief Idempotently create @ref DataMutexHandle.
 *
 * Intended to be called by `Hardware_Init` (task 10.1).  Exposed at
 * file scope so the unit test that drives task 9.1 in isolation can
 * also create the mutex without pulling in the full init pipeline.
 *
 * @retval DRV_OK              Mutex created (or already existed).
 * @retval DRV_ERR_NOT_INIT    `osMutexNew` returned NULL.
 *
 * Validates: Requirement 7.7.
 */
Driver_Status DataMutex_Create(void)
{
    if (DataMutexHandle != NULL) {
        return DRV_OK;          /* idempotent */
    }
    DataMutexHandle = osMutexNew(&s_data_mutex_attr);
    if (DataMutexHandle == NULL) {
        return DRV_ERR_NOT_INIT;
    }
    return DRV_OK;
}


/* ------------------------------------------------------------------ */
/* Internal lock helpers — single source of truth for the             */
/* "scheduler-not-running ⇒ skip mutex" rule (Requirement 7.8).        */
/* ------------------------------------------------------------------ */

static inline bool data_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief Acquire @ref DataMutexHandle if the scheduler is running.
 *
 * @retval DRV_OK            Lock taken, or scheduler not yet running.
 * @retval DRV_ERR_TIMEOUT   `osMutexAcquire` did not return `osOK`
 *                            within @ref HAL_TIMEOUT_MS (also covers
 *                            the rare case of a NULL handle, which
 *                            CMSIS-RTOS rejects with `osErrorParameter`).
 */
static Driver_Status data_lock(void)
{
    if (!data_scheduler_running()) {
        return DRV_OK;
    }
    if (osMutexAcquire(DataMutexHandle, HAL_TIMEOUT_MS) != osOK) {
        return DRV_ERR_TIMEOUT;
    }
    return DRV_OK;
}

static void data_unlock(void)
{
    if (data_scheduler_running() && DataMutexHandle != NULL) {
        (void)osMutexRelease(DataMutexHandle);
    }
}


/* ------------------------------------------------------------------ */
/* Public accessors                                                    */
/* ------------------------------------------------------------------ */

/**
 * @copydoc SensorData_Set
 *
 * Validates: Requirements 7.3, 7.4, 7.8.
 */
Driver_Status SensorData_Set(const SensorData_t *src)
{
    if (src == NULL) {
        return DRV_ERR_PARAM;
    }
    Driver_Status st = data_lock();
    if (st != DRV_OK) {
        return st;
    }
    memcpy(&g_sensor_data, src, sizeof(g_sensor_data));
    data_unlock();
    return DRV_OK;
}

/**
 * @copydoc SensorData_Get
 *
 * Validates: Requirements 7.3, 7.5, 7.8.
 */
Driver_Status SensorData_Get(SensorData_t *dst)
{
    if (dst == NULL) {
        return DRV_ERR_PARAM;
    }
    Driver_Status st = data_lock();
    if (st != DRV_OK) {
        return st;
    }
    memcpy(dst, &g_sensor_data, sizeof(g_sensor_data));
    data_unlock();
    return DRV_OK;
}

/**
 * @copydoc AttitudeData_Set
 *
 * Validates: Requirements 7.3, 7.6, 7.8.
 */
Driver_Status AttitudeData_Set(const AttitudeData_t *src)
{
    if (src == NULL) {
        return DRV_ERR_PARAM;
    }
    Driver_Status st = data_lock();
    if (st != DRV_OK) {
        return st;
    }
    memcpy(&g_attitude_data, src, sizeof(g_attitude_data));
    data_unlock();
    return DRV_OK;
}

/**
 * @copydoc AttitudeData_Get
 *
 * Validates: Requirements 7.3, 7.6, 7.8.
 */
Driver_Status AttitudeData_Get(AttitudeData_t *dst)
{
    if (dst == NULL) {
        return DRV_ERR_PARAM;
    }
    Driver_Status st = data_lock();
    if (st != DRV_OK) {
        return st;
    }
    memcpy(dst, &g_attitude_data, sizeof(g_attitude_data));
    data_unlock();
    return DRV_OK;
}


/* ================================================================== */
/* Hardware_Init (task 10.1, Requirements 1.3, 1.4, 1.5, 1.6, 7.7)    */
/* ================================================================== */

/**
 * @brief Initialise the entire Driver_Layer in a fixed order.
 *
 * Sequence:
 *   1. Create @ref DataMutexHandle via @ref DataMutex_Create.
 *   2. Call each sub-module `*_Init` in order:
 *        Debug_UART → BMP280 → IST8310 → BMI088 → Heater.
 *   3. On the first non-`DRV_OK` return:
 *        - If the failing module is NOT Debug_UART and Debug_UART has
 *          already been initialised (i.e. index > 0), emit an error log
 *          via `Debug_Log_Level(DBG_ERR, ...)` containing the module
 *          name and numeric error code.
 *        - If the failing module IS Debug_UART (index 0), skip logging.
 *        - Return the error code immediately; do NOT call later modules.
 *
 * Validates: Requirements 1.3, 1.4, 1.5, 1.6, 7.7.
 */
Driver_Status Hardware_Init(void)
{
    Driver_Status err;

    /* ---- Step 1: Create the shared-data mutex ---- */
    err = DataMutex_Create();
    if (err != DRV_OK) {
        return err;
    }

    /* ---- Step 2: Init sub-modules in order ---- */

    /* Module 0: Debug_UART (must be first — other modules depend on it
     * for error logging). */
    err = DebugUART_Init();
    if (err != DRV_OK) {
        /* Requirement 1.5: Debug_UART itself failed → skip log output,
         * return error directly. */
        return err;
    }

    /* Module 1: BMP280 */
    err = BMP280_Init();
    if (err != DRV_OK) {
        /* Requirement 1.6: Debug_UART is up → log the failure. */
        Debug_Log_Level(DBG_ERR, "BMP280 init failed: %d", (int)err);
        return err;
    }

    /* Module 2: IST8310 */
    err = IST8310_Init();
    if (err != DRV_OK) {
        Debug_Log_Level(DBG_ERR, "IST8310 init failed: %d", (int)err);
        return err;
    }

    /* Module 3: BMI088 */
    err = BMI088_Init();
    if (err != DRV_OK) {
        Debug_Log_Level(DBG_ERR, "BMI088 init failed: %d", (int)err);
        return err;
    }

    /* Module 4: Heater */
    err = Heater_Init();
    if (err != DRV_OK) {
        Debug_Log_Level(DBG_ERR, "Heater init failed: %d", (int)err);
        return err;
    }

    return DRV_OK;
}


/* ================================================================== */
/* Hardware_SelfTest (task 10.2, Requirements 10.1..10.5)              */
/* ================================================================== */

Driver_Status Hardware_SelfTest(void)
{
    Driver_Status result = DRV_OK;
    uint8_t acc_id = 0, gyro_id = 0, ist_id = 0, bmp_id = 0;

    /* BMI088 accelerometer + gyroscope (single HAL call, two IDs). */
    Driver_Status st = BMI088_GetChipID(&acc_id, &gyro_id);
    if (st == DRV_OK) {
        Debug_Log_Level(DBG_INFO, "BMI088_ACC ID=0x%02X OK", acc_id);
    } else {
        Debug_Log_Level(DBG_ERR, "BMI088_ACC ID=0x%02X FAIL", acc_id);
        result = DRV_ERR_ID;
    }

    if (gyro_id == 0x0F) {
        Debug_Log_Level(DBG_INFO, "BMI088_GYRO ID=0x%02X OK", gyro_id);
    } else {
        Debug_Log_Level(DBG_ERR, "BMI088_GYRO ID=0x%02X FAIL", gyro_id);
        result = DRV_ERR_ID;
    }

    /* IST8310 */
    st = IST8310_GetChipID(&ist_id);
    if (st == DRV_OK) {
        Debug_Log_Level(DBG_INFO, "IST8310 ID=0x%02X OK", ist_id);
    } else {
        Debug_Log_Level(DBG_ERR, "IST8310 ID=0x%02X FAIL", ist_id);
        result = DRV_ERR_ID;
    }

    /* BMP280 */
    st = BMP280_GetChipID(&bmp_id);
    if (st == DRV_OK) {
        Debug_Log_Level(DBG_INFO, "BMP280 ID=0x%02X OK", bmp_id);
    } else {
        Debug_Log_Level(DBG_ERR, "BMP280 ID=0x%02X FAIL", bmp_id);
        result = DRV_ERR_ID;
    }

    return result;
}


/* ================================================================== */
/* Sensor_SampleOnce (task 11.1, Requirements 9.1, 9.2)               */
/* ================================================================== */

/* Throttle divisors: 1 kHz call rate → 50 Hz mag, 10 Hz baro. */
static uint32_t s_mag_div = 0u;
static uint32_t s_bar_div = 0u;

/* Per-sensor error counters (exposed via Sensor_GetErrorCounters). */
static uint32_t s_err_bmi = 0u;
static uint32_t s_err_ist = 0u;
static uint32_t s_err_bmp = 0u;

Driver_Status Sensor_SampleOnce(SensorData_t *out)
{
    if (out == NULL) {
        return DRV_ERR_PARAM;
    }

    Driver_Status first_err = DRV_OK;

    /* BMI088 (every call) */
    Driver_Status st = BMI088_ReadAccel(&out->ax, &out->ay, &out->az);
    if (st != DRV_OK) {
        s_err_bmi++;
        if (first_err == DRV_OK) first_err = st;
    }

    st = BMI088_ReadGyro(&out->gx, &out->gy, &out->gz);
    if (st != DRV_OK) {
        s_err_bmi++;
        if (first_err == DRV_OK) first_err = st;
    }

    /* IST8310 (every 20th call → ~50 Hz) */
    s_mag_div++;
    if (s_mag_div >= 20u) {
        s_mag_div = 0u;
        st = IST8310_ReadMag(&out->mx, &out->my, &out->mz);
        if (st != DRV_OK) {
            s_err_ist++;
            if (first_err == DRV_OK) first_err = st;
        }
    }

    /* BMP280 (every 100th call → ~10 Hz) */
    s_bar_div++;
    if (s_bar_div >= 100u) {
        s_bar_div = 0u;
        st = BMP280_ReadTemperature(&out->temperature);
        if (st != DRV_OK) {
            s_err_bmp++;
            if (first_err == DRV_OK) first_err = st;
        }
    }

    out->timestamp_ms = HAL_GetTick();
    return first_err;
}


/* ================================================================== */
/* Sensor_GetErrorCounters (task 11.2, Requirement 9.3)               */
/* ================================================================== */

void Sensor_GetErrorCounters(uint32_t *bmi, uint32_t *ist, uint32_t *bmp)
{
    if (bmi) *bmi = s_err_bmi;
    if (ist) *ist = s_err_ist;
    if (bmp) *bmp = s_err_bmp;
}


/* ================================================================== */
/* Heater_ApplyDuty (task 11.2, Requirements 9.4, 9.5)                */
/* ================================================================== */

Driver_Status Heater_ApplyDuty(float duty)
{
    /* Set PWM first (no data lock held — lock order: bus lock → data lock). */
    Driver_Status st = Heater_SetDuty(duty);
    if (st != DRV_OK) {
        return st;
    }

    /* Clamp and write back into g_sensor_data under DataMutexHandle. */
    float clamped = duty;
    if (clamped < 0.0f) clamped = 0.0f;
    else if (clamped > 1.0f) clamped = 1.0f;

    st = data_lock();
    if (st != DRV_OK) {
        return st;
    }
    g_sensor_data.pwm_duty = clamped;
    data_unlock();
    return DRV_OK;
}
