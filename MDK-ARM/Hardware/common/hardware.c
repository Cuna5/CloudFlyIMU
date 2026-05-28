/**
 * @file    hardware.c
 * @brief   驱动层聚合实现。
 *
 * 包含：Driver_MapHalStatus、共享数据全局变量与互斥锁、
 * Hardware_Init、Hardware_SelfTest、Sensor_SampleOnce、
 * Sensor_GetErrorCounters、Heater_ApplyDuty。
 */
#include "hardware.h"

#include <string.h>     /* memcpy */


/* ================================================================== */
/* Driver_MapHalStatus                                                 */
/* ================================================================== */

/**
 * @brief 将 HAL_StatusTypeDef 映射为驱动层返回码。
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
/* 共享数据 + 互斥锁                                                   */
/* ================================================================== */

/* 最新原始传感器帧，由 SensorData_Set 写入，SensorData_Get 读取。
 * 静态零初始化，确保调度器启动前读取也有确定性输出（timestamp_ms==0）。 */
static SensorData_t   g_sensor_data;

/* 最新姿态帧，加锁和零初始化语义与 g_sensor_data 相同。 */
static AttitudeData_t g_attitude_data;

/* 保护上述两个全局变量的互斥锁。
 * 存储于此，hardware.h 中 extern 声明供其他子模块使用。
 * 由 DataMutex_Create 在 Hardware_Init 中创建一次。
 * 加锁顺序：总线互斥锁（SPI/I2C）必须在本锁之前获取。 */
osMutexId_t DataMutexHandle;

/* 互斥锁属性，存于静态存储以保证 name 字符串生命周期。 */
static const osMutexAttr_t s_data_mutex_attr = {
    .name      = "DataMutex",
    .attr_bits = 0u,
    .cb_mem    = NULL,
    .cb_size   = 0u,
};

/**
 * @brief 幂等地创建 DataMutexHandle。由 Hardware_Init 调用。
 */
Driver_Status DataMutex_Create(void)
{
    if (DataMutexHandle != NULL) {
        return DRV_OK;          /* 幂等 */
    }
    DataMutexHandle = osMutexNew(&s_data_mutex_attr);
    if (DataMutexHandle == NULL) {
        return DRV_ERR_NOT_INIT;
    }
    return DRV_OK;
}


/* ------------------------------------------------------------------ */
/* 内部加锁辅助函数——"调度器未运行则跳过互斥锁"规则的唯一实现处。    */
/* ------------------------------------------------------------------ */

static inline bool data_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief 调度器运行时获取 DataMutexHandle。
 *
 * @return DRV_OK 已加锁或调度器未运行；DRV_ERR_TIMEOUT 超时。
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
/* 公共访问器                                                          */
/* ------------------------------------------------------------------ */

/**
 * @copydoc SensorData_Set
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
/* Hardware_Init                                                       */
/* ================================================================== */

/**
 * @brief 按固定顺序初始化整个驱动层。
 *
 * Order: Debug_UART -> BMP280 -> IST8310 -> BMI088 -> Heater -> SDStorage.
 * SDStorage is non-critical at boot; parameter users can fall back to defaults.
 */
Driver_Status Hardware_Init(void)
{
    Driver_Status err;

    /* ---- 步骤 1：创建共享数据互斥锁 ---- */
    err = DataMutex_Create();
    if (err != DRV_OK) {
        return err;
    }

    /* ---- 步骤 2：按顺序初始化子模块 ---- */

    LED_Init();

    /* 模块 0：Debug_UART（必须最先初始化，其他模块依赖它输出错误日志） */
    err = DebugUART_Init();
    if (err != DRV_OK) {
        /* Debug_UART 本身失败，跳过日志输出，直接返回错误 */
        return err;
    }

    /* 模块 1：BMP280 */
//    err = BMP280_Init();
//    if (err != DRV_OK) {
//        Debug_Log_Level(DBG_ERR, "BMP280 init failed: %d", (int)err);
//        return err;
//    }

    /* 模块 2：IST8310 */
    err = IST8310_Init();
    if (err != DRV_OK) {
        Debug_Log_Level(DBG_ERR, "IST8310 init failed: %d", (int)err);
        return err;
    }

    /* 模块 3：BMI088 */
    err = BMI088_Init();
    if (err != DRV_OK) {
        Debug_Log_Level(DBG_ERR, "BMI088 init failed: %d", (int)err);
        return err;
    }

    /* 模块 4：Heater */
    err = Heater_Init();
    if (err != DRV_OK) {
        Debug_Log_Level(DBG_ERR, "Heater init failed: %d", (int)err);
        return err;
    }

    /* Module 5: SD/FatFs storage (non-critical, parameters can use defaults) */
    err = SDStorage_Init();
    if (err != DRV_OK) {
        Debug_Log_Level(DBG_WARN, "SD storage init failed: %d (params will use defaults)", (int)err);
    }

    return DRV_OK;
}


/* ================================================================== */
/* Hardware_SelfTest                                                   */
/* ================================================================== */

Driver_Status Hardware_SelfTest(void)
{
    Driver_Status result = DRV_OK;
    uint8_t acc_id = 0, gyro_id = 0, ist_id = 0, bmp_id = 0;

    /* BMI088 加速度计 + 陀螺仪 */
    Driver_Status st = BMI088_GetChipID(&acc_id, &gyro_id);
    if (st == DRV_OK && acc_id == 0x1E) {
        Debug_Log_Level(DBG_INFO, "BMI088_ACC ID=0x%02X OK\r\n", acc_id);
    } else {
        Debug_Log_Level(DBG_ERR, "BMI088_ACC ID=0x%02X FAIL\r\n", acc_id);
        result = DRV_ERR_ID;
    }

    if (st == DRV_OK && gyro_id == 0x0F) {
        Debug_Log_Level(DBG_INFO, "BMI088_GYRO ID=0x%02X OK\r\n", gyro_id);
    } else {
        Debug_Log_Level(DBG_ERR, "BMI088_GYRO ID=0x%02X FAIL\r\n", gyro_id);
        result = DRV_ERR_ID;
    }

    /* IST8310 */
    st = IST8310_GetChipID(&ist_id);
    if (st == DRV_OK && ist_id == 0x10) {
        Debug_Log_Level(DBG_INFO, "IST8310 ID=0x%02X OK\r\n", ist_id);
    } else {
        Debug_Log_Level(DBG_ERR, "IST8310 ID=0x%02X FAIL\r\n", ist_id);
        result = DRV_ERR_ID;
    }

    /* BMP280 */
    st = BMP280_GetChipID(&bmp_id);
    if (st == DRV_OK && bmp_id == 0x58) {
        Debug_Log_Level(DBG_INFO, "BMP280 ID=0x%02X OK\r\n", bmp_id);
    } else {
        Debug_Log_Level(DBG_ERR, "BMP280 ID=0x%02X FAIL\r\n", bmp_id);
        result = DRV_ERR_ID;
    }

    return result;
}


/* ================================================================== */
/* Sensor_SampleOnce                                                   */
/* ================================================================== */

/* 节流计数器：假设 1 kHz 调用频率 → 磁力计约 50 Hz，气压计约 10 Hz */
static uint32_t s_mag_div = 0u;
static uint32_t s_bar_div = 0u;

/* 各传感器累计错误计数（通过 Sensor_GetErrorCounters 暴露） */
static uint32_t s_err_bmi = 0u;
static uint32_t s_err_ist = 0u;
static uint32_t s_err_bmp = 0u;

Driver_Status Sensor_SampleOnce(SensorData_t *out)
{
    if (out == NULL) {
        return DRV_ERR_PARAM;
    }

    Driver_Status first_err = DRV_OK;

    /* BMI088（每次调用） */
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

    /* IST8310（每 20 次调用约 50 Hz） */
    s_mag_div++;
    if (s_mag_div >= 20u) {
        s_mag_div = 0u;
        st = IST8310_ReadMag(&out->mx, &out->my, &out->mz);
        if (st != DRV_OK) {
            s_err_ist++;
            if (first_err == DRV_OK) first_err = st;
        }
    }

    /* BMP280（每 100 次调用约 10 Hz） */
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
/* Sensor_GetErrorCounters                                             */
/* ================================================================== */

void Sensor_GetErrorCounters(uint32_t *bmi, uint32_t *ist, uint32_t *bmp)
{
    if (bmi) *bmi = s_err_bmi;
    if (ist) *ist = s_err_ist;
    if (bmp) *bmp = s_err_bmp;
}


/* ================================================================== */
/* Heater_ApplyDuty                                                    */
/* ================================================================== */

Driver_Status Heater_ApplyDuty(float duty)
{
    /* 先设置 PWM（不持有数据锁——加锁顺序：总线锁 → 数据锁） */
    Driver_Status st = Heater_SetDuty(duty);
    if (st != DRV_OK) {
        return st;
    }

    /* 夹紧后在 DataMutexHandle 保护下写回 g_sensor_data */
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
