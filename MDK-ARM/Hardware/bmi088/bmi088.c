/**
 * @file    bmi088.c
 * @brief   BMI088 六轴 IMU 驱动（SPI1，双片选 PC4/PC5）实现。
 *
 * SPI 读协议差异（BMI088 数据手册 §6.1.2 / §6.1.3）：
 *   - 加速度计读取：地址字节后需一个哑字节，事务共发 len+2 字节，
 *     保留 rx[2..2+len-1]。
 *   - 陀螺仪读取：无哑字节，事务共发 len+1 字节，保留 rx[1..len]。
 *   - 单寄存器写入：2 字节 [reg, value]。
 *
 * 并发：所有公共 API 在调度器运行时通过 SPIMutexHandle 加锁，
 * 调度器未启动时跳过加锁（单线程启动阶段）。
 * HAL 返回码通过 Driver_MapHalStatus 映射为统一的 Driver_Status。
 */
/* 先包含 hardware.h：它拥有 Driver_Status 枚举和 HAL_TIMEOUT_MS，
 * 并通过聚合门面重新导出 bmi088.h。 */
#include "hardware.h"   /* Driver_Status, HAL_TIMEOUT_MS, Driver_MapHalStatus */
#include "spi.h"        /* extern hspi1 */
#include "main.h"       /* BMI_ACC_CS_Pin, BMI_GYRO_CS_Pin */

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* M_PI 不属于严格 ISO C，提供可移植的备用定义 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* 共享 SPI 总线互斥锁（由 CubeMX 在 Core/Src/freertos.c 中创建）     */
/* ------------------------------------------------------------------ */
extern osMutexId_t SPIMutexHandle;

/* ------------------------------------------------------------------ */
/* BMI088 寄存器映射——仅列出本驱动使用的地址                          */
/* ------------------------------------------------------------------ */

/* --- 加速度计（从机 A）--- */
#define BMI_ACC_REG_CHIP_ID         0x00u   /* 期望值 0x1E                                  */
#define BMI_ACC_REG_DATA            0x12u   /* 突发起始：ACC_X_LSB..ACC_Z_MSB（6 字节）     */
#define BMI_ACC_REG_CONF            0x40u   /* ODR / BW                                     */
#define BMI_ACC_REG_RANGE           0x41u   /* 量程                                         */
#define BMI_ACC_REG_PWR_CTRL        0x7Du   /* 电源控制（Normal / Suspend）                 */
#define BMI_ACC_REG_SOFTRESET       0x7Eu   /* 写 0xB6 触发软复位                           */

/* --- 陀螺仪（从机 G）--- */
#define BMI_GYRO_REG_CHIP_ID        0x00u   /* 期望值 0x0F                                  */
#define BMI_GYRO_REG_DATA           0x02u   /* 突发起始：RATE_X_LSB..RATE_Z_MSB（6 字节）   */
#define BMI_GYRO_REG_RANGE          0x0Fu   /* 量程                                         */
#define BMI_GYRO_REG_BANDWIDTH      0x10u   /* ODR / BW                                     */

/* 期望的 Chip ID 值 */
#define BMI_ACC_CHIP_ID             0x1Eu
#define BMI_GYRO_CHIP_ID            0x0Fu

#define BMI_ACC_REG_PWR_CONF        0x7Cu
#define BMI_ACC_PWR_CONF_ACTIVE     0x00u
#define BMI_ACC_PWR_CTRL_ENABLE     0x04u
#define BMI_ACC_PWR_CTRL_DELAY_MS   5u
#define BMI_ACC_POWER_UP_DELAY_MS   50u

/* 初始化时使用的寄存器值 */
#define BMI_ACC_SOFTRESET_VALUE     0xB6u   /* 与 BMP280 软复位值相同 */
#define BMI_ACC_RANGE_6G            0x01u   /* ±6 g                                       */
#define BMI_ACC_CONF_NORMAL_800HZ   0xABu   /* Normal BW，ODR = 800 Hz                    */
#define BMI_GYRO_RANGE_2000DPS      0x00u   /* ±2000 °/s                                  */
#define BMI_GYRO_BW_1000_116        0x02u   /* ODR 1000 Hz / 3-dB BW 116 Hz               */

/* SPI 访问标志 */
#define BMI_SPI_READ_BIT            0x80u   /* 读操作时与地址字节 OR */

/* 软复位稳定时间（数据手册 §5.4）：典型 30 ms，规格 50 ms */
#define BMI_ACC_SOFT_RESET_DELAY_MS 50u
#define BMI_ACC_SPI_WAKE_DELAY_MS   1u
#define BMI_ACC_ID_RETRY_COUNT      5u
#define BMI_ACC_ID_RETRY_DELAY_MS   2u

/* ------------------------------------------------------------------ */
/* 灵敏度 / 单位转换                                                   */
/* ------------------------------------------------------------------ */

/** 加速度计 ±6g 量程灵敏度：5460 LSB/g（数据手册 §5.3）。 */
#define BMI_ACC_LSB_PER_G           5460.0f
/** 标准重力加速度，m/s²。 */
#define BMI_GRAVITY_MS2             9.80665f
/** 陀螺仪 ±2000°/s 量程灵敏度：16.384 LSB/(°/s)（数据手册 §5.2）。 */
#define BMI_GYRO_LSB_PER_DPS        16.384f
/** 角度转弧度系数。 */
#define BMI_DEG_TO_RAD              ((float)M_PI / 180.0f)

/* ------------------------------------------------------------------ */
/* 模块状态                                                            */
/* ------------------------------------------------------------------ */

/** BMI088_Init 成功完成后置 true，用于门控所有读取操作。 */
static bool s_initialized = false;

/* ------------------------------------------------------------------ */
/* 片选辅助函数                                                        */
/* ------------------------------------------------------------------ */

/** 拉低 PC4、拉高 PC5，选中加速度计。 */
static inline void bmi_acc_select(void)
{
    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port, BMI_GYRO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI_ACC_CS_GPIO_Port,  BMI_ACC_CS_Pin,  GPIO_PIN_RESET);
}

/** 拉高 PC4、拉低 PC5，选中陀螺仪。 */
static inline void bmi_gyro_select(void)
{
    HAL_GPIO_WritePin(BMI_ACC_CS_GPIO_Port,  BMI_ACC_CS_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port, BMI_GYRO_CS_Pin, GPIO_PIN_RESET);
}

/** 拉高两路片选（空闲/取消选中）。每次事务的所有退出路径均调用此函数，
 *  确保 HAL 错误发生时不会遗留已选中的芯片。 */
static inline void bmi_deselect_all(void)
{
    HAL_GPIO_WritePin(BMI_ACC_CS_GPIO_Port,  BMI_ACC_CS_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port, BMI_GYRO_CS_Pin, GPIO_PIN_SET);
}

/* ------------------------------------------------------------------ */
/* RTOS 辅助函数                                                       */
/* ------------------------------------------------------------------ */

static inline bool bmi_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief 调度器运行时获取共享 SPI 总线互斥锁。
 *
 * @return DRV_OK 已加锁或调度器未运行；DRV_ERR_TIMEOUT 超时。
 */
static Driver_Status bmi_bus_lock(void)
{
    if (!bmi_scheduler_running()) {
        return DRV_OK;
    }
    if (osMutexAcquire(SPIMutexHandle, HAL_TIMEOUT_MS) != osOK) {
        return DRV_ERR_TIMEOUT;
    }
    return DRV_OK;
}

/** 调度器运行时释放 SPI 总线互斥锁；调度器未运行时为空操作。 */
static void bmi_bus_unlock(void)
{
    if (bmi_scheduler_running()) {
        (void)osMutexRelease(SPIMutexHandle);
    }
}

/** 调度器感知延时：调度器运行时用 osDelay，启动阶段用 HAL_Delay。 */
#if defined(DWT) && defined(CoreDebug) && \
    defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
#define BMI_HAS_DWT_DELAY 1
static void bmi_busy_delay_ms(uint32_t ms)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t cycles_per_ms = SystemCoreClock / 1000u;
    if (cycles_per_ms == 0u) {
        cycles_per_ms = 1u;
    }

    while (ms-- > 0u) {
        uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles_per_ms) {
            __NOP();
        }
    }
}
#endif

static void bmi_delay_ms(uint32_t ms)
{
    if (bmi_scheduler_running()) {
        (void)osDelay(ms);
    } else {
#if defined(BMI_HAS_DWT_DELAY)
        bmi_busy_delay_ms(ms);
#else
        HAL_Delay(ms);
#endif
    }
}

/* ------------------------------------------------------------------ */
/* 底层 SPI 原语                                                       */
/*                                                                      */
/* 前提：调用方已持有总线互斥锁，且已拉低对应片选。                    */
/* 这些辅助函数不操作片选，允许调用方在多次写入间保持片选低电平。      */
/* ------------------------------------------------------------------ */

/**
 * @brief 从加速度计寄存器 reg 开始读取 len 字节。
 *
 * Emits `len + 2` SPI bytes, discards the address echo and dummy byte.
 *
 * @pre `reg + 0x80` fits in 8 bits and `len + 2 <= sizeof rx_buf`.
 */
static Driver_Status bmi_acc_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* 本驱动最大有效载荷为 6 字节（数据突发），len+2 ≤ 8，上限设为 16 */
    enum { BMI_ACC_MAX_FRAME = 16 };
    if (buf == NULL || len == 0 || (uint16_t)(len + 2) > BMI_ACC_MAX_FRAME) {
        return DRV_ERR_PARAM;
    }

    uint8_t tx[BMI_ACC_MAX_FRAME] = { 0 };
    uint8_t rx[BMI_ACC_MAX_FRAME] = { 0 };

    tx[0] = (uint8_t)(reg | BMI_SPI_READ_BIT);
    /* tx[1..len+1] 已为零（哑字节 + 时钟字节） */

    HAL_StatusTypeDef hs = HAL_SPI_TransmitReceive(&hspi1,
                                                   tx, rx,
                                                   (uint16_t)(len + 2u),
                                                   HAL_TIMEOUT_MS);
    if (hs != HAL_OK) {
        return Driver_MapHalStatus(hs);
    }
    /* rx[0] = 地址回显，rx[1] = 哑字节；有效数据从 rx[2] 开始 */
    for (uint16_t i = 0; i < len; ++i) {
        buf[i] = rx[i + 2u];
    }
    return DRV_OK;
}

/**
 * @brief 从陀螺仪寄存器 reg 开始读取 len 字节。
 *
 * 发送 len+1 字节，仅丢弃地址回显（无哑字节）。
 */
static Driver_Status bmi_gyro_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    enum { BMI_GYRO_MAX_FRAME = 16 };
    if (buf == NULL || len == 0 || (uint16_t)(len + 1) > BMI_GYRO_MAX_FRAME) {
        return DRV_ERR_PARAM;
    }

    uint8_t tx[BMI_GYRO_MAX_FRAME] = { 0 };
    uint8_t rx[BMI_GYRO_MAX_FRAME] = { 0 };

    tx[0] = (uint8_t)(reg | BMI_SPI_READ_BIT);

    HAL_StatusTypeDef hs = HAL_SPI_TransmitReceive(&hspi1,
                                                   tx, rx,
                                                   (uint16_t)(len + 1u),
                                                   HAL_TIMEOUT_MS);
    if (hs != HAL_OK) {
        return Driver_MapHalStatus(hs);
    }
    for (uint16_t i = 0; i < len; ++i) {
        buf[i] = rx[i + 1u];
    }
    return DRV_OK;
}

/**
 * @brief 向寄存器 reg 写入单字节 value。
 *
 * 两个芯片的写操作均为 2 字节帧，无需哑字节。地址字节 MSB 清零（写操作）。
 */
static Driver_Status bmi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = (uint8_t)(reg & 0x7Fu);
    tx[1] = value;
    HAL_StatusTypeDef hs = HAL_SPI_Transmit(&hspi1, tx, 2u, HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}

/* ------------------------------------------------------------------ */
/* 各芯片单字节寄存器辅助函数（片选由这些函数管理）                   */
/* ------------------------------------------------------------------ */

static Driver_Status bmi_acc_read_reg(uint8_t reg, uint8_t *value)
{
    bmi_acc_select();
    Driver_Status st = bmi_acc_read(reg, value, 1u);
    bmi_deselect_all();
    return st;
}

static Driver_Status bmi_gyro_read_reg(uint8_t reg, uint8_t *value)
{
    bmi_gyro_select();
    Driver_Status st = bmi_gyro_read(reg, value, 1u);
    bmi_deselect_all();
    return st;
}

static Driver_Status bmi_acc_write_reg(uint8_t reg, uint8_t value)
{
    bmi_acc_select();
    Driver_Status st = bmi_write_reg(reg, value);
    bmi_deselect_all();
    return st;
}

static Driver_Status bmi_gyro_write_reg(uint8_t reg, uint8_t value)
{
    bmi_gyro_select();
    Driver_Status st = bmi_write_reg(reg, value);
    bmi_deselect_all();
    return st;
}

/**
 * @brief 让加速度计从默认 I2C 接口切到 SPI 接口。
 *
 * BMI088 加速度计上电/复位后需要一次 CSB1 上升沿；用一次丢弃结果的
 * Chip-ID 读取产生完整 SPI 事务，随后再正式读取寄存器。
 */
static Driver_Status bmi_acc_enter_spi_mode(void)
{
    uint8_t dummy = 0u;

    bmi_deselect_all();
    bmi_delay_ms(BMI_ACC_SPI_WAKE_DELAY_MS);

    bmi_acc_select();
    Driver_Status st = bmi_acc_read(BMI_ACC_REG_CHIP_ID, &dummy, 1u);
    bmi_deselect_all();
    bmi_delay_ms(BMI_ACC_SPI_WAKE_DELAY_MS);

    return st;
}

static Driver_Status bmi_acc_read_id_retry(uint8_t *acc_id)
{
    if (acc_id == NULL) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = DRV_OK;
    for (uint32_t i = 0u; i < BMI_ACC_ID_RETRY_COUNT; ++i) {
        st = bmi_acc_read_reg(BMI_ACC_REG_CHIP_ID, acc_id);
        if (st != DRV_OK) {
            return st;
        }
        if (*acc_id == BMI_ACC_CHIP_ID) {
            return DRV_OK;
        }
        bmi_delay_ms(BMI_ACC_ID_RETRY_DELAY_MS);
    }

    return DRV_ERR_ID;
}

/* ------------------------------------------------------------------ */
/* 公共 API                                                            */
/* ------------------------------------------------------------------ */

Driver_Status BMI088_Init(void)
{
    Driver_Status st = DRV_OK;
    uint8_t acc_id = 0u;
    uint8_t gyro_id = 0u;

    s_initialized = false;
    st = bmi_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* 确保两路片选初始为高电平，防止 SPI 外设重配置时干扰进行中的事务 */
    bmi_deselect_all();

    /* 步骤 1：修正 CubeMX 生成的 DataSize 异常。
     * CloudFlyIMU.ioc 将 SPI1 配置为 SPI_DATASIZE_4BIT，对 BMI088 无效。
     * 在实际句柄上覆盖该字段并重新初始化外设，确保字节交换前配置正确。 */
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_StatusTypeDef hs = HAL_SPI_Init(&hspi1);
    if (hs != HAL_OK) {
        st = Driver_MapHalStatus(hs);
        goto fail;
    }

    /* 步骤 2：产生 CSB1 上升沿，让加速度计进入 SPI 模式。 */
    st = bmi_acc_enter_spi_mode();
    if (st != DRV_OK) {
        goto fail;
    }

    /* 步骤 3：加速度计软复位（数据手册 §5.4）+ 等待 50 ms */
    st = bmi_acc_write_reg(BMI_ACC_REG_SOFTRESET, BMI_ACC_SOFTRESET_VALUE);
    if (st != DRV_OK) {
        goto fail;
    }
    bmi_delay_ms(BMI_ACC_SOFT_RESET_DELAY_MS);

    /* 步骤 4：哑读 Chip-ID 以唤醒加速度计 SPI 引擎。
     * 软复位后首次 SPI 访问返回垃圾数据；需一次带片选切换的事务
     * 使芯片从 I2C 模式切换到 SPI 模式。丢弃结果。 */
    st = bmi_acc_enter_spi_mode();
    if (st != DRV_OK) {
        goto fail;
    }

    /* 步骤 5：验证加速度计 Chip ID = 0x1E */
    st = bmi_acc_read_id_retry(&acc_id);
    if (st != DRV_OK) {
        goto fail;
    }

    /* 步骤 6：验证陀螺仪 Chip ID = 0x0F */
    st = bmi_gyro_read_reg(BMI_GYRO_REG_CHIP_ID, &gyro_id);
    if (st != DRV_OK) {
        goto fail;
    }
    if (gyro_id != BMI_GYRO_CHIP_ID) {
        st = DRV_ERR_ID;
        goto fail;
    }

    /* 步骤 7：配置加速度计
     *   0x7D ← 0x04（加速度计开启）
     *   等待至少 5 ms
     *   0x7C ← 0x00（退出省电，进入 Active）
     *   等待约 50 ms
     *   0x41 ← 0x01（量程 = ±6 g）
     *   0x40 ← 0xAB（ODR = 800 Hz，Normal BW）*/
    st = bmi_acc_write_reg(BMI_ACC_REG_PWR_CTRL, BMI_ACC_PWR_CTRL_ENABLE);
    if (st != DRV_OK) { goto fail; }
    bmi_delay_ms(BMI_ACC_PWR_CTRL_DELAY_MS);

    st = bmi_acc_write_reg(BMI_ACC_REG_PWR_CONF, BMI_ACC_PWR_CONF_ACTIVE);
    if (st != DRV_OK) { goto fail; }
    bmi_delay_ms(BMI_ACC_POWER_UP_DELAY_MS);

    st = bmi_acc_write_reg(BMI_ACC_REG_RANGE, BMI_ACC_RANGE_6G);
    if (st != DRV_OK) { goto fail; }

    st = bmi_acc_write_reg(BMI_ACC_REG_CONF, BMI_ACC_CONF_NORMAL_800HZ);
    if (st != DRV_OK) { goto fail; }

    /* 步骤 8：配置陀螺仪
     *   0x0F ← 0x00（量程 = ±2000 °/s）
     *   0x10 ← 0x02（ODR 1000 Hz / BW 116 Hz）*/
    st = bmi_gyro_write_reg(BMI_GYRO_REG_RANGE, BMI_GYRO_RANGE_2000DPS);
    if (st != DRV_OK) { goto fail; }

    st = bmi_gyro_write_reg(BMI_GYRO_REG_BANDWIDTH, BMI_GYRO_BW_1000_116);
    if (st != DRV_OK) { goto fail; }

    /* 所有配置成功——开启门控标志 */
    s_initialized = true;

    bmi_deselect_all();
    bmi_bus_unlock();
    return DRV_OK;

fail:
    bmi_deselect_all();
    bmi_bus_unlock();
    return st;
}

Driver_Status BMI088_ReadAccel(float *ax, float *ay, float *az)
{
    if (ax == NULL || ay == NULL || az == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmi_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    uint8_t buf[6] = { 0 };
    bmi_acc_select();
    st = bmi_acc_read(BMI_ACC_REG_DATA, buf, sizeof(buf));
    bmi_deselect_all();
    bmi_bus_unlock();
    if (st != DRV_OK) {
        return st;
    }

    int16_t raw_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t raw_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t raw_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    *ax = (float)raw_x / BMI_ACC_LSB_PER_G * BMI_GRAVITY_MS2;
    *ay = (float)raw_y / BMI_ACC_LSB_PER_G * BMI_GRAVITY_MS2;
    *az = (float)raw_z / BMI_ACC_LSB_PER_G * BMI_GRAVITY_MS2;
    return DRV_OK;
}

Driver_Status BMI088_ReadGyro(float *gx, float *gy, float *gz)
{
    if (gx == NULL || gy == NULL || gz == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmi_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    uint8_t buf[6] = { 0 };
    bmi_gyro_select();
    st = bmi_gyro_read(BMI_GYRO_REG_DATA, buf, sizeof(buf));
    bmi_deselect_all();
    bmi_bus_unlock();
    if (st != DRV_OK) {
        return st;
    }

    int16_t raw_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t raw_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t raw_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    *gx = (float)raw_x / BMI_GYRO_LSB_PER_DPS * BMI_DEG_TO_RAD;
    *gy = (float)raw_y / BMI_GYRO_LSB_PER_DPS * BMI_DEG_TO_RAD;
    *gz = (float)raw_z / BMI_GYRO_LSB_PER_DPS * BMI_DEG_TO_RAD;
    return DRV_OK;
}

Driver_Status BMI088_GetChipID(uint8_t *acc_id, uint8_t *gyro_id)
{
    if (acc_id == NULL || gyro_id == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmi_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    st = bmi_acc_read_reg(BMI_ACC_REG_CHIP_ID, acc_id);
    if (st != DRV_OK) {
        bmi_deselect_all();
        bmi_bus_unlock();
        return st;
    }

    st = bmi_gyro_read_reg(BMI_GYRO_REG_CHIP_ID, gyro_id);
    bmi_deselect_all();
    bmi_bus_unlock();
    return st;
}
