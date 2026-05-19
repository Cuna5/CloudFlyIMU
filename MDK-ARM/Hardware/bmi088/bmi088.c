/**
 * @file    bmi088.c
 * @brief   BMI088 6-axis IMU driver (SPI1, dual CS PC4/PC5).
 *
 * Spec: .kiro/specs/hardware-base-drivers/{requirements,design,tasks}.md
 *       (task 5.1, Requirements 2.1..2.12, 8.6)
 *
 * Bus framing:
 *   - SPI peripheral           = SPI1 (`hspi1`).
 *   - Accelerometer CS         = PC4 (`BMI_ACC_CS_Pin`, active-low).
 *   - Gyroscope CS             = PC5 (`BMI_GYRO_CS_Pin`, active-low).
 *
 * Read protocol differences between the two dies (BMI088 datasheet
 * §6.1.2 / §6.1.3):
 *   - Accelerometer SPI read drops one dummy byte after the address byte.
 *     A read transaction therefore emits `len + 2` bytes:
 *         [ reg | 0x80, 0x00 (dummy), 0x00 ... 0x00 (len bytes) ]
 *     and we keep `rx[2 .. 2 + len - 1]`.
 *   - Gyroscope SPI read has no dummy, so a transaction emits `len + 1`
 *     bytes:
 *         [ reg | 0x80, 0x00 ... 0x00 (len bytes) ]
 *     and we keep `rx[1 .. len]`.
 *   - Single-register writes are exactly 2 bytes: `[reg, value]`.
 *
 * Concurrency: every public API acquires `SPIMutexHandle` via
 * `osMutexAcquire(SPIMutexHandle, HAL_TIMEOUT_MS)` while the FreeRTOS
 * scheduler is running, and releases it (along with both CS lines) on
 * every return path. Before scheduler start the lock is bypassed because
 * boot is single-threaded.
 *
 * HAL return codes are mapped through `Driver_MapHalStatus` so the
 * caller observes the unified `Driver_Status` enum.
 */
/* hardware.h first: it owns the Driver_Status enum and HAL_TIMEOUT_MS, and
 * re-exports bmi088.h as part of the aggregate facade. Including the
 * aggregate header keeps this driver's translation unit consistent with the
 * sibling drivers (bmp280.c, ist8310.c) and with what application code sees. */
#include "hardware.h"   /* Driver_Status, HAL_TIMEOUT_MS, Driver_MapHalStatus */
#include "spi.h"        /* extern hspi1 */
#include "main.h"       /* BMI_ACC_CS_Pin, BMI_GYRO_CS_Pin (host shim mirrors these) */

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* M_PI is not part of strict ISO C; define a portable fallback that
 * matches the value <math.h> would supply on POSIX systems. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* Shared SPI bus mutex (created by CubeMX in Core/Src/freertos.c).    */
/* The host test harness defines its own storage in mock_handles.c.   */
/* ------------------------------------------------------------------ */
extern osMutexId_t SPIMutexHandle;

/* ------------------------------------------------------------------ */
/* BMI088 register map — only addresses used by this driver are named. */
/* ------------------------------------------------------------------ */

/* --- Accelerometer (slave A) --- */
#define BMI_ACC_REG_CHIP_ID         0x00u   /* expect 0x1E                                  */
#define BMI_ACC_REG_DATA            0x12u   /* burst start: ACC_X_LSB..ACC_Z_MSB (6 bytes)  */
#define BMI_ACC_REG_CONF            0x40u   /* ODR / BW                                     */
#define BMI_ACC_REG_RANGE           0x41u   /* full-scale range                             */
#define BMI_ACC_REG_PWR_CTRL        0x7Du   /* power control (normal / suspend)             */
#define BMI_ACC_REG_SOFTRESET       0x7Eu   /* write 0xB6 to soft-reset                     */

/* --- Gyroscope (slave G) --- */
#define BMI_GYRO_REG_CHIP_ID        0x00u   /* expect 0x0F                                  */
#define BMI_GYRO_REG_DATA           0x02u   /* burst start: RATE_X_LSB..RATE_Z_MSB (6 bytes)*/
#define BMI_GYRO_REG_RANGE          0x0Fu   /* full-scale range                             */
#define BMI_GYRO_REG_BANDWIDTH      0x10u   /* ODR / BW                                     */

/* Expected chip-ID values. */
#define BMI_ACC_CHIP_ID             0x1Eu
#define BMI_GYRO_CHIP_ID            0x0Fu

/* Register values used during init (kept symbolic for traceability). */
#define BMI_ACC_SOFTRESET_VALUE     0xB6u   /* same as BMP280 soft-reset value, by design */
#define BMI_ACC_RANGE_6G            0x01u   /* ±6 g                                       */
#define BMI_ACC_CONF_OSR4_800HZ     0xABu   /* OSR4, ODR = 800 Hz                         */
#define BMI_ACC_PWR_CTRL_NORMAL     0x04u   /* accelerometer ON                           */
#define BMI_GYRO_RANGE_2000DPS      0x01u   /* ±2000 °/s                                  */
#define BMI_GYRO_BW_1000_116        0x02u   /* ODR 1000 Hz / 3-dB BW 116 Hz               */

/* SPI access flags. */
#define BMI_SPI_READ_BIT            0x80u   /* OR'd into the address byte for reads */

/* Soft-reset settling time (datasheet §5.4): typical 30 ms; spec 50 ms. */
#define BMI_ACC_SOFT_RESET_DELAY_MS 50u

/* ------------------------------------------------------------------ */
/* Sensitivity / unit conversion                                       */
/* ------------------------------------------------------------------ */

/** Accelerometer sensitivity at ±6 g full scale: 5460 LSB/g (datasheet §5.3). */
#define BMI_ACC_LSB_PER_G           5460.0f
/** Standard gravity, m/s². */
#define BMI_GRAVITY_MS2             9.80665f
/** Gyro sensitivity at ±2000 °/s full scale: 16.384 LSB/(°/s) (datasheet §5.2). */
#define BMI_GYRO_LSB_PER_DPS        16.384f
/** Degrees → radians scaling factor. */
#define BMI_DEG_TO_RAD              ((float)M_PI / 180.0f)

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

/** Set true at the end of a successful `BMI088_Init`; gates all reads. */
static bool s_initialized = false;

/* ------------------------------------------------------------------ */
/* CS line helpers                                                     */
/*                                                                      */
/* On the real target `BMI_ACC_CS_Pin` / `BMI_GYRO_CS_Pin` come from   */
/* `main.h`; on host they are provided by `stm32h7xx_hal_gpio.h`.       */
/* ------------------------------------------------------------------ */

/** Drive PC4 low + PC5 high to select the accelerometer die. */
static inline void bmi_acc_select(void)
{
    HAL_GPIO_WritePin(GPIOC, BMI_ACC_CS_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, BMI_GYRO_CS_Pin, GPIO_PIN_SET);
}

/** Drive PC4 high + PC5 low to select the gyroscope die. */
static inline void bmi_gyro_select(void)
{
    HAL_GPIO_WritePin(GPIOC, BMI_ACC_CS_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, BMI_GYRO_CS_Pin, GPIO_PIN_RESET);
}

/** Drive both CS lines high (idle / deselect). Always called on every
 *  exit path of every transaction so that no die is left selected if a
 *  HAL error occurs mid-transfer. */
static inline void bmi_deselect_all(void)
{
    HAL_GPIO_WritePin(GPIOC, BMI_ACC_CS_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, BMI_GYRO_CS_Pin, GPIO_PIN_SET);
}

/* ------------------------------------------------------------------ */
/* RTOS helpers                                                        */
/* ------------------------------------------------------------------ */

static inline bool bmi_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief Acquire the shared SPI bus mutex if the scheduler is running.
 *
 * Returns @ref DRV_OK if the lock was taken (or skipped because the
 * scheduler is not yet running), @ref DRV_ERR_TIMEOUT otherwise.
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

/** Release the shared SPI bus mutex; no-op if the scheduler is inactive. */
static void bmi_bus_unlock(void)
{
    if (bmi_scheduler_running()) {
        (void)osMutexRelease(SPIMutexHandle);
    }
}

/** RTOS-aware delay: `osDelay` when the scheduler is running, `HAL_Delay`
 *  during boot. */
static void bmi_delay_ms(uint32_t ms)
{
    if (bmi_scheduler_running()) {
        (void)osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

/* ------------------------------------------------------------------ */
/* Low-level SPI primitives                                            */
/*                                                                      */
/* These helpers assume:                                                */
/*   - The bus mutex is already held by the caller.                     */
/*   - The relevant CS line is already driven low by the caller.        */
/* They never touch CS lines themselves so the caller can keep CS low   */
/* across multiple writes (e.g. burst init sequences).                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Read `len` bytes starting at register `reg` from the
 *        accelerometer die.
 *
 * Emits `len + 2` SPI bytes, discards the address echo and dummy byte.
 *
 * @pre `reg + 0x80` fits in 8 bits and `len + 2 <= sizeof rx_buf`.
 */
static Driver_Status bmi_acc_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* Maximum payload used by this driver is 6 bytes (data burst), so
     * `len + 2 <= 8`. We cap at 16 to leave room for future growth. */
    enum { BMI_ACC_MAX_FRAME = 16 };
    if (buf == NULL || len == 0 || (uint16_t)(len + 2) > BMI_ACC_MAX_FRAME) {
        return DRV_ERR_PARAM;
    }

    uint8_t tx[BMI_ACC_MAX_FRAME] = { 0 };
    uint8_t rx[BMI_ACC_MAX_FRAME] = { 0 };

    tx[0] = (uint8_t)(reg | BMI_SPI_READ_BIT);
    /* tx[1..len+1] already zero (dummy + clocking bytes). */

    HAL_StatusTypeDef hs = HAL_SPI_TransmitReceive(&hspi1,
                                                   tx, rx,
                                                   (uint16_t)(len + 2u),
                                                   HAL_TIMEOUT_MS);
    if (hs != HAL_OK) {
        return Driver_MapHalStatus(hs);
    }
    /* rx[0] = address echo, rx[1] = dummy; payload starts at rx[2]. */
    for (uint16_t i = 0; i < len; ++i) {
        buf[i] = rx[i + 2u];
    }
    return DRV_OK;
}

/**
 * @brief Read `len` bytes starting at register `reg` from the gyroscope die.
 *
 * Emits `len + 1` SPI bytes, discards the address echo only (no dummy).
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
 * @brief Write a single byte `value` to register `reg`.
 *
 * The same 2-byte frame works for both dies because writes never need a
 * dummy byte. The MSB of the address byte is left clear (= write).
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
/* Per-die single-byte register helpers (CS is owned by these helpers) */
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

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

Driver_Status BMI088_Init(void)
{
    Driver_Status st = bmi_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* Make sure both CS lines start high so the reconfiguration of the
     * SPI peripheral cannot corrupt an in-flight transaction. */
    bmi_deselect_all();

    /* ---------------------------------------------------------------- */
    /* Step 1: patch CubeMX-generated DataSize anomaly.                  */
    /* `CloudFlyIMU.ioc` configures SPI1 with `SPI_DATASIZE_4BIT`, which */
    /* is invalid for the BMI088. Override the field on the live handle */
    /* and re-init the peripheral before any byte is exchanged.         */
    /* ---------------------------------------------------------------- */
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_StatusTypeDef hs = HAL_SPI_Init(&hspi1);
    if (hs != HAL_OK) {
        bmi_deselect_all();
        bmi_bus_unlock();
        return Driver_MapHalStatus(hs);
    }

    /* ---------------------------------------------------------------- */
    /* Step 2: accelerometer soft-reset (datasheet §5.4) + 50 ms wait.  */
    /* ---------------------------------------------------------------- */
    st = bmi_acc_write_reg(BMI_ACC_REG_SOFTRESET, BMI_ACC_SOFTRESET_VALUE);
    if (st != DRV_OK) {
        bmi_deselect_all();
        bmi_bus_unlock();
        return st;
    }
    bmi_delay_ms(BMI_ACC_SOFT_RESET_DELAY_MS);

    /* ---------------------------------------------------------------- */
    /* Step 3: dummy Chip-ID read to wake the accelerometer SPI engine. */
    /* The first SPI access after soft-reset returns garbage; per the   */
    /* datasheet the slave needs one transaction with CS toggled to     */
    /* leave I2C and enter SPI mode. Discard the result.                */
    /* ---------------------------------------------------------------- */
    {
        uint8_t dummy = 0;
        (void)bmi_acc_read_reg(BMI_ACC_REG_CHIP_ID, &dummy);
    }

    /* ---------------------------------------------------------------- */
    /* Step 4: verify accelerometer Chip ID = 0x1E.                     */
    /* ---------------------------------------------------------------- */
    uint8_t acc_id = 0;
    st = bmi_acc_read_reg(BMI_ACC_REG_CHIP_ID, &acc_id);
    if (st != DRV_OK) {
        bmi_deselect_all();
        bmi_bus_unlock();
        return st;
    }
    if (acc_id != BMI_ACC_CHIP_ID) {
        bmi_deselect_all();
        bmi_bus_unlock();
        return DRV_ERR_ID;
    }

    /* ---------------------------------------------------------------- */
    /* Step 5: verify gyroscope Chip ID = 0x0F.                         */
    /* ---------------------------------------------------------------- */
    uint8_t gyro_id = 0;
    st = bmi_gyro_read_reg(BMI_GYRO_REG_CHIP_ID, &gyro_id);
    if (st != DRV_OK) {
        bmi_deselect_all();
        bmi_bus_unlock();
        return st;
    }
    if (gyro_id != BMI_GYRO_CHIP_ID) {
        bmi_deselect_all();
        bmi_bus_unlock();
        return DRV_ERR_ID;
    }

    /* ---------------------------------------------------------------- */
    /* Step 6: configure accelerometer.                                 */
    /*   0x41 ← 0x01 (range = ±6 g)                                     */
    /*   0x40 ← 0xAB (ODR = 800 Hz, BW = OSR4)                          */
    /*   0x7D ← 0x04 (power-control = Normal)                           */
    /* ---------------------------------------------------------------- */
    st = bmi_acc_write_reg(BMI_ACC_REG_RANGE,    BMI_ACC_RANGE_6G);
    if (st != DRV_OK) { bmi_deselect_all(); bmi_bus_unlock(); return st; }
    st = bmi_acc_write_reg(BMI_ACC_REG_CONF,     BMI_ACC_CONF_OSR4_800HZ);
    if (st != DRV_OK) { bmi_deselect_all(); bmi_bus_unlock(); return st; }
    st = bmi_acc_write_reg(BMI_ACC_REG_PWR_CTRL, BMI_ACC_PWR_CTRL_NORMAL);
    if (st != DRV_OK) { bmi_deselect_all(); bmi_bus_unlock(); return st; }

    /* ---------------------------------------------------------------- */
    /* Step 7: configure gyroscope.                                     */
    /*   0x0F ← 0x01 (range = ±2000 °/s)                                */
    /*   0x10 ← 0x02 (ODR 1000 Hz / BW 116 Hz)                          */
    /* ---------------------------------------------------------------- */
    st = bmi_gyro_write_reg(BMI_GYRO_REG_RANGE,     BMI_GYRO_RANGE_2000DPS);
    if (st != DRV_OK) { bmi_deselect_all(); bmi_bus_unlock(); return st; }
    st = bmi_gyro_write_reg(BMI_GYRO_REG_BANDWIDTH, BMI_GYRO_BW_1000_116);
    if (st != DRV_OK) { bmi_deselect_all(); bmi_bus_unlock(); return st; }

    /* All configuration succeeded — flip the gate. */
    s_initialized = true;

    bmi_deselect_all();
    bmi_bus_unlock();
    return DRV_OK;
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
