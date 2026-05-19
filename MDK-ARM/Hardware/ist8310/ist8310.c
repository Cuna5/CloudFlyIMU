/**
 * @file    ist8310.c
 * @brief   IST8310 magnetometer driver (I2C1, 7-bit address 0x0E).
 *
 * Spec: .kiro/specs/hardware-base-drivers/{requirements,design,tasks}.md
 *       (task 4.1, Requirements 3.1..3.8, 8.6)
 *
 * I2C frame layout used here:
 *   - Slave 7-bit address      = 0x0E.
 *   - HAL_I2C_Mem_* devAddress = 0x0E << 1 = 0x1C.
 *   - Memory address size      = 1 byte (`I2C_MEMADD_SIZE_8BIT`).
 *
 * All I2C transfers are wrapped in `osMutexAcquire(I2CMutexHandle, ...)`
 * when the FreeRTOS scheduler is running (the same lock used by
 * `bmp280.c`, since both devices share I2C1). When the scheduler is not
 * yet running (e.g. `Hardware_Init` called from `main` before
 * `osKernelStart`), the mutex is bypassed and HAL APIs are called
 * directly.
 *
 * HAL return codes are mapped through `Driver_MapHalStatus` so that the
 * caller observes the unified `Driver_Status` enum.
 */
/* hardware.h first: it owns the Driver_Status enum and HAL_TIMEOUT_MS, and
 * re-exports ist8310.h as part of the aggregate facade. Including the
 * aggregate header keeps this driver's translation unit consistent with the
 * sibling drivers (bmp280.c, bmi088.c) and with what application code sees. */
#include "hardware.h"   /* Driver_Status, HAL_TIMEOUT_MS, Driver_MapHalStatus */
#include "i2c.h"        /* extern hi2c1 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Shared I2C bus mutex (created by CubeMX in Core/Src/freertos.c)     */
/* ------------------------------------------------------------------ */
extern osMutexId_t I2CMutexHandle;

/* ------------------------------------------------------------------ */
/* Device-specific constants                                           */
/* ------------------------------------------------------------------ */

/** 7-bit slave address — datasheet §7.1 (CAD0/CAD1 strapped low). */
#define IST8310_I2C_ADDR_7BIT       0x0Eu

/** HAL_I2C_* call expects the address left-shifted by one (R/W flag bit). */
#define IST8310_I2C_ADDR_HAL        ((uint16_t)(IST8310_I2C_ADDR_7BIT << 1))   /* 0x1C */

/** Expected Chip ID (WAI) value — datasheet §8.1. */
#define IST8310_CHIP_ID             0x10u

/* Register map — only the addresses used here are named. */
#define IST8310_REG_WAI             0x00u   /* Who-Am-I (Chip ID) */
#define IST8310_REG_DATA_X_L        0x03u   /* First of 6 burst-readable data bytes */
#define IST8310_REG_CTRL1           0x0Au   /* Control 1 — write 0x01 to start single measurement */
#define IST8310_REG_CTRL2           0x0Bu   /* Control 2 — DRDY / interrupt enable */
#define IST8310_REG_AVG_CTRL        0x41u   /* Average control */
#define IST8310_REG_PD_CTRL         0x42u   /* Pulse duration / performance */

/* Control values used during init / read. */
#define IST8310_CTRL2_INIT_VALUE    0xC0u
#define IST8310_AVG_INIT_VALUE      0x09u
#define IST8310_PD_INIT_VALUE       0xC0u
#define IST8310_CTRL1_SINGLE_MEAS   0x01u

/** Single-shot measurement settling time (datasheet §6.1, typical 6 ms). */
#define IST8310_MEAS_DELAY_MS       6u

/** Sensitivity: 0.3 µT per LSB (datasheet §6.1). */
#define IST8310_LSB_TO_UT           (0.3f)

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

/** Set true at the end of a successful `IST8310_Init`; gates all reads. */
static bool s_initialized = false;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static inline bool ist8310_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief Acquire the shared I2C bus mutex if the scheduler is running.
 *
 * Returns @ref DRV_OK if the lock was taken (or skipped because the
 * scheduler is not yet running), @ref DRV_ERR_TIMEOUT otherwise.
 */
static Driver_Status ist8310_bus_lock(void)
{
    if (!ist8310_scheduler_running()) {
        return DRV_OK;
    }
    if (osMutexAcquire(I2CMutexHandle, HAL_TIMEOUT_MS) != osOK) {
        return DRV_ERR_TIMEOUT;
    }
    return DRV_OK;
}

/** Release the shared I2C bus mutex; no-op if the scheduler is inactive. */
static void ist8310_bus_unlock(void)
{
    if (ist8310_scheduler_running()) {
        (void)osMutexRelease(I2CMutexHandle);
    }
}

/** Burst-read `len` bytes starting at register `reg` into `buf`. */
static Driver_Status ist8310_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Read(&hi2c1,
                                            IST8310_I2C_ADDR_HAL,
                                            (uint16_t)reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf,
                                            len,
                                            HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}

/** Write a single byte `value` to register `reg`. */
static Driver_Status ist8310_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t payload = value;
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Write(&hi2c1,
                                             IST8310_I2C_ADDR_HAL,
                                             (uint16_t)reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             &payload,
                                             1u,
                                             HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}

/** RTOS-aware ≥ ms delay used between trigger and burst read. */
static void ist8310_delay_ms(uint32_t ms)
{
    if (ist8310_scheduler_running()) {
        (void)osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

Driver_Status IST8310_Init(void)
{
    Driver_Status st = ist8310_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    uint8_t chip_id = 0;
    st = ist8310_read(IST8310_REG_WAI, &chip_id, 1u);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }
    if (chip_id != IST8310_CHIP_ID) {
        ist8310_bus_unlock();
        return DRV_ERR_ID;
    }

    st = ist8310_write_reg(IST8310_REG_CTRL2, IST8310_CTRL2_INIT_VALUE);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    st = ist8310_write_reg(IST8310_REG_AVG_CTRL, IST8310_AVG_INIT_VALUE);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    st = ist8310_write_reg(IST8310_REG_PD_CTRL, IST8310_PD_INIT_VALUE);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    s_initialized = true;
    ist8310_bus_unlock();
    return DRV_OK;
}

Driver_Status IST8310_ReadMag(float *mx, float *my, float *mz)
{
    if (mx == NULL || my == NULL || mz == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = ist8310_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* 1) Trigger a single-shot conversion. */
    st = ist8310_write_reg(IST8310_REG_CTRL1, IST8310_CTRL1_SINGLE_MEAS);
    if (st != DRV_OK) {
        ist8310_bus_unlock();
        return st;
    }

    /* 2) Wait at least 6 ms for the conversion to complete. */
    ist8310_delay_ms(IST8310_MEAS_DELAY_MS);

    /* 3) Burst-read 6 bytes from 0x03..0x08 (X, Y, Z, little-endian). */
    uint8_t buf[6] = { 0 };
    st = ist8310_read(IST8310_REG_DATA_X_L, buf, sizeof(buf));
    ist8310_bus_unlock();
    if (st != DRV_OK) {
        return st;
    }

    /* 4) Reconstruct signed 16-bit values, then scale to microtesla. */
    int16_t raw_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t raw_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t raw_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    *mx = (float)raw_x * IST8310_LSB_TO_UT;
    *my = (float)raw_y * IST8310_LSB_TO_UT;
    *mz = (float)raw_z * IST8310_LSB_TO_UT;
    return DRV_OK;
}

Driver_Status IST8310_GetChipID(uint8_t *id)
{
    if (id == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = ist8310_bus_lock();
    if (st != DRV_OK) {
        return st;
    }

    st = ist8310_read(IST8310_REG_WAI, id, 1u);
    ist8310_bus_unlock();
    return st;
}
