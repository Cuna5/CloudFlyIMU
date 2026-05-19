/**
 * @file    bmp280.c
 * @brief   BMP280 temperature/pressure driver (I2C1).
 *
 * Implements the BMP280 driver per design.md `## Components and Interfaces
 * > bmp280/bmp280` and Requirement 4 (4.1..4.10) plus the cross-cutting
 * `DRV_ERR_NOT_INIT` rule (Requirement 8.6).
 *
 * Bus: I2C1 (`hi2c1`, declared in `i2c.h` on target / `tests/include/i2c.h`
 *       on host). The HAL takes a left-shifted 7-bit address.
 *
 * Mutex: every public API acquires `I2CMutexHandle` via
 *        `osMutexAcquire(I2CMutexHandle, HAL_TIMEOUT_MS)` while the FreeRTOS
 *        scheduler is running; before scheduler start the lock is skipped
 *        (single-threaded boot).
 *
 * Validates: Requirements 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 4.10, 8.6.
 */

#include "hardware.h"      /* Driver_Status, HAL_TIMEOUT_MS, sub-headers     */
#include "i2c.h"           /* hi2c1                                          */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


/* ================================================================== */
/* Module-private constants                                            */
/* ================================================================== */

/** Default 7-bit I2C address (SDO tied to GND). */
#define BMP280_ADDR7_DEFAULT    0x76u
/** Alternate 7-bit I2C address (SDO tied to VDDIO). */
#define BMP280_ADDR7_ALT        0x77u

/** Expected ChipID returned by register 0xD0. */
#define BMP280_CHIP_ID          0x58u

/** Register addresses (datasheet table 18). */
#define BMP280_REG_CALIB_T1_LSB 0x88u   /* start of 24-byte calibration block */
#define BMP280_REG_ID           0xD0u
#define BMP280_REG_RESET        0xE0u
#define BMP280_REG_CTRL_MEAS    0xF4u
#define BMP280_REG_CONFIG       0xF5u
#define BMP280_REG_PRES_MSB     0xF7u   /* 0xF7..0xF9 holds 20-bit pressure   */
#define BMP280_REG_TEMP_MSB     0xFAu   /* 0xFA..0xFC holds 20-bit temperature */

#define BMP280_RESET_KEY        0xB6u
#define BMP280_CTRL_MEAS_VALUE  0x57u   /* osrs_t=×2, osrs_p=×16, mode=Normal */
#define BMP280_CONFIG_VALUE     0x90u   /* t_sb=62.5 ms, filter=16            */

#define BMP280_RESET_WAIT_MS    5u


/* ================================================================== */
/* Module-private state                                                */
/* ================================================================== */

/**
 * @brief BMP280 calibration coefficients parsed from registers 0x88..0x9F.
 *
 * Per datasheet table 17, dig_T1 and dig_P1 are unsigned 16-bit; the rest
 * are signed 16-bit. The block is little-endian on the wire.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

static uint8_t        s_addr7        = BMP280_ADDR7_DEFAULT;
static bool           s_initialized  = false;
static bmp280_calib_t s_calib;
static int32_t        s_t_fine       = 0;
static bool           s_t_fine_valid = false;


/* ================================================================== */
/* Mutex / kernel helpers                                              */
/* ================================================================== */

/* The shared I2C bus mutex is created in `Core/Src/freertos.c`
 * (`MX_FREERTOS_Init`); we simply reference it. On the host test build the
 * mock harness must provide its own definition. */
extern osMutexId_t I2CMutexHandle;

static inline bool bmp280_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief Acquire the I2C bus mutex if the scheduler is running.
 *
 * @retval DRV_OK             Lock taken, or scheduler not yet running.
 * @retval DRV_ERR_TIMEOUT    `osMutexAcquire` returned non-osOK.
 */
static Driver_Status bmp280_lock(void)
{
    if (!bmp280_scheduler_running()) {
        return DRV_OK;
    }
    if (osMutexAcquire(I2CMutexHandle, HAL_TIMEOUT_MS) != osOK) {
        return DRV_ERR_TIMEOUT;
    }
    return DRV_OK;
}

static void bmp280_unlock(void)
{
    if (bmp280_scheduler_running()) {
        (void)osMutexRelease(I2CMutexHandle);
    }
}

static void bmp280_delay_ms(uint32_t ms)
{
    if (bmp280_scheduler_running()) {
        (void)osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}


/* ================================================================== */
/* Low-level I2C wrappers (caller already holds the bus mutex)         */
/* ================================================================== */

static inline uint16_t bmp280_dev_addr(void)
{
    /* HAL expects the 7-bit address shifted left by one (R/W bit slot). */
    return (uint16_t)((uint16_t)s_addr7 << 1);
}

static Driver_Status bmp280_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Read(&hi2c1, bmp280_dev_addr(),
                                            reg, I2C_MEMADD_SIZE_8BIT,
                                            buf, len, HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}

static Driver_Status bmp280_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t v = value;
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Write(&hi2c1, bmp280_dev_addr(),
                                             reg, I2C_MEMADD_SIZE_8BIT,
                                             &v, 1u, HAL_TIMEOUT_MS);
    return Driver_MapHalStatus(hs);
}


/* ================================================================== */
/* Calibration block parsing                                           */
/* ================================================================== */

/**
 * @brief Parse 24 raw bytes from registers 0x88..0x9F into @ref s_calib.
 *
 * The block is little-endian; signed/unsigned widths follow datasheet
 * table 17.
 */
static void bmp280_parse_calibration(const uint8_t *raw)
{
    s_calib.dig_T1 = (uint16_t)(raw[0]  | ((uint16_t)raw[1]  << 8));
    s_calib.dig_T2 = (int16_t) (raw[2]  | ((uint16_t)raw[3]  << 8));
    s_calib.dig_T3 = (int16_t) (raw[4]  | ((uint16_t)raw[5]  << 8));
    s_calib.dig_P1 = (uint16_t)(raw[6]  | ((uint16_t)raw[7]  << 8));
    s_calib.dig_P2 = (int16_t) (raw[8]  | ((uint16_t)raw[9]  << 8));
    s_calib.dig_P3 = (int16_t) (raw[10] | ((uint16_t)raw[11] << 8));
    s_calib.dig_P4 = (int16_t) (raw[12] | ((uint16_t)raw[13] << 8));
    s_calib.dig_P5 = (int16_t) (raw[14] | ((uint16_t)raw[15] << 8));
    s_calib.dig_P6 = (int16_t) (raw[16] | ((uint16_t)raw[17] << 8));
    s_calib.dig_P7 = (int16_t) (raw[18] | ((uint16_t)raw[19] << 8));
    s_calib.dig_P8 = (int16_t) (raw[20] | ((uint16_t)raw[21] << 8));
    s_calib.dig_P9 = (int16_t) (raw[22] | ((uint16_t)raw[23] << 8));
}


/* ================================================================== */
/* Compensation formulas (BMP280 datasheet section 3.11.3)             */
/* ================================================================== */

/**
 * @brief Assemble a 20-bit unsigned ADC value from a 3-byte register read.
 *
 * Layout:  [MSB] [LSB] [XLSB], where the 20-bit value occupies the upper
 * 4 bits of XLSB. Always treated as unsigned per datasheet.
 */
static uint32_t bmp280_pack_adc20(const uint8_t raw[3])
{
    return ((uint32_t)raw[0] << 12) |
           ((uint32_t)raw[1] << 4)  |
           ((uint32_t)raw[2] >> 4);
}

/**
 * @brief Apply the integer temperature-compensation formula and update t_fine.
 *
 * @return Temperature in 0.01 ℃ (i.e. 5123 ⇒ 51.23 ℃).
 */
static int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) *
                    ((int32_t)s_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
                    ((int32_t)s_calib.dig_T3)) >> 14;
    s_t_fine = var1 + var2;
    s_t_fine_valid = true;
    return (s_t_fine * 5 + 128) >> 8;
}

/**
 * @brief Apply the int64 pressure-compensation formula.
 *
 * @return Pressure in Q24.8 fixed-point Pa (i.e. value / 256 = Pa).
 *         Returns 0 if the calibration leads to a divide-by-zero.
 */
static uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1 = ((int64_t)s_t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = ((((int64_t)1 << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;
    if (var1 == 0) {
        return 0u;
    }
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);
    return (uint32_t)p;
}


/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

Driver_Status BMP280_SetAddress(uint8_t addr7)
{
    /* Address selection is locked once Init succeeds (Requirement 4.9
     * + design constraint: prevents mid-flight bus address changes). */
    if (s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (addr7 != BMP280_ADDR7_DEFAULT && addr7 != BMP280_ADDR7_ALT) {
        return DRV_ERR_PARAM;
    }
    s_addr7 = addr7;
    return DRV_OK;
}

Driver_Status BMP280_Init(void)
{
    Driver_Status st;

    st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* 1. ChipID check (Requirement 4.1, 4.2). */
    uint8_t id = 0;
    st = bmp280_read_regs(BMP280_REG_ID, &id, 1u);
    if (st != DRV_OK) {
        goto fail;
    }
    if (id != BMP280_CHIP_ID) {
        st = DRV_ERR_ID;
        goto fail;
    }

    /* 2. Soft reset + wait ≥ 5 ms (Requirement 4.3). */
    st = bmp280_write_reg(BMP280_REG_RESET, BMP280_RESET_KEY);
    if (st != DRV_OK) {
        goto fail;
    }
    bmp280_delay_ms(BMP280_RESET_WAIT_MS);

    /* 3. Burst-read 24-byte calibration block (Requirement 4.4). */
    uint8_t calib_raw[24];
    st = bmp280_read_regs(BMP280_REG_CALIB_T1_LSB, calib_raw, sizeof(calib_raw));
    if (st != DRV_OK) {
        goto fail;
    }
    bmp280_parse_calibration(calib_raw);

    /* 4. Configure ctrl_meas + config (Requirement 4.5). */
    st = bmp280_write_reg(BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VALUE);
    if (st != DRV_OK) {
        goto fail;
    }
    st = bmp280_write_reg(BMP280_REG_CONFIG, BMP280_CONFIG_VALUE);
    if (st != DRV_OK) {
        goto fail;
    }

    /* All good — reset the t_fine cache so the first ReadPressure has to
     * re-derive it (or the caller may issue ReadTemperature first). */
    s_t_fine_valid = false;
    s_t_fine       = 0;
    s_initialized  = true;

    bmp280_unlock();
    return DRV_OK;

fail:
    bmp280_unlock();
    return st;
}

Driver_Status BMP280_GetChipID(uint8_t *id)
{
    if (id == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }
    st = bmp280_read_regs(BMP280_REG_ID, id, 1u);
    bmp280_unlock();
    return st;
}

/**
 * @brief Internal temperature read; assumes the bus mutex is already held.
 */
static Driver_Status bmp280_read_temperature_locked(float *temp_c)
{
    uint8_t raw[3];
    Driver_Status st = bmp280_read_regs(BMP280_REG_TEMP_MSB, raw, sizeof(raw));
    if (st != DRV_OK) {
        return st;
    }
    int32_t adc_T = (int32_t)bmp280_pack_adc20(raw);
    int32_t T_x100 = bmp280_compensate_T_int32(adc_T);
    *temp_c = (float)T_x100 / 100.0f;
    return DRV_OK;
}

Driver_Status BMP280_ReadTemperature(float *temp_c)
{
    if (temp_c == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }
    st = bmp280_read_temperature_locked(temp_c);
    bmp280_unlock();
    return st;
}

Driver_Status BMP280_ReadPressure(float *pressure_pa)
{
    if (pressure_pa == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = bmp280_lock();
    if (st != DRV_OK) {
        return st;
    }

    /* Pressure compensation depends on t_fine; if no temperature has been
     * read since init, run one internally so the result is meaningful. */
    if (!s_t_fine_valid) {
        float dummy_temp;
        st = bmp280_read_temperature_locked(&dummy_temp);
        if (st != DRV_OK) {
            bmp280_unlock();
            return st;
        }
    }

    uint8_t raw[3];
    st = bmp280_read_regs(BMP280_REG_PRES_MSB, raw, sizeof(raw));
    if (st != DRV_OK) {
        bmp280_unlock();
        return st;
    }

    int32_t adc_P = (int32_t)bmp280_pack_adc20(raw);
    uint32_t p_q248 = bmp280_compensate_P_int64(adc_P);
    *pressure_pa = (float)p_q248 / 256.0f;

    bmp280_unlock();
    return DRV_OK;
}
