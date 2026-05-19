/**
 * @file    bmp280.h
 * @brief   BMP280 temperature/pressure driver (I2C1).
 *
 * Implementation lives in `bmp280.c` (task 3.1).
 *
 * Public API contract (Driver_Status return type, error codes, mutex
 * handling, scheduler-aware delays) follows the conventions defined in
 * @ref hardware.h. This header MUST be reached via `#include "hardware.h"`,
 * never included directly by application code.
 *
 * Key constants:
 *   - 7-bit I2C address: 0x76 (default) or 0x77 (selectable via
 *     @ref BMP280_SetAddress before @ref BMP280_Init).
 *   - Chip ID register 0xD0 must read back 0x58.
 *
 * Validates: Requirements 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 4.10, 8.6.
 */
#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* This header is included from `common/hardware.h` AFTER the shared
 * Driver_Status enum and project-wide tunables (HAL_TIMEOUT_MS, etc.)
 * have been defined, so we can refer to `Driver_Status` directly here.
 *
 * Application code MUST include "hardware.h" rather than this file.
 */

/**
 * @brief Select the I2C 7-bit address used by the BMP280 module.
 *
 * Must be called BEFORE @ref BMP280_Init. Only `0x76` and `0x77` are
 * legal values.
 *
 * @retval DRV_OK            Address accepted.
 * @retval DRV_ERR_PARAM     `addr7` is neither 0x76 nor 0x77.
 * @retval DRV_ERR_NOT_INIT  @ref BMP280_Init has already completed; the
 *                           address is locked from this point on.
 */
Driver_Status BMP280_SetAddress(uint8_t addr7);

/**
 * @brief Initialise the BMP280 over I2C1.
 *
 * Sequence:
 *   1. Read register 0xD0; verify ChipID == 0x58.
 *   2. Soft reset (write 0xE0 = 0xB6) and wait at least 5 ms.
 *   3. Burst read 24 bytes from 0x88 to populate dig_T1..T3, dig_P1..P9.
 *   4. Write 0xF4 = 0x57 (temp ×2, pres ×16, Normal mode).
 *   5. Write 0xF5 = 0x90 (standby 62.5 ms, IIR coefficient 16).
 *
 * @retval DRV_OK          Driver ready.
 * @retval DRV_ERR_ID      ChipID mismatch.
 * @retval DRV_ERR_BUS     HAL returned HAL_ERROR/HAL_BUSY.
 * @retval DRV_ERR_TIMEOUT HAL or mutex acquire timed out.
 */
Driver_Status BMP280_Init(void);

/**
 * @brief Read a compensated temperature sample.
 *
 * Reads 3 raw bytes from 0xFA..0xFC and applies the datasheet
 * compensation formula. Updates the module-internal `t_fine` value
 * required by @ref BMP280_ReadPressure.
 *
 * @param[out] temp_c  Temperature in degrees Celsius (≥ 0.01 ℃ resolution).
 *
 * @retval DRV_OK            Success; `*temp_c` updated.
 * @retval DRV_ERR_PARAM     `temp_c` is NULL.
 * @retval DRV_ERR_NOT_INIT  Driver has not been initialised.
 * @retval DRV_ERR_BUS / DRV_ERR_TIMEOUT  Bus failure (per HAL mapping).
 */
Driver_Status BMP280_ReadTemperature(float *temp_c);

/**
 * @brief Read a compensated pressure sample.
 *
 * If a temperature has not been read since init, an internal temperature
 * read is triggered first to obtain a valid `t_fine`. Then 3 raw bytes
 * are read from 0xF7..0xF9 and the int64 datasheet formula is applied.
 *
 * @param[out] pressure_pa  Pressure in Pascals.
 *
 * @retval DRV_OK            Success; `*pressure_pa` updated.
 * @retval DRV_ERR_PARAM     `pressure_pa` is NULL.
 * @retval DRV_ERR_NOT_INIT  Driver has not been initialised.
 * @retval DRV_ERR_BUS / DRV_ERR_TIMEOUT  Bus failure (per HAL mapping).
 */
Driver_Status BMP280_ReadPressure(float *pressure_pa);

/**
 * @brief Read the BMP280 ChipID register (0xD0).
 *
 * Useful for `Hardware_SelfTest` and debugging. Driver must be
 * initialised first.
 *
 * @param[out] id  Receives the raw ChipID byte.
 *
 * @retval DRV_OK            Success; `*id` updated.
 * @retval DRV_ERR_PARAM     `id` is NULL.
 * @retval DRV_ERR_NOT_INIT  Driver has not been initialised.
 * @retval DRV_ERR_BUS / DRV_ERR_TIMEOUT  Bus failure.
 */
Driver_Status BMP280_GetChipID(uint8_t *id);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
