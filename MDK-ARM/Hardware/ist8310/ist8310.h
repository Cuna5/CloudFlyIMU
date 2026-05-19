/**
 * @file    ist8310.h
 * @brief   IST8310 magnetometer driver (I2C1, 7-bit address 0x0E).
 *
 * Public API delivered by task 4.1. This header is intentionally part of
 * the aggregate facade (`common/hardware.h`): the aggregate header defines
 * @ref Driver_Status and then `#include`s this file, so we do not pull in
 * `hardware.h` here to avoid a circular include. Implementation lives in
 * `ist8310/ist8310.c`.
 *
 * Validates: Requirements 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 8.6.
 */
#ifndef IST8310_H
#define IST8310_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Driver_Status is provided transitively by `common/hardware.h`, which
 * includes this header last. Compiling ist8310.c directly should therefore
 * include `hardware.h` first to make Driver_Status visible. */

/**
 * @brief Initialise the IST8310 over I2C1.
 *
 * Steps (under @c I2CMutexHandle when the scheduler is running):
 *   1. Read WAI (`0x00`) and verify it equals `0x10` (Chip ID).
 *   2. Write CTRL2 (`0x0B`) ← `0xC0` to enable DRDY + interrupt.
 *   3. Write AVG_CTRL (`0x41`) ← `0x09` (X/Y/Z average configuration).
 *   4. Write PD_CTRL (`0x42`) ← `0xC0` (pulse duration / performance).
 *   5. Mark module as initialised.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_ID        Chip ID mismatch.
 * @retval DRV_ERR_BUS       HAL returned `HAL_ERROR` / `HAL_BUSY` / other.
 * @retval DRV_ERR_TIMEOUT   HAL timeout, or mutex acquire timeout.
 */
Driver_Status IST8310_Init(void);

/**
 * @brief Trigger one single-shot measurement and read X/Y/Z magnetic field.
 *
 * Sequence (under @c I2CMutexHandle when the scheduler is running):
 *   1. Write CTRL1 (`0x0A`) ← `0x01` to start a single conversion.
 *   2. Wait at least 6 ms (`osDelay(6)` if scheduler is running, else
 *      `HAL_Delay(6)`).
 *   3. Burst read 6 bytes from `0x03..0x08` (XL, XH, YL, YH, ZL, ZH).
 *   4. Reconstruct little-endian `int16_t` and scale by `0.3 µT/LSB`.
 *
 * @param mx Output X magnetic field, microtesla. Must not be NULL.
 * @param my Output Y magnetic field, microtesla. Must not be NULL.
 * @param mz Output Z magnetic field, microtesla. Must not be NULL.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_PARAM     any output pointer is NULL.
 * @retval DRV_ERR_NOT_INIT  module has not been initialised.
 * @retval DRV_ERR_BUS       HAL returned `HAL_ERROR` / `HAL_BUSY` / other.
 * @retval DRV_ERR_TIMEOUT   HAL timeout, or mutex acquire timeout.
 */
Driver_Status IST8310_ReadMag(float *mx, float *my, float *mz);

/**
 * @brief Read the IST8310 Chip ID register (`0x00`).
 *
 * @param id Output Chip ID byte. Must not be NULL.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_PARAM     `id` is NULL.
 * @retval DRV_ERR_NOT_INIT  module has not been initialised.
 * @retval DRV_ERR_BUS       HAL returned `HAL_ERROR` / `HAL_BUSY` / other.
 * @retval DRV_ERR_TIMEOUT   HAL timeout, or mutex acquire timeout.
 */
Driver_Status IST8310_GetChipID(uint8_t *id);

#ifdef __cplusplus
}
#endif

#endif /* IST8310_H */
