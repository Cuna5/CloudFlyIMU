/**
 * @file    bmi088.h
 * @brief   BMI088 6-axis IMU driver (SPI1, dual CS PC4/PC5).
 *
 * Public API delivered by task 5.1. Like every other driver header in
 * `MDK-ARM/Hardware/`, this file is part of the aggregate facade
 * `common/hardware.h`: the aggregate header defines @ref Driver_Status and
 * then `#include`s this file, so we deliberately do NOT include
 * `hardware.h` here to avoid a circular include. Implementations live in
 * `bmi088/bmi088.c`.
 *
 * Bus layout:
 *   - SPI peripheral           = SPI1 (`hspi1`).
 *   - Accelerometer CS         = PC4 (`BMI_ACC_CS_Pin`).
 *   - Gyroscope CS             = PC5 (`BMI_GYRO_CS_Pin`).
 *
 * Spec: .kiro/specs/hardware-base-drivers/{requirements,design,tasks}.md
 *
 * Validates: Requirements 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9,
 *            2.10, 2.11, 2.12, 8.6.
 */
#ifndef BMI088_H
#define BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Driver_Status is provided transitively by `common/hardware.h`, which
 * includes this header last. Compiling bmi088.c directly therefore
 * includes `hardware.h` first to make Driver_Status visible. */

/**
 * @brief Initialise the BMI088 (accelerometer + gyroscope) over SPI1.
 *
 * Steps (under @c SPIMutexHandle when the scheduler is running):
 *   1. Fix `hspi1.Init.DataSize` to `SPI_DATASIZE_8BIT` and re-call
 *      `HAL_SPI_Init(&hspi1)` — patches the CubeMX `SPI_DATASIZE_4BIT`
 *      anomaly in `CloudFlyIMU.ioc`.
 *   2. Issue an accelerometer soft-reset (`0x7E ← 0xB6`) and wait at
 *      least 50 ms.
 *   3. Force the accelerometer SPI bus into SPI mode by issuing a dummy
 *      Chip-ID read (datasheet §6.1.2).
 *   4. Read accelerometer Chip ID (`0x00`) and verify it equals `0x1E`.
 *   5. Read gyroscope Chip ID (`0x00`) and verify it equals `0x0F`.
 *   6. Configure the accelerometer:
 *        - `0x41 ← 0x01` (range = ±6 g)
 *        - `0x40 ← 0xAB` (ODR = 800 Hz, BW = OSR4)
 *        - `0x7D ← 0x04` (power-control = Normal)
 *   7. Configure the gyroscope:
 *        - `0x0F ← 0x01` (range = ±2000 °/s)
 *        - `0x10 ← 0x02` (ODR 1000 Hz / BW 116 Hz)
 *   8. Mark the module as initialised.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_ID        Chip ID mismatch on the accelerometer or
 *                           the gyroscope.
 * @retval DRV_ERR_BUS       HAL returned `HAL_ERROR` / `HAL_BUSY` / other.
 * @retval DRV_ERR_TIMEOUT   HAL timeout, or mutex acquire timeout.
 */
Driver_Status BMI088_Init(void);

/**
 * @brief Burst-read the latest accelerometer sample and convert to m/s².
 *
 * Reads 6 bytes from accelerometer `0x12..0x17` (XL, XH, YL, YH, ZL, ZH)
 * and converts each axis with the ±6 g sensitivity 5460 LSB/g:
 *   `axis = raw / 5460.0f * 9.80665f`.
 *
 * @param ax Output X acceleration, m/s². Must not be NULL.
 * @param ay Output Y acceleration, m/s². Must not be NULL.
 * @param az Output Z acceleration, m/s². Must not be NULL.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_PARAM     any output pointer is NULL.
 * @retval DRV_ERR_NOT_INIT  module has not been initialised.
 * @retval DRV_ERR_BUS       HAL returned `HAL_ERROR` / `HAL_BUSY` / other.
 * @retval DRV_ERR_TIMEOUT   HAL timeout, or mutex acquire timeout.
 */
Driver_Status BMI088_ReadAccel(float *ax, float *ay, float *az);

/**
 * @brief Burst-read the latest gyroscope sample and convert to rad/s.
 *
 * Reads 6 bytes from gyroscope `0x02..0x07` (XL, XH, YL, YH, ZL, ZH)
 * and converts each axis with the ±2000 °/s sensitivity 16.384 LSB/(°/s):
 *   `axis = raw / 16.384f * (M_PI / 180.0f)`.
 *
 * @param gx Output X angular rate, rad/s. Must not be NULL.
 * @param gy Output Y angular rate, rad/s. Must not be NULL.
 * @param gz Output Z angular rate, rad/s. Must not be NULL.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_PARAM     any output pointer is NULL.
 * @retval DRV_ERR_NOT_INIT  module has not been initialised.
 * @retval DRV_ERR_BUS       HAL returned `HAL_ERROR` / `HAL_BUSY` / other.
 * @retval DRV_ERR_TIMEOUT   HAL timeout, or mutex acquire timeout.
 */
Driver_Status BMI088_ReadGyro(float *gx, float *gy, float *gz);

/**
 * @brief Read the Chip ID of both BMI088 dies (accelerometer + gyroscope).
 *
 * @param acc_id  Output accelerometer Chip ID byte (expected `0x1E`).
 *                Must not be NULL.
 * @param gyro_id Output gyroscope Chip ID byte (expected `0x0F`).
 *                Must not be NULL.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_PARAM     any output pointer is NULL.
 * @retval DRV_ERR_NOT_INIT  module has not been initialised.
 * @retval DRV_ERR_BUS       HAL returned `HAL_ERROR` / `HAL_BUSY` / other.
 * @retval DRV_ERR_TIMEOUT   HAL timeout, or mutex acquire timeout.
 */
Driver_Status BMI088_GetChipID(uint8_t *acc_id, uint8_t *gyro_id);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
