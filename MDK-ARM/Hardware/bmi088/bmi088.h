/**
 * @file    bmi088.h
 * @brief   BMI088 六轴 IMU 驱动（SPI1，双片选 PC4/PC5）公共 API。
 *
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 * 实现位于 bmi088/bmi088.c。
 *
 * 总线布局：
 *   - SPI 外设          = SPI1（hspi1）
 *   - 加速度计片选      = PC4（BMI_ACC_CS_Pin）
 *   - 陀螺仪片选        = PC5（BMI_GYRO_CS_Pin）
 */
#ifndef BMI088_H
#define BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Driver_Status 由 common/hardware.h 传递定义，
 * bmi088.c 中通过先包含 hardware.h 使该类型可见。 */

/**
 * @brief 通过 SPI1 初始化 BMI088（加速度计 + 陀螺仪）。
 *
 * 步骤（调度器运行时在 SPIMutexHandle 保护下执行）：
 *   1. 将 hspi1.Init.DataSize 修正为 SPI_DATASIZE_8BIT 并重新调用
 *      HAL_SPI_Init，修复 CubeMX 生成的 SPI_DATASIZE_4BIT 问题。
 *   2. 发送加速度计软复位（0x7E ← 0xB6），等待至少 50 ms。
 *   3. 发送一次虚拟 Chip ID 读取，强制加速度计进入 SPI 模式（数据手册 §6.1.2）。
 *   4. 读取加速度计 Chip ID（0x00），验证等于 0x1E。
 *   5. 读取陀螺仪 Chip ID（0x00），验证等于 0x0F。
 *   6. 配置加速度计：0x7D←0x04（使能），0x7C←0x00（Active），
 *      0x41←0x01（量程 ±6g），0x40←0xAB（ODR 800Hz/Normal BW）。
 *   7. 配置陀螺仪：0x0F←0x00（量程 ±2000°/s），0x10←0x02（ODR 1000Hz/BW 116Hz）。
 *   8. 标记模块已初始化。
 *
 * @return DRV_OK 成功；DRV_ERR_ID 加速度计或陀螺仪 Chip ID 不匹配；
 *         DRV_ERR_BUS HAL 返回 HAL_ERROR/HAL_BUSY；
 *         DRV_ERR_TIMEOUT HAL 或互斥锁超时。
 */
Driver_Status BMI088_Init(void);

/**
 * @brief 突发读取最新加速度计数据并转换为 m/s²。
 *
 * 从加速度计 0x12..0x17 读取 6 字节（XL, XH, YL, YH, ZL, ZH），
 * 按 ±6g 灵敏度 5460 LSB/g 转换：axis = raw / 5460.0f * 9.80665f。
 *
 * @param ax  输出 X 轴加速度，m/s²，不得为 NULL。
 * @param ay  输出 Y 轴加速度，m/s²，不得为 NULL。
 * @param az  输出 Z 轴加速度，m/s²，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM 任意输出指针为 NULL；
 *         DRV_ERR_NOT_INIT 模块未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status BMI088_ReadAccel(float *ax, float *ay, float *az);

/**
 * @brief 突发读取最新陀螺仪数据并转换为 rad/s。
 *
 * 从陀螺仪 0x02..0x07 读取 6 字节（XL, XH, YL, YH, ZL, ZH），
 * 按 ±2000°/s 灵敏度 16.384 LSB/(°/s) 转换：axis = raw / 16.384f * (π/180)。
 *
 * @param gx  输出 X 轴角速度，rad/s，不得为 NULL。
 * @param gy  输出 Y 轴角速度，rad/s，不得为 NULL。
 * @param gz  输出 Z 轴角速度，rad/s，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM 任意输出指针为 NULL；
 *         DRV_ERR_NOT_INIT 模块未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status BMI088_ReadGyro(float *gx, float *gy, float *gz);

/**
 * @brief 读取 BMI088 加速度计和陀螺仪的 Chip ID。
 *
 * @param acc_id   输出加速度计 Chip ID（期望值 0x1E），不得为 NULL。
 * @param gyro_id  输出陀螺仪 Chip ID（期望值 0x0F），不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM 任意输出指针为 NULL；
 *         DRV_ERR_NOT_INIT 模块未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status BMI088_GetChipID(uint8_t *acc_id, uint8_t *gyro_id);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
