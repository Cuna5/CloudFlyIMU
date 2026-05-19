/**
 * @file    ist8310.h
 * @brief   IST8310 磁力计驱动（I2C1，7-bit 地址 0x0E）公共 API。
 *
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 * 实现位于 ist8310/ist8310.c。
 */
#ifndef IST8310_H
#define IST8310_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Driver_Status 由 common/hardware.h 传递定义，
 * ist8310.c 中通过先包含 hardware.h 使该类型可见。 */

/**
 * @brief 通过 I2C1 初始化 IST8310。
 *
 * 步骤（调度器运行时在 I2CMutexHandle 保护下执行）：
 *   1. 读 WAI（0x00），验证等于 0x10（Chip ID）。
 *   2. 写 CTRL2（0x0B）← 0xC0，使能 DRDY + 中断。
 *   3. 写 AVG_CTRL（0x41）← 0x09（X/Y/Z 平均配置）。
 *   4. 写 PD_CTRL（0x42）← 0xC0（脉冲宽度/性能配置）。
 *   5. 标记模块已初始化。
 *
 * @return DRV_OK 成功；DRV_ERR_ID Chip ID 不匹配；
 *         DRV_ERR_BUS HAL 返回 HAL_ERROR/HAL_BUSY；
 *         DRV_ERR_TIMEOUT HAL 或互斥锁超时。
 */
Driver_Status IST8310_Init(void);

/**
 * @brief 触发一次单次测量并读取 X/Y/Z 磁场数据。
 *
 * 步骤（调度器运行时在 I2CMutexHandle 保护下执行）：
 *   1. 写 CTRL1（0x0A）← 0x01，启动单次转换。
 *   2. 等待至少 6 ms（调度器运行时用 osDelay，否则用 HAL_Delay）。
 *   3. 从 0x03..0x08 突发读取 6 字节（XL, XH, YL, YH, ZL, ZH）。
 *   4. 按小端 int16_t 重建三轴原始值，乘以 0.3 µT/LSB 转换为物理量。
 *
 * @param mx  输出 X 轴磁场，微特斯拉，不得为 NULL。
 * @param my  输出 Y 轴磁场，微特斯拉，不得为 NULL。
 * @param mz  输出 Z 轴磁场，微特斯拉，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM 任意输出指针为 NULL；
 *         DRV_ERR_NOT_INIT 模块未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status IST8310_ReadMag(float *mx, float *my, float *mz);

/**
 * @brief 读取 IST8310 Chip ID 寄存器（0x00）。
 *
 * @param id  输出 Chip ID 字节，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM id 为 NULL；
 *         DRV_ERR_NOT_INIT 模块未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status IST8310_GetChipID(uint8_t *id);

#ifdef __cplusplus
}
#endif

#endif /* IST8310_H */
