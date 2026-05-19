/**
 * @file    bmp280.h
 * @brief   BMP280 温度/气压传感器驱动（I2C1）公共 API。
 *
 * 实现位于 bmp280.c。
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 *
 * 关键常量：
 *   - 7-bit I2C 地址：0x76（默认）或 0x77（在 BMP280_Init 之前通过
 *     BMP280_SetAddress 选择）。
 *   - Chip ID 寄存器 0xD0 应读回 0x58。
 */
#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 本头文件由 common/hardware.h 在定义 Driver_Status 等共享类型之后包含，
 * 因此可直接使用 Driver_Status。应用层必须通过 "hardware.h" 包含本文件。 */

/**
 * @brief 选择 BMP280 使用的 I2C 7-bit 地址。
 *
 * 必须在 BMP280_Init 之前调用。仅接受 0x76 和 0x77。
 *
 * @return DRV_OK 地址已接受；DRV_ERR_PARAM addr7 不是 0x76 或 0x77；
 *         DRV_ERR_NOT_INIT BMP280_Init 已完成，地址已锁定。
 */
Driver_Status BMP280_SetAddress(uint8_t addr7);

/**
 * @brief 通过 I2C1 初始化 BMP280。
 *
 * 步骤：
 *   1. 读寄存器 0xD0，验证 ChipID == 0x58。
 *   2. 软复位（写 0xE0 = 0xB6），等待至少 5 ms。
 *   3. 从 0x88 突发读取 24 字节，解析 dig_T1..T3、dig_P1..P9。
 *   4. 写 0xF4 = 0x57（温度 ×2，气压 ×16，Normal 模式）。
 *   5. 写 0xF5 = 0x90（待机 62.5 ms，IIR 系数 16）。
 *
 * @return DRV_OK 成功；DRV_ERR_ID ChipID 不匹配；
 *         DRV_ERR_BUS HAL 返回 HAL_ERROR/HAL_BUSY；
 *         DRV_ERR_TIMEOUT HAL 或互斥锁超时。
 */
Driver_Status BMP280_Init(void);

/**
 * @brief 读取补偿后的温度值。
 *
 * 从 0xFA..0xFC 读取 3 字节原始数据，按数据手册补偿公式计算。
 * 同时更新内部 t_fine 值（BMP280_ReadPressure 需要）。
 *
 * @param temp_c  输出温度，摄氏度（分辨率 ≥ 0.01 ℃）。
 * @return DRV_OK 成功；DRV_ERR_PARAM temp_c 为 NULL；
 *         DRV_ERR_NOT_INIT 驱动未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status BMP280_ReadTemperature(float *temp_c);

/**
 * @brief 读取补偿后的气压值。
 *
 * 若初始化后尚未读取过温度，内部先触发一次温度读取以获得有效 t_fine，
 * 再从 0xF7..0xF9 读取 3 字节，按数据手册 int64 补偿公式计算。
 *
 * @param pressure_pa  输出气压，Pa。
 * @return DRV_OK 成功；DRV_ERR_PARAM pressure_pa 为 NULL；
 *         DRV_ERR_NOT_INIT 驱动未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status BMP280_ReadPressure(float *pressure_pa);

/**
 * @brief 读取 BMP280 ChipID 寄存器（0xD0）。
 *
 * 用于 Hardware_SelfTest 和调试，驱动必须已初始化。
 *
 * @param id  输出 ChipID 字节。
 * @return DRV_OK 成功；DRV_ERR_PARAM id 为 NULL；
 *         DRV_ERR_NOT_INIT 驱动未初始化；
 *         DRV_ERR_BUS / DRV_ERR_TIMEOUT 总线错误。
 */
Driver_Status BMP280_GetChipID(uint8_t *id);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
