/**
 * @file    heater.h
 * @brief   加热 PWM 驱动（TIM3_CH1，PC6）公共 API。
 *
 * 加热器通过 PC6 驱动 YJL3400A N-MOSFET，控制 5V/10Ω/5W 加热电阻。
 * 输出为约 1 Hz 的慢速 PWM，由 TIM3_CH1 产生。
 *
 * 状态模型：
 *   - 未初始化  → 所有非初始化 API 返回 DRV_ERR_NOT_INIT。
 *   - 正常      → Heater_SetDuty 驱动比较器。
 *   - 故障锁存  → Heater_SetDuty 拒绝操作并返回 DRV_ERR_NOT_INIT，
 *                 直到 Heater_ClearFault 清除锁存。
 *
 * 本头文件通过 common/hardware.h 聚合导出，依赖 Driver_Status、bool 和
 * HEATER_OVERHEAT_THRESHOLD_C 已在作用域内。
 */
#ifndef HEATER_H
#define HEATER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 TIM3_CH1 PWM 输出，设置 CCR1=0，启动 PWM。
 */
Driver_Status Heater_Init(void);

/**
 * @brief 设置 PWM 占空比（0.0 ~ 1.0）。
 *
 * 负值夹紧为 0，大于 1 的值夹紧为 1，NaN 返回 DRV_ERR_PARAM。
 * 模块未初始化或故障锁存时返回 DRV_ERR_NOT_INIT，不写硬件寄存器。
 */
Driver_Status Heater_SetDuty(float duty);

/**
 * @brief 读取最近一次成功应用的占空比。
 *
 * @return DRV_ERR_PARAM duty 为 NULL；DRV_ERR_NOT_INIT 未初始化。
 */
Driver_Status Heater_GetDuty(float *duty);

/**
 * @brief 立即将 CCR1 置 0 并锁存故障标志。
 *
 * 调用后 Heater_SetDuty 拒绝驱动加热器，直到 Heater_ClearFault 被调用。
 */
Driver_Status Heater_EmergencyStop(void);

/** @brief 清除故障锁存。硬件不会自动恢复，调用方需再次调用 Heater_SetDuty。 */
Driver_Status Heater_ClearFault(void);

/**
 * @brief 当测量温度超过 HEATER_OVERHEAT_THRESHOLD_C 时触发急停。
 *
 * @return DRV_OK 低于阈值；DRV_ERR_PARAM 达到或超过阈值（内部已调用急停）。
 */
Driver_Status Heater_OverheatCheck(float temperature_c);

/** @brief 只读查询故障锁存状态。模块未初始化时返回 false。 */
bool          Heater_IsFaultLatched(void);

#ifdef __cplusplus
}
#endif

#endif /* HEATER_H */
