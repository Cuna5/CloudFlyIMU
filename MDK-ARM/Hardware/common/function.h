/**
 * @file    function.h
 * @brief   FreeRTOS 任务入口函数声明。
 *
 * 每个函数对应一个 FreeRTOS 任务，由 freertos.c 中的任务包装函数调用。
 * 实现位于 common/function.c。
 */
#ifndef FUNCTION_H
#define FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hardware.h"

/** @brief 传感器采样任务（5 ms 周期）：读取 BMI088/IST8310/BMP280，写入共享数据。 */
void Task_Sensor(void);

/** @brief 加热控制任务：运行 PID 控制器，调用 Heater_ApplyDuty。 */
void Task_Heat(void);

/** @brief Mahony 互补滤波姿态解算任务（5 ms 周期）。 */
void Task_MahonyFusion(void);

/** @brief EKF 姿态融合任务（5 ms 周期）。 */
void Task_EKFFusion(void);

/** @brief 调试/显示任务（50 ms 周期）：刷新 OLED，输出 CPU 占用统计。 */
void Task_Debug(void);

/** @brief SD 卡日志任务（20 ms 周期）：从环形缓冲区批量写入 CSV 文件。 */
void Task_SdLog(void);

#ifdef __cplusplus
}
#endif

#endif /* FUNCTION_H */
