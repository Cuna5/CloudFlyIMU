/**
 * @file    hardware.h
 * @brief   CloudFlyIMU 驱动层聚合头文件。
 *
 * 本文件是基础硬件驱动层（Driver_Layer）的唯一公共入口。
 * 应用层和 RTOS 任务应 `#include "hardware.h"`，不要直接包含各子模块头文件。
 *
 * 本头文件职责：
 *   - 定义统一驱动返回码 @ref Driver_Status。
 *   - 暴露全局可调参数宏：@ref HAL_TIMEOUT_MS、
 *     @ref HEATER_OVERHEAT_THRESHOLD_C、@ref DEBUG_LEVEL_MIN。
 *   - 定义共享数据结构 @ref SensorData_t、@ref AttitudeData_t
 *     以及调试等级枚举 @ref DEBUG_LEVEL_t。
 *   - 声明所有顶层驱动函数原型。
 *   - 声明共享数据互斥锁句柄 @ref DataMutexHandle。
 *   - 重新导出各子模块头文件，使单次 include 即可访问全部 API。
 */
#ifndef HARDWARE_H
#define HARDWARE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 标准库                                                              */
/* ------------------------------------------------------------------ */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* STM32 HAL（HAL_StatusTypeDef、外设句柄类型）                        */
/* ------------------------------------------------------------------ */
#include "stm32h7xx_hal.h"

/* ------------------------------------------------------------------ */
/* CMSIS-RTOS V2（osMutexId_t、osKernelGetState、osMutexAcquire 等）  */
/* ------------------------------------------------------------------ */
#include "cmsis_os2.h"


/* ================================================================== */
/* 1. 统一驱动返回码                                                   */
/* ================================================================== */

/**
 * @brief 所有公共驱动 API 的统一返回码。
 *
 * 驱动层每个初始化、读取、写入接口均返回此类型，
 * 上层代码可用单个 switch/if 链处理任意子模块的错误。
 */
typedef enum {
    DRV_OK              = 0,    /**< 成功。 */
    DRV_ERR_BUS         = 1,    /**< 总线错误（HAL_ERROR / HAL_BUSY）。 */
    DRV_ERR_ID          = 2,    /**< 初始化或自检时 Chip ID 不匹配。 */
    DRV_ERR_TIMEOUT     = 3,    /**< HAL 超时或互斥锁获取超时。 */
    DRV_ERR_PARAM       = 4,    /**< 空指针、非法地址或超温。 */
    DRV_ERR_NOT_INIT    = 5,    /**< 模块未初始化，或加热器故障锁存。 */
} Driver_Status;


/* ================================================================== */
/* 2. 全局可调参数                                                     */
/* ================================================================== */

/** HAL 总线 API 默认超时时间（毫秒），用于所有 HAL_SPI_*、HAL_I2C_*、
 *  HAL_UART_* 调用以及 osMutexAcquire。 */
#define HAL_TIMEOUT_MS                      100u

/** 加热器过温保护阈值（摄氏度）。 */
#ifndef HEATER_OVERHEAT_THRESHOLD_C
#define HEATER_OVERHEAT_THRESHOLD_C         55.0f
#endif


/* ================================================================== */
/* 3. 调试等级枚举 + 最低等级宏                                        */
/* ================================================================== */

/**
 * @brief Debug_Log_Level 接受的严重等级。
 *
 * 数值越小严重程度越高。运行时过滤器 DEBUG_LEVEL_MIN 会丢弃
 * 等级值大于（即不如）阈值的消息。
 */
typedef enum {
    DBG_ERR     = 0,    /**< 导致功能中断的错误。   "[ERR] "  */
    DBG_WARN    = 1,    /**< 可恢复的警告。          "[WARN] " */
    DBG_INFO    = 2,    /**< 默认信息级别。          "[INF] "  */
    DBG_DEBUG   = 3,    /**< 详细调试跟踪。          "[DBG] "  */
} DEBUG_LEVEL_t;

/** 运行时最低输出等级。lvl > DEBUG_LEVEL_MIN 的消息被 Debug_Log_Level 静默丢弃。 */
#ifndef DEBUG_LEVEL_MIN
#define DEBUG_LEVEL_MIN                     DBG_INFO
#endif


/* ================================================================== */
/* 4. 共享数据结构                                                     */
/* ================================================================== */

/**
 * @brief SensorTask 与消费者之间共享的最新原始传感器帧。
 *
 * 存储于 common/hardware.c 的 g_sensor_data；
 * 仅通过 SensorData_Set / SensorData_Get 在持有 DataMutexHandle 时读写
 * （FreeRTOS 调度器未启动时可直接访问）。
 */
typedef struct {
    uint32_t timestamp_ms;      /**< 采集时刻的 HAL_GetTick() 值。   */
    float    ax, ay, az;        /**< 加速度，m/s²（BMI088）。         */
    float    gx, gy, gz;        /**< 角速度，rad/s（BMI088）。        */
    float    mx, my, mz;        /**< 磁场，微特斯拉（IST8310）。      */
    float    temperature;       /**< 温度，摄氏度（BMP280）。         */
    float    pressure;          /**< 气压，Pa（BMP280）。             */
    float    pwm_duty;          /**< 最近一次应用的加热占空比，0~1。  */
} SensorData_t;

/**
 * @brief 最新姿态 / EKF 输出帧。
 *
 * 数据契约由驱动层定义；实际融合逻辑由上层实现。
 * 驱动层只保证结构体布局和互斥锁保护的访问器。
 */
typedef struct {
    uint32_t timestamp_ms;      /**< 更新时刻的 HAL_GetTick() 值。   */
    float    roll, pitch, yaw;  /**< 欧拉角，弧度。                   */
    float    bgx, bgy, bgz;     /**< 陀螺仪 bias 估计，rad/s。        */
    float    mag_reliability;   /**< 磁力计可信度，0.0 ~ 1.0。        */
    uint8_t  ekf_mode;          /**< 0=空闲，1=六轴，2=九轴。         */
} AttitudeData_t;


/* ================================================================== */
/* 5. 共享互斥锁句柄                                                   */
/* ================================================================== */

/**
 * @brief 保护 g_sensor_data 和 g_attitude_data 的互斥锁。
 *
 * 由 Hardware_Init 通过 osMutexNew 创建。存储于 common/hardware.c，
 * 此处 extern 声明供其他子模块（如 Heater_ApplyDuty）获取锁时使用。
 *
 * 加锁顺序：总线互斥锁（SPI/I2C）必须在本锁之前获取，不得反向，
 * 以避免死锁。
 */
extern osMutexId_t DataMutexHandle;

/**
 * @brief 幂等地通过 osMutexNew 创建 DataMutexHandle。
 *
 * 由 Hardware_Init 调用。多次调用安全：后续调用直接返回 DRV_OK。
 *
 * @return DRV_OK 成功（或已存在）；DRV_ERR_NOT_INIT osMutexNew 返回 NULL。
 */
Driver_Status DataMutex_Create(void);


/* ================================================================== */
/* 6. 顶层驱动函数原型                                                 */
/* ================================================================== */

/**
 * @brief 按固定顺序初始化整个驱动层。
 *
 * 顺序：Debug_UART → BMP280 → IST8310 → BMI088 → Heater → W25Q64。
 * 遇到第一个非 DRV_OK 的子模块立即返回（W25Q64 失败除外，仅记日志）。
 * 同时通过 osMutexNew 创建 DataMutexHandle。
 */
Driver_Status Hardware_Init(void);

/**
 * @brief 读取每个传感器的 Chip ID 并输出逐传感器 PASS/FAIL 日志。
 *
 * 即使部分检查失败也会遍历完整列表，以便一次运行得到完整诊断。
 * 所有 Chip ID 均匹配时返回 DRV_OK，否则返回 DRV_ERR_ID。
 * 可在 osKernelStart() 之前调用。
 *
 * Validates: Requirements 10.1, 10.2, 10.3, 10.4, 10.5.
 */
Driver_Status Hardware_SelfTest(void);

/**
 * @brief 将 HAL_StatusTypeDef 映射为驱动层返回码。
 *
 * 映射关系：
 *   - HAL_OK      → DRV_OK
 *   - HAL_TIMEOUT → DRV_ERR_TIMEOUT
 *   - HAL_ERROR   → DRV_ERR_BUS
 *   - HAL_BUSY    → DRV_ERR_BUS
 *   - 其他        → DRV_ERR_BUS
 */
Driver_Status Driver_MapHalStatus(HAL_StatusTypeDef hs);


/* ================================================================== */
/* 7. 共享数据访问器                                                   */
/* ================================================================== */

/** 在 DataMutexHandle 保护下将 *src 写入全局 SensorData。 */
Driver_Status SensorData_Set(const SensorData_t *src);

/** 在 DataMutexHandle 保护下将全局 SensorData 读出到 *dst。 */
Driver_Status SensorData_Get(SensorData_t *dst);

/** 在 DataMutexHandle 保护下将 *src 写入全局 AttitudeData。 */
Driver_Status AttitudeData_Set(const AttitudeData_t *src);

/** 在 DataMutexHandle 保护下将全局 AttitudeData 读出到 *dst。 */
Driver_Status AttitudeData_Get(AttitudeData_t *dst);


/* ================================================================== */
/* 8. 任务集成钩子                                                     */
/* ================================================================== */

/**
 * @brief 采集一帧原始传感器数据到 *out。
 *
 * 每次调用均读取 BMI088 加速度/角速度；IST8310 节流至约 50 Hz，
 * BMP280 节流至约 10 Hz（假设本函数以约 1 kHz 调用）。
 *
 * 单个传感器失败时保留上次成功值，累加错误计数，继续读取其余传感器。
 * 返回值为首个非 DRV_OK 状态（BMI → IST → BMP 顺序）。
 */
Driver_Status Sensor_SampleOnce(SensorData_t *out);

/**
 * @brief 快照各传感器累计错误计数。
 *
 * 任意输出指针可为 NULL 以跳过对应计数器。
 */
void Sensor_GetErrorCounters(uint32_t *bmi, uint32_t *ist, uint32_t *bmp);

/**
 * @brief 设置加热占空比并将实际值写回 SensorData。
 *
 * 内部调用 Heater_SetDuty(duty)；成功后在 DataMutexHandle 保护下
 * 更新 g_sensor_data.pwm_duty。
 */
Driver_Status Heater_ApplyDuty(float duty);


/* ================================================================== */
/* 9. 子模块头文件（单次 include 门面）                                */
/*                                                                      */
/* 放在最后，使各子模块头文件可以使用上方定义的共享类型和宏。          */
/* ================================================================== */
#include "debug_uart.h"
#include "bmp280.h"
#include "ist8310.h"
#include "bmi088.h"
#include "heater.h"
#include "pid.h"
#include "w25q64.h"
#include "mahony.h"
#include "ekf.h"


#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_H */
