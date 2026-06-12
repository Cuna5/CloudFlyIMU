/**
 * @file    csv_logger.h
 * @brief   CSV 数据记录模块：环形缓冲区 + FatFS 批量写入。
 *
 * 使用方式：
 *   1. 高频任务（SensorTask / FusionTask）调用 CsvLogger_Push() 写入环形缓冲区；
 *   2. 低优先级 SdLogTask 调用 CsvLogger_Flush() 批量写入 SD 卡；
 *   3. 通过 CsvLogger_Open / CsvLogger_Close 管理文件生命周期。
 *
 * 线程安全：Push 和 Flush 分别由不同任务调用，内部用 FreeRTOS 信号量保护。
 */
#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "hardware.h"

/* ------------------------------------------------------------------ */
/* 可调参数                                                            */
/* ------------------------------------------------------------------ */

/** 环形缓冲区容量（条记录数）。每条记录约 160 字节，总占用约 16 KB。 */
#ifndef CSV_RING_CAPACITY
#define CSV_RING_CAPACITY   100U
#endif

/** 每次 Flush 最多写入的记录条数（控制单次 SD 写入时长）。 */
#ifndef CSV_FLUSH_BATCH
#define CSV_FLUSH_BATCH     50U
#endif

/** 日志文件目录（不含末尾斜杠）。 */
#define CSV_LOG_DIR         "0:/log"

/* ------------------------------------------------------------------ */
/* 数据结构                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 一条完整的 CSV 日志记录，包含传感器原始数据和算法中间量。
 *
 * 字段顺序与 CSV 表头一致：
 * time_ms, T, pwm, ax, ay, az, gx, gy, gz, mx, my, mz,
 * bgx, bgy, bgz, gx_c, gy_c, gz_c, mag_norm, mag_score,
 * roll, pitch, yaw, ekf_mode
 */
typedef struct {
    uint32_t time_ms;

    float T;
    float pwm;

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    float bgx, bgy, bgz;       /**< AI 预测零偏（暂时填 EKF 估计值）。 */
    float gx_c, gy_c, gz_c;    /**< 补偿后角速度（暂时填原始值）。 */

    float mag_norm;
    float mag_score;

    float roll, pitch, yaw;
    uint8_t ekf_mode;
} CsvRecord_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief 初始化 CSV 记录模块（创建内部信号量，清空缓冲区）。
 *
 * 必须在 FreeRTOS 调度器启动后调用。
 * @return DRV_OK 成功；DRV_ERR_NOT_INIT 信号量创建失败。
 */
Driver_Status CsvLogger_Init(void);

/**
 * @brief 打开一个新的 CSV 日志文件并写入表头。
 *
 * 文件名格式：0:/log/log_NNNN.csv，NNNN 从 0001 自增。
 * 若目录不存在则自动创建。
 * @return DRV_OK 成功；DRV_ERR_NOT_INIT SD 未就绪；DRV_ERR_BUS 文件系统错误。
 */
Driver_Status CsvLogger_Open(void);

/**
 * @brief 关闭当前 CSV 文件（flush 并 f_close）。
 *
 * @return DRV_OK 成功或文件本未打开；DRV_ERR_BUS 关闭失败。
 */
Driver_Status CsvLogger_Close(void);

/**
 * @brief 将一条记录压入环形缓冲区（非阻塞）。
 *
 * 可在高频任务中调用。缓冲区满时丢弃最旧记录并累计溢出计数。
 * @param rec  待写入的记录，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM rec 为 NULL；DRV_ERR_NOT_INIT 未初始化。
 */
Driver_Status CsvLogger_Push(const CsvRecord_t *rec);

/**
 * @brief 将环形缓冲区中的记录批量写入 SD 卡（最多 CSV_FLUSH_BATCH 条）。
 *
 * 应在低优先级 SdLogTask 中周期调用。文件未打开时自动尝试打开。
 * @return DRV_OK 成功（含无数据可写）；DRV_ERR_BUS 写入失败。
 */
Driver_Status CsvLogger_Flush(void);

/**
 * @brief 查询环形缓冲区当前积压的记录条数。
 */
uint32_t CsvLogger_Pending(void);

/**
 * @brief 查询累计溢出丢弃的记录条数。
 */
uint32_t CsvLogger_OverflowCount(void);

#ifdef __cplusplus
}
#endif

#endif /* CSV_LOGGER_H */
