/**
 * @file    sd_storage.h
 * @brief   基于 SD/FatFs 的应用参数持久化存储驱动公共 API。
 *
 * 参数文件路径：0:/CFIMUCFG.BIN，包含 magic、版本号、CRC32 校验和应用参数。
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 */
#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** 应用参数结构体，总大小固定为 128 字节（含保留字段）。 */
typedef struct {
    float heat_kp;          /**< 加热 PID 比例增益。 */
    float heat_ki;          /**< 加热 PID 积分增益。 */
    float heat_kd;          /**< 加热 PID 微分增益。 */
    float heat_target_c;    /**< 加热目标温度，摄氏度。 */

    uint8_t reserved[108];  /**< 保留字段，供后续扩展使用。 */
} AppParams_t;

/**
 * @brief 初始化 SD 存储驱动，检查 FatFs 注册状态并尝试挂载文件系统。
 *
 * 调度器未运行时仅标记初始化完成，挂载延迟到首次读写时执行。
 * @return DRV_OK 成功；DRV_ERR_NOT_INIT SD 卡未注册或挂载失败。
 */
Driver_Status SDStorage_Init(void);

/** @brief 查询 SD 卡是否已初始化且文件系统已挂载。 */
bool SDStorage_IsReady(void);

/**
 * @brief 从 SD 卡加载应用参数。
 *
 * 读取参数文件，验证 magic、版本号和 CRC32，通过后将参数复制到 params。
 * @param params  输出参数结构体，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM params 为 NULL 或文件校验失败；
 *         DRV_ERR_NOT_INIT SD 未就绪；DRV_ERR_BUS 文件系统错误。
 */
Driver_Status Params_Load(AppParams_t *params);

/**
 * @brief 将应用参数保存到 SD 卡，写入后回读验证。
 *
 * @param params  待保存的参数结构体，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM params 为 NULL；
 *         DRV_ERR_NOT_INIT SD 未就绪；DRV_ERR_BUS 写入或验证失败。
 */
Driver_Status Params_Save(const AppParams_t *params);

#ifdef __cplusplus
}
#endif

#endif /* SD_STORAGE_H */
