/**
 * @file    w25q64.h
 * @brief   W25Q64 QSPI NOR Flash driver (8 MB, 4096 sectors × 4096 B).
 *
 * 提供三层接口：
 *   1. 原始 Flash 操作：Init / ReadID / Read / EraseSector / PageProgram
 *   2. 参数存储：Params_Load / Params_Save（带 CRC32 校验）
 *
 * 参数布局（Flash 地址 0x000000 起）：
 *   [0x000000] ParamPage_t（256 B）：CRC32(4B) + 版本(1B) + 保留(3B) + 参数体
 *
 * 所有接口返回 Driver_Status，与其余驱动保持一致。
 */
#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* 参数结构体 — 按需扩展字段，不要改变已有字段的偏移                  */
/* ------------------------------------------------------------------ */
typedef struct {
    /* PID 恒温控制 */
    float heat_kp;
    float heat_ki;
    float heat_kd;
    float heat_target_c;

    /* 预留扩展（EKF 初始协方差等） */
    uint8_t reserved[108];
} AppParams_t;                  /* 总计 128 B，留在 256 B 页内 */

/* ------------------------------------------------------------------ */
/* 原始 Flash 操作                                                     */
/* ------------------------------------------------------------------ */

/** 初始化并校验 JEDEC ID（0xEF4017）。 */
Driver_Status W25Q64_Init(void);

/** 读取 JEDEC ID（3 字节）。 */
Driver_Status W25Q64_ReadID(uint32_t *id);

/**
 * @brief 从 Flash 读取任意长度数据。
 * @param addr   24-bit 字节地址
 * @param buf    目标缓冲区
 * @param len    字节数
 */
Driver_Status W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief 擦除一个 4 KB 扇区（地址自动对齐到扇区边界）。
 * @param addr   扇区内任意地址
 */
Driver_Status W25Q64_EraseSector(uint32_t addr);

/**
 * @brief 写入最多 256 B（一页）。地址必须页对齐，跨页行为未定义。
 * @param addr   页对齐地址
 * @param buf    数据
 * @param len    ≤ 256
 */
Driver_Status W25Q64_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len);

/* ------------------------------------------------------------------ */
/* 参数存储（带 CRC32 校验）                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief 从 Flash 加载参数到 @p params。
 *
 * 若 CRC 校验失败，@p params 保持不变，返回 DRV_ERR_ID。
 * 调用方应在失败时使用编译期默认值。
 */
Driver_Status Params_Load(AppParams_t *params);

/**
 * @brief 将 @p params 写入 Flash（先擦扇区再写页）。
 *
 * 写入后立即回读校验 CRC，失败返回 DRV_ERR_BUS。
 */
Driver_Status Params_Save(const AppParams_t *params);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
