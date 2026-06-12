/**
 * @file    w25q64.h
 * @brief   W25Q64 QSPI NOR Flash 底层驱动公共 API。
 *
 * 通过 QSPI 接口操作 W25Q64（8 MB，JEDEC ID 0xEF4017）。
 * 应用参数持久化已迁移至 SD/FatFs（sd_storage），本驱动仅提供原始读写接口。
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 */
#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化 W25Q64，读取并验证 JEDEC ID（期望值 0xEF4017）。
 * @return DRV_OK 成功；DRV_ERR_ID JEDEC ID 不匹配；DRV_ERR_BUS 总线错误。
 */
Driver_Status W25Q64_Init(void);

/**
 * @brief 读取 JEDEC ID（3 字节：制造商 + 存储类型 + 容量）。
 * @param id  输出 24-bit ID，不得为 NULL。
 * @return DRV_OK 成功；DRV_ERR_PARAM id 为 NULL；DRV_ERR_BUS 总线错误。
 */
Driver_Status W25Q64_ReadID(uint32_t *id);

/**
 * @brief 从指定地址读取任意长度数据（Quad 输出快速读，指令 0x6B）。
 * @param addr  起始地址（24-bit）。
 * @param buf   输出缓冲区，不得为 NULL。
 * @param len   读取字节数，不得为 0。
 * @return DRV_OK 成功；DRV_ERR_NOT_INIT 未初始化；
 *         DRV_ERR_PARAM 参数非法；DRV_ERR_BUS 总线错误。
 */
Driver_Status W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief 擦除包含 addr 的 4 KB 扇区（地址自动对齐到扇区边界）。
 *
 * 擦除前自动发送写使能命令，擦除后轮询忙标志直到完成（最长 3 秒）。
 * @return DRV_OK 成功；DRV_ERR_NOT_INIT 未初始化；
 *         DRV_ERR_BUS 总线错误；DRV_ERR_TIMEOUT 等待超时。
 */
Driver_Status W25Q64_EraseSector(uint32_t addr);

/**
 * @brief 向指定地址写入最多 256 字节（页编程，Quad 输入，指令 0x32）。
 *
 * 写入前自动发送写使能命令，写入后轮询忙标志直到完成。
 * 调用方须确保目标区域已擦除，且写入范围不跨页边界。
 * @param addr  起始地址（24-bit）。
 * @param buf   数据缓冲区，不得为 NULL。
 * @param len   写入字节数，范围 [1, 256]。
 * @return DRV_OK 成功；DRV_ERR_NOT_INIT 未初始化；
 *         DRV_ERR_PARAM 参数非法；DRV_ERR_BUS 总线错误；DRV_ERR_TIMEOUT 超时。
 */
Driver_Status W25Q64_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
