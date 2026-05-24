/**
 * @file    w25q64.h
 * @brief   W25Q64 QSPI NOR Flash raw driver.
 *
 * Application parameter storage has moved to SD/FatFs (`sd_storage`).
 */
#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

Driver_Status W25Q64_Init(void);
Driver_Status W25Q64_ReadID(uint32_t *id);
Driver_Status W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len);
Driver_Status W25Q64_EraseSector(uint32_t addr);
Driver_Status W25Q64_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
