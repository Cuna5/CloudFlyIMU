/**
 * @file    w25q64.c
 * @brief   W25Q64 QSPI NOR Flash 底层驱动实现。
 *
 * 使用 STM32 QSPI 外设（hqspi）操作 W25Q64：
 *   - 读 ID：单线指令 0x9F，接收 3 字节
 *   - 快速读：单线指令 0x6B，四线数据，8 哑周期
 *   - 扇区擦除：单线指令 0x20，4 KB 对齐
 *   - 页编程：单线指令 0x32，四线数据，最大 256 字节
 * 写操作前均自动发送写使能（0x06），完成后轮询状态寄存器忙标志。
 */
#include "hardware.h"
#include "quadspi.h"

/* QSPI 指令码 */
#define CMD_READ_ID         0x9FU   /* 读 JEDEC ID */
#define CMD_READ_DATA       0x6BU   /* Quad 输出快速读 */
#define CMD_WRITE_ENABLE    0x06U   /* 写使能 */
#define CMD_SECTOR_ERASE    0x20U   /* 4 KB 扇区擦除 */
#define CMD_PAGE_PROGRAM    0x32U   /* Quad 输入页编程 */
#define CMD_READ_SR1        0x05U   /* 读状态寄存器 1 */

#define W25Q64_JEDEC_ID     0xEF4017U   /* 期望的 JEDEC ID */
#define W25Q64_PAGE_SIZE    256U        /* 页大小，字节 */
#define W25Q64_SECTOR_SIZE  4096U       /* 扇区大小，字节 */

static bool s_initialized = false;

/** @brief 轮询状态寄存器 BUSY 位，直到操作完成或超时（3 秒）。 */
static Driver_Status qspi_wait_busy(void)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_READ_SR1;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 1;
    cmd.DummyCycles     = 0;

    uint8_t sr;
    uint32_t deadline = HAL_GetTick() + 3000U;
    do {
        if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) {
            return DRV_ERR_BUS;
        }
        if (HAL_QSPI_Receive(&hqspi, &sr, HAL_TIMEOUT_MS) != HAL_OK) {
            return DRV_ERR_BUS;
        }
        if (HAL_GetTick() > deadline) {
            return DRV_ERR_TIMEOUT;
        }
    } while ((sr & 0x01U) != 0U);

    return DRV_OK;
}

/** @brief 发送写使能命令（0x06），擦除/编程前必须调用。 */
static Driver_Status qspi_write_enable(void)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_WRITE_ENABLE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;

    return Driver_MapHalStatus(HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS));
}

Driver_Status W25Q64_Init(void)
{
    uint32_t id = 0U;
    Driver_Status st = W25Q64_ReadID(&id);
    if (st != DRV_OK) {
        return st;
    }
    if (id != W25Q64_JEDEC_ID) {
        return DRV_ERR_ID;
    }

    s_initialized = true;
    return DRV_OK;
}

Driver_Status W25Q64_ReadID(uint32_t *id)
{
    if (id == NULL) {
        return DRV_ERR_PARAM;
    }

    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_READ_ID;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 3;
    cmd.DummyCycles     = 0;

    uint8_t buf[3] = {0};
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) {
        return DRV_ERR_BUS;
    }
    if (HAL_QSPI_Receive(&hqspi, buf, HAL_TIMEOUT_MS) != HAL_OK) {
        return DRV_ERR_BUS;
    }

    *id = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return DRV_OK;
}

Driver_Status W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (buf == NULL || len == 0U) {
        return DRV_ERR_PARAM;
    }

    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_READ_DATA;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address         = addr;
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.NbData          = len;
    cmd.DummyCycles     = 8;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) {
        return DRV_ERR_BUS;
    }

    return Driver_MapHalStatus(HAL_QSPI_Receive(&hqspi, buf, HAL_TIMEOUT_MS));
}

Driver_Status W25Q64_EraseSector(uint32_t addr)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = qspi_write_enable();
    if (st != DRV_OK) {
        return st;
    }

    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_SECTOR_ERASE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address         = addr & ~(W25Q64_SECTOR_SIZE - 1U);
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.DataMode        = QSPI_DATA_NONE;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) {
        return DRV_ERR_BUS;
    }

    return qspi_wait_busy();
}

Driver_Status W25Q64_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (buf == NULL || len == 0U || len > W25Q64_PAGE_SIZE) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = qspi_write_enable();
    if (st != DRV_OK) {
        return st;
    }

    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_PAGE_PROGRAM;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address         = addr;
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.NbData          = len;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) {
        return DRV_ERR_BUS;
    }
    if (HAL_QSPI_Transmit(&hqspi, (uint8_t *)buf, HAL_TIMEOUT_MS) != HAL_OK) {
        return DRV_ERR_BUS;
    }

    return qspi_wait_busy();
}
