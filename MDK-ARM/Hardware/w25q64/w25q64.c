#include "hardware.h"
#include "quadspi.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* W25Q64 指令集                                                       */
/* ------------------------------------------------------------------ */
#define CMD_READ_ID         0x9FU
#define CMD_READ_DATA       0x6BU   /* Quad Output Fast Read */
#define CMD_WRITE_ENABLE    0x06U
#define CMD_SECTOR_ERASE    0x20U
#define CMD_PAGE_PROGRAM    0x32U   /* Quad Input Page Program */
#define CMD_READ_SR1        0x05U

#define W25Q64_JEDEC_ID     0xEF4017U
#define W25Q64_PAGE_SIZE    256U
#define W25Q64_SECTOR_SIZE  4096U
#define PARAMS_FLASH_ADDR   0x000000U

static bool s_initialized = false;

/* ------------------------------------------------------------------ */
/* 内部辅助                                                            */
/* ------------------------------------------------------------------ */

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
        if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) return DRV_ERR_BUS;
        if (HAL_QSPI_Receive(&hqspi, &sr, HAL_TIMEOUT_MS)  != HAL_OK) return DRV_ERR_BUS;
        if (HAL_GetTick() > deadline) return DRV_ERR_TIMEOUT;
    } while (sr & 0x01U);
    return DRV_OK;
}

static Driver_Status qspi_write_enable(void)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_WRITE_ENABLE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;
    return Driver_MapHalStatus(HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS));
}

/* CRC32 IEEE 802.3，无查表 */
static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ------------------------------------------------------------------ */
/* 参数页布局（256 B）                                                 */
/* ------------------------------------------------------------------ */
#define PARAM_VERSION  1U

typedef struct {
    uint32_t    crc32;
    uint8_t     version;
    uint8_t     _pad[3];
    AppParams_t params;
} ParamPage_t;

/* ================================================================== */
/* 公共 API                                                            */
/* ================================================================== */

Driver_Status W25Q64_Init(void)
{
    uint32_t id = 0;
    Driver_Status st = W25Q64_ReadID(&id);
    if (st != DRV_OK) return st;
    if (id != W25Q64_JEDEC_ID) return DRV_ERR_ID;
    s_initialized = true;
    return DRV_OK;
}

Driver_Status W25Q64_ReadID(uint32_t *id)
{
    if (!id) return DRV_ERR_PARAM;
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_READ_ID;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 3;
    cmd.DummyCycles     = 0;

    uint8_t buf[3] = {0};
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) return DRV_ERR_BUS;
    if (HAL_QSPI_Receive(&hqspi, buf, HAL_TIMEOUT_MS)  != HAL_OK) return DRV_ERR_BUS;
    *id = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return DRV_OK;
}

Driver_Status W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!s_initialized) return DRV_ERR_NOT_INIT;
    if (!buf || len == 0) return DRV_ERR_PARAM;

    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_READ_DATA;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address         = addr;
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.NbData          = len;
    cmd.DummyCycles     = 8;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) return DRV_ERR_BUS;
    return Driver_MapHalStatus(HAL_QSPI_Receive(&hqspi, buf, HAL_TIMEOUT_MS));
}

Driver_Status W25Q64_EraseSector(uint32_t addr)
{
    if (!s_initialized) return DRV_ERR_NOT_INIT;
    Driver_Status st = qspi_write_enable();
    if (st != DRV_OK) return st;

    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_SECTOR_ERASE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address         = addr & ~(W25Q64_SECTOR_SIZE - 1U);
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.DataMode        = QSPI_DATA_NONE;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) return DRV_ERR_BUS;
    return qspi_wait_busy();
}

Driver_Status W25Q64_PageProgram(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!s_initialized) return DRV_ERR_NOT_INIT;
    if (!buf || len == 0 || len > W25Q64_PAGE_SIZE) return DRV_ERR_PARAM;

    Driver_Status st = qspi_write_enable();
    if (st != DRV_OK) return st;

    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction     = CMD_PAGE_PROGRAM;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address         = addr;
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.NbData          = len;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_TIMEOUT_MS) != HAL_OK) return DRV_ERR_BUS;
    if (HAL_QSPI_Transmit(&hqspi, (uint8_t *)buf, HAL_TIMEOUT_MS) != HAL_OK) return DRV_ERR_BUS;
    return qspi_wait_busy();
}

/* ------------------------------------------------------------------ */
/* 参数存储                                                            */
/* ------------------------------------------------------------------ */

Driver_Status Params_Load(AppParams_t *params)
{
    if (!s_initialized) return DRV_ERR_NOT_INIT;
    if (!params) return DRV_ERR_PARAM;

    ParamPage_t page;
    Driver_Status st = W25Q64_Read(PARAMS_FLASH_ADDR, (uint8_t *)&page, sizeof(page));
    if (st != DRV_OK) return st;

    uint32_t expected = crc32_calc((const uint8_t *)&page.params, sizeof(AppParams_t));
    if (page.crc32 != expected) return DRV_ERR_ID;

    memcpy(params, &page.params, sizeof(AppParams_t));
    return DRV_OK;
}

Driver_Status Params_Save(const AppParams_t *params)
{
    if (!s_initialized) return DRV_ERR_NOT_INIT;
    if (!params) return DRV_ERR_PARAM;

    ParamPage_t page;
    memset(&page, 0xFF, sizeof(page));
    page.version = PARAM_VERSION;
    memcpy(&page.params, params, sizeof(AppParams_t));
    page.crc32 = crc32_calc((const uint8_t *)&page.params, sizeof(AppParams_t));

    Driver_Status st = W25Q64_EraseSector(PARAMS_FLASH_ADDR);
    if (st != DRV_OK) return st;

    st = W25Q64_PageProgram(PARAMS_FLASH_ADDR, (const uint8_t *)&page, sizeof(page));
    if (st != DRV_OK) return st;

    /* 回读校验 */
    ParamPage_t verify;
    st = W25Q64_Read(PARAMS_FLASH_ADDR, (uint8_t *)&verify, sizeof(verify));
    if (st != DRV_OK) return st;
    if (verify.crc32 != page.crc32) return DRV_ERR_BUS;

    return DRV_OK;
}
