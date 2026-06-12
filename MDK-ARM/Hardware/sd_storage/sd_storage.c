/**
 * @file    sd_storage.c
 * @brief   基于 SD/FatFs 的应用参数持久化存储驱动实现。
 *
 * 参数文件格式（SdParamFile_t）：
 *   [crc32(4)] [magic(4)] [version(1)] [reserved(3)] [AppParams_t(128)]
 * 写入后回读验证，确保数据完整性。
 */
#include "hardware.h"
#include "fatfs.h"

#include <string.h>

/* 参数文件标识与版本 */
#define SD_PARAM_MAGIC      0x4346494DU   /* "CFIM" */
#define SD_PARAM_VERSION    1U
#define SD_PARAM_FILE       "0:/CFIMUCFG.BIN"

/** 参数文件在磁盘上的完整布局（含校验头）。 */
typedef struct {
    uint32_t crc32;
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    AppParams_t params;
} SdParamFile_t;

static bool s_initialized;
static bool s_mounted;

/** @brief 计算 data[0..len-1] 的 CRC32（IEEE 802.3 多项式）。 */
static uint32_t sd_crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    while (len-- != 0U) {
        crc ^= *data++;
        for (uint32_t i = 0U; i < 8U; i++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/** @brief 将 FatFs FRESULT 映射为统一驱动返回码。 */
static Driver_Status sd_map_fresult(FRESULT fr)
{
    switch (fr) {
        case FR_OK:
            return DRV_OK;
        case FR_NO_FILE:
        case FR_NO_PATH:
        case FR_INVALID_NAME:
            return DRV_ERR_ID;
        case FR_TIMEOUT:
            return DRV_ERR_TIMEOUT;
        case FR_INVALID_OBJECT:
        case FR_INVALID_PARAMETER:
            return DRV_ERR_PARAM;
        case FR_NOT_READY:
            return DRV_ERR_NOT_INIT;
        default:
            return DRV_ERR_BUS;
    }
}

/** @brief 立即挂载 SD 文件系统（仅在未挂载时执行）。 */
static Driver_Status sd_mount_now(void)
{
    if (s_mounted) {
        return DRV_OK;
    }
    if (retSD != 0U || SDPath[0] == '\0') {
        return DRV_ERR_NOT_INIT;
    }

    FRESULT fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1);
    if (fr != FR_OK) {
        s_mounted = false;
        return sd_map_fresult(fr);
    }

    s_mounted = true;
    return DRV_OK;
}

/** @brief 确保 SD 已初始化并挂载，调度器未运行时跳过挂载。 */
static Driver_Status sd_ensure_mounted(void)
{
    Driver_Status st;

    if (!s_initialized) {
        st = SDStorage_Init();
        if (st != DRV_OK) {
            return st;
        }
    }

    if (s_mounted) {
        return DRV_OK;
    }

    if (osKernelGetState() == osKernelInactive) {
        return DRV_ERR_NOT_INIT;
    }

    return sd_mount_now();
}

/** @brief 验证参数文件的 magic、版本号和 CRC32。 */
static bool sd_param_file_is_valid(const SdParamFile_t *file)
{
    if (file == NULL) {
        return false;
    }
    if (file->magic != SD_PARAM_MAGIC || file->version != SD_PARAM_VERSION) {
        return false;
    }

    uint32_t crc = sd_crc32_calc((const uint8_t *)&file->params, sizeof(file->params));
    return file->crc32 == crc;
}

/** @brief 从 SD 卡读取并校验参数文件到 file 缓冲区。 */
static Driver_Status sd_read_param_file(SdParamFile_t *file)
{
    if (file == NULL) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = sd_ensure_mounted();
    if (st != DRV_OK) {
        return st;
    }

    FIL fp;
    FRESULT fr = f_open(&fp, SD_PARAM_FILE, FA_READ);
    if (fr != FR_OK) {
        return sd_map_fresult(fr);
    }

    UINT bytes_read = 0U;
    memset(file, 0, sizeof(*file));
    fr = f_read(&fp, file, sizeof(*file), &bytes_read);
    FRESULT close_fr = f_close(&fp);

    if (fr != FR_OK) {
        return sd_map_fresult(fr);
    }
    if (close_fr != FR_OK) {
        return sd_map_fresult(close_fr);
    }
    if (bytes_read != sizeof(*file)) {
        return DRV_ERR_ID;
    }
    if (!sd_param_file_is_valid(file)) {
        return DRV_ERR_ID;
    }

    return DRV_OK;
}

Driver_Status SDStorage_Init(void)
{
    if (retSD != 0U || SDPath[0] == '\0') {
        s_initialized = false;
        s_mounted = false;
        return DRV_ERR_NOT_INIT;
    }

    s_initialized = true;

    if (osKernelGetState() == osKernelInactive) {
        return DRV_OK;
    }

    return sd_mount_now();
}

bool SDStorage_IsReady(void)
{
    return s_initialized && s_mounted;
}

Driver_Status Params_Load(AppParams_t *params)
{
    if (params == NULL) {
        return DRV_ERR_PARAM;
    }

    SdParamFile_t file;
    Driver_Status st = sd_read_param_file(&file);
    if (st != DRV_OK) {
        return st;
    }

    memcpy(params, &file.params, sizeof(*params));
    return DRV_OK;
}

Driver_Status Params_Save(const AppParams_t *params)
{
    if (params == NULL) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = sd_ensure_mounted();
    if (st != DRV_OK) {
        return st;
    }

    SdParamFile_t file;
    memset(&file, 0, sizeof(file));
    file.magic = SD_PARAM_MAGIC;
    file.version = SD_PARAM_VERSION;
    memcpy(&file.params, params, sizeof(file.params));
    file.crc32 = sd_crc32_calc((const uint8_t *)&file.params, sizeof(file.params));

    FIL fp;
    FRESULT fr = f_open(&fp, SD_PARAM_FILE, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        return sd_map_fresult(fr);
    }

    UINT bytes_written = 0U;
    fr = f_write(&fp, &file, sizeof(file), &bytes_written);
    if (fr == FR_OK) {
        fr = f_sync(&fp);
    }
    FRESULT close_fr = f_close(&fp);

    if (fr != FR_OK) {
        return sd_map_fresult(fr);
    }
    if (close_fr != FR_OK) {
        return sd_map_fresult(close_fr);
    }
    if (bytes_written != sizeof(file)) {
        return DRV_ERR_BUS;
    }

    SdParamFile_t verify;
    st = sd_read_param_file(&verify);
    if (st != DRV_OK) {
        return st;
    }
    if (memcmp(&verify.params, params, sizeof(*params)) != 0) {
        return DRV_ERR_BUS;
    }

    return DRV_OK;
}
