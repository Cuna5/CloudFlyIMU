/**
 * @file    csv_logger.c
 * @brief   CSV 数据记录模块实现：环形缓冲区 + FatFS 批量写入。
 */
#include "csv_logger.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* 内部状态                                                            */
/* ------------------------------------------------------------------ */

/** 环形缓冲区（静态分配，避免堆碎片）。 */
static CsvRecord_t s_ring[CSV_RING_CAPACITY];
static volatile uint32_t s_head;       /**< 写指针（Push 推进）。 */
static volatile uint32_t s_tail;       /**< 读指针（Flush 推进）。 */
static volatile uint32_t s_overflow;   /**< 溢出丢弃计数。 */

static osSemaphoreId_t s_sem;          /**< 保护 s_head/s_tail 的二值信号量。 */
static bool s_initialized;

static FIL  s_file;
static bool s_file_open;

/* ------------------------------------------------------------------ */
/* 内部辅助                                                            */
/* ------------------------------------------------------------------ */

/** 将 FatFS FRESULT 映射为驱动返回码（与 sd_storage.c 保持一致）。 */
static Driver_Status csv_map_fr(FRESULT fr)
{
    switch (fr) {
        case FR_OK:             return DRV_OK;
        case FR_TIMEOUT:        return DRV_ERR_TIMEOUT;
        case FR_NOT_READY:      return DRV_ERR_NOT_INIT;
        default:                return DRV_ERR_BUS;
    }
}

/** 确保 0:/log 目录存在，不存在则创建。 */
static Driver_Status csv_ensure_dir(void)
{
    FRESULT fr = f_mkdir(CSV_LOG_DIR);
    if (fr == FR_OK || fr == FR_EXIST) {
        return DRV_OK;
    }
    return csv_map_fr(fr);
}

/**
 * @brief 在 0:/log/ 下找到下一个可用的 log_NNNN.csv 文件名。
 *
 * 从 0001 开始递增，直到找到不存在的文件名（最多尝试 9999 次）。
 */
static Driver_Status csv_next_filename(char *buf, uint32_t buf_size)
{
    for (uint32_t i = 1U; i <= 9999U; i++) {
        (void)snprintf(buf, buf_size, CSV_LOG_DIR "/log_%04lu.csv", (unsigned long)i);
        FILINFO fno;
        FRESULT fr = f_stat(buf, &fno);
        if (fr == FR_NO_FILE) {
            return DRV_OK;
        }
    }
    return DRV_ERR_BUS;
}

/** 将一条 CsvRecord_t 格式化为 CSV 行，写入已打开的文件。 */
static Driver_Status csv_write_record(const CsvRecord_t *r)
{
    char line[200];
    int len = snprintf(line, sizeof(line),
        "%lu,%.2f,%.4f,"
        "%.5f,%.5f,%.5f,"
        "%.6f,%.6f,%.6f,"
        "%.3f,%.3f,%.3f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.3f,%.4f,"
        "%.5f,%.5f,%.5f,%u\r\n",
        (unsigned long)r->time_ms,
        (double)r->T, (double)r->pwm,
        (double)r->ax, (double)r->ay, (double)r->az,
        (double)r->gx, (double)r->gy, (double)r->gz,
        (double)r->mx, (double)r->my, (double)r->mz,
        (double)r->bgx, (double)r->bgy, (double)r->bgz,
        (double)r->gx_c, (double)r->gy_c, (double)r->gz_c,
        (double)r->mag_norm, (double)r->mag_score,
        (double)r->roll, (double)r->pitch, (double)r->yaw,
        (unsigned int)r->ekf_mode);

    if (len <= 0 || (uint32_t)len >= sizeof(line)) {
        return DRV_ERR_PARAM;
    }

    UINT bw = 0U;
    FRESULT fr = f_write(&s_file, line, (UINT)len, &bw);
    if (fr != FR_OK || bw != (UINT)len) {
        return DRV_ERR_BUS;
    }
    return DRV_OK;
}

/* ------------------------------------------------------------------ */
/* 公共 API                                                            */
/* ------------------------------------------------------------------ */

Driver_Status CsvLogger_Init(void)
{
    if (s_initialized) {
        return DRV_OK;
    }

    s_head = 0U;
    s_tail = 0U;
    s_overflow = 0U;
    s_file_open = false;

    static const osSemaphoreAttr_t sem_attr = { .name = "CsvLogSem" };
    s_sem = osSemaphoreNew(1U, 1U, &sem_attr);
    if (s_sem == NULL) {
        return DRV_ERR_NOT_INIT;
    }

    s_initialized = true;
    return DRV_OK;
}

Driver_Status CsvLogger_Open(void)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (!SDStorage_IsReady()) {
        return DRV_ERR_NOT_INIT;
    }
    if (s_file_open) {
        return DRV_OK;
    }

    Driver_Status st = csv_ensure_dir();
    if (st != DRV_OK) {
        return st;
    }

    char path[40];
    st = csv_next_filename(path, sizeof(path));
    if (st != DRV_OK) {
        return st;
    }

    FRESULT fr = f_open(&s_file, path, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK) {
        return csv_map_fr(fr);
    }

    /* 写入 CSV 表头 */
    static const char header[] =
        "time_ms,T,pwm,"
        "ax,ay,az,"
        "gx,gy,gz,"
        "mx,my,mz,"
        "bgx,bgy,bgz,"
        "gx_c,gy_c,gz_c,"
        "mag_norm,mag_score,"
        "roll,pitch,yaw,ekf_mode\r\n";

    UINT bw = 0U;
    fr = f_write(&s_file, header, sizeof(header) - 1U, &bw);
    if (fr != FR_OK || bw != sizeof(header) - 1U) {
        (void)f_close(&s_file);
        return DRV_ERR_BUS;
    }

    s_file_open = true;
    Debug_Log_Level(DBG_INFO, "CSV opened: %s", path);
    return DRV_OK;
}

Driver_Status CsvLogger_Close(void)
{
    if (!s_file_open) {
        return DRV_OK;
    }

    (void)f_sync(&s_file);
    FRESULT fr = f_close(&s_file);
    s_file_open = false;

    if (fr != FR_OK) {
        return DRV_ERR_BUS;
    }
    Debug_Log_Level(DBG_INFO, "CSV closed");
    return DRV_OK;
}

Driver_Status CsvLogger_Push(const CsvRecord_t *rec)
{
    if (rec == NULL) {
        return DRV_ERR_PARAM;
    }
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    if (osSemaphoreAcquire(s_sem, 0U) != osOK) {
        /* 信号量忙（Flush 正在读），直接丢弃本条记录 */
        s_overflow++;
        return DRV_OK;
    }

    uint32_t next_head = (s_head + 1U) % CSV_RING_CAPACITY;
    if (next_head == s_tail) {
        /* 缓冲区满，覆盖最旧记录（s_tail 前进） */
        s_tail = (s_tail + 1U) % CSV_RING_CAPACITY;
        s_overflow++;
    }
    s_ring[s_head] = *rec;
    s_head = next_head;

    (void)osSemaphoreRelease(s_sem);
    return DRV_OK;
}

Driver_Status CsvLogger_Flush(void)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    /* 若文件未打开，先尝试打开 */
    if (!s_file_open) {
        Driver_Status st = CsvLogger_Open();
        if (st != DRV_OK) {
            return st;
        }
    }

    uint32_t written = 0U;
    Driver_Status result = DRV_OK;

    while (written < CSV_FLUSH_BATCH) {
        /* 取信号量，检查是否有数据 */
        if (osSemaphoreAcquire(s_sem, HAL_TIMEOUT_MS) != osOK) {
            result = DRV_ERR_TIMEOUT;
            break;
        }

        if (s_tail == s_head) {
            /* 缓冲区空 */
            (void)osSemaphoreRelease(s_sem);
            break;
        }

        CsvRecord_t rec = s_ring[s_tail];
        s_tail = (s_tail + 1U) % CSV_RING_CAPACITY;
        (void)osSemaphoreRelease(s_sem);

        Driver_Status st = csv_write_record(&rec);
        if (st != DRV_OK) {
            result = st;
            /* 写入失败，关闭文件，下次 Flush 重新打开 */
            (void)CsvLogger_Close();
            break;
        }
        written++;
    }

    /* 每批写完后 sync，减少数据丢失风险 */
    if (written > 0U && s_file_open) {
        (void)f_sync(&s_file);
    }

    return result;
}

uint32_t CsvLogger_Pending(void)
{
    uint32_t h = s_head;
    uint32_t t = s_tail;
    return (h >= t) ? (h - t) : (CSV_RING_CAPACITY - t + h);
}

uint32_t CsvLogger_OverflowCount(void)
{
    return s_overflow;
}
