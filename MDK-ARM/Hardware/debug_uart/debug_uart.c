/**
 * @file    debug_uart.c
 * @brief   调试串口驱动（USART1）实现。
 *
 * - DebugUART_Init 验证 CubeMX 生成的 huart1 句柄指向 USART1，
 *   创建内部串行化互斥锁（不重新初始化外设）。
 * - Debug_Log / Debug_Log_Level 通过 vsnprintf 格式化到 256 字节静态缓冲区，
 *   添加等级前缀后调用 HAL_UART_Transmit 发送。超长内容截断，末尾保留 '\0'。
 * - 运行时过滤器 DEBUG_LEVEL_MIN 丢弃比阈值更冗长的消息。
 * - FreeRTOS 调度器运行时通过内部互斥锁串行化，调度器未启动时跳过加锁。
 * - printf 通过 fputc（ARMCC/IAR）和弱符号 _write（GCC/armclang+newlib）
 *   重定向到本驱动。
 * - 任何发送失败（未初始化、互斥锁超时、HAL 错误、等级过滤）均静默丢弃，
 *   不递归调用 Debug_Log_Level，累加计数器供 Debug_GetDroppedCount 查询。
 */
#include "hardware.h"   /* Driver_Status, DEBUG_LEVEL_t, DEBUG_LEVEL_MIN,
                         * HAL_TIMEOUT_MS，以及通过门面包含的 debug_uart.h */
#include "usart.h"      /* extern UART_HandleTypeDef huart1                */

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/* ================================================================== */
/* 模块私有常量                                                        */
/* ================================================================== */

/** 格式化/发送缓冲区大小（字节）。 */
#define DEBUG_UART_BUF_SIZE         256u


/* ================================================================== */
/* 模块私有状态                                                        */
/* ================================================================== */

/** 静态格式化缓冲区，255 字节有效载荷 + 终止符，无需动态分配。 */
static char            s_buf[DEBUG_UART_BUF_SIZE];

/** 内部串行化互斥锁；在 DebugUART_Init 中创建。 */
static osMutexId_t     s_dbg_mutex;

/** 互斥锁名称属性，存于静态存储以保证字符串生命周期。 */
static const osMutexAttr_t s_dbg_mutex_attr = {
    .name      = "DebugUART",
    .attr_bits = 0u,
    .cb_mem    = NULL,
    .cb_size   = 0u,
};

/** DebugUART_Init 成功后置 true，所有公共 API 和 C 库重定向钩子均检查此标志。 */
static bool            s_initialized;

/** 静默丢弃的日志行计数器，在 UINT32_MAX 处饱和。 */
static uint32_t        s_dropped;


/* ================================================================== */
/* 内部辅助函数                                                        */
/* ================================================================== */

static inline bool dbg_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief 调度器运行时获取串行化互斥锁。
 *
 * 成功（已加锁或调度器未运行）返回 true；超时/错误返回 false，调用方应丢弃该行。
 */
static bool dbg_lock(void)
{
    if (!dbg_scheduler_running()) {
        return true;
    }
    if (s_dbg_mutex == NULL) {
        /* 防御性检查：Init 成功后不应发生，但避免无限等待 */
        return false;
    }
    return osMutexAcquire(s_dbg_mutex, HAL_TIMEOUT_MS) == osOK;
}

static void dbg_unlock(void)
{
    if (dbg_scheduler_running() && s_dbg_mutex != NULL) {
        (void)osMutexRelease(s_dbg_mutex);
    }
}

/** s_dropped 饱和累加（上限 UINT32_MAX）。 */
static void dbg_count_drop(void)
{
    if (s_dropped != UINT32_MAX) {
        s_dropped++;
    }
}

/** 返回等级前缀字符串；未知等级返回 "" 以防缓冲区计算崩溃。 */
static const char *dbg_level_prefix(DEBUG_LEVEL_t lvl)
{
    switch (lvl) {
        case DBG_ERR:   return "[ERR] ";
        case DBG_WARN:  return "[WARN] ";
        case DBG_INFO:  return "[INF] ";
        case DBG_DEBUG: return "[DBG] ";
        default:        return "";
    }
}

/**
 * @brief 通过 USART1 发送 len 字节。
 *
 * 前提：调用方已持有串行化互斥锁（或调度器未运行），且模块已初始化。
 * HAL 失败时累加丢弃计数器，不递归调用 Debug_Log_Level。
 */
static void dbg_uart_tx(const uint8_t *data, uint16_t len)
{
    if (len == 0u) {
        return;
    }
    HAL_StatusTypeDef hs = HAL_UART_Transmit(&huart1, data, len, HAL_TIMEOUT_MS);
    if (hs != HAL_OK) {
        dbg_count_drop();
    }
}

/**
 * @brief 将等级前缀 + vsnprintf 格式化内容写入 s_buf 并发送。
 *
 * @param lvl  严重等级，同时作为过滤输入——大于 DEBUG_LEVEL_MIN 时提前返回。
 * @param fmt  printf 格式字符串，不得为 NULL。
 * @param ap   与 fmt 配对的可变参数列表。
 */
static void dbg_emit(DEBUG_LEVEL_t lvl, const char *fmt, va_list ap)
{
    if (!s_initialized || fmt == NULL) {
        dbg_count_drop();
        return;
    }
    /* 等级过滤：比阈值更冗长的消息直接丢弃 */
    if ((int)lvl > (int)DEBUG_LEVEL_MIN) {
        return;
    }

    if (!dbg_lock()) {
        dbg_count_drop();
        return;
    }

    /* 阶段 1：将等级前缀复制到缓冲区头部（前缀不含 '%'，用 memcpy 更高效） */
    const char *prefix     = dbg_level_prefix(lvl);
    size_t      prefix_len = strlen(prefix);
    if (prefix_len >= DEBUG_UART_BUF_SIZE) {
        /* 当前前缀最长 7 字节，此处为防御性检查 */
        prefix_len = DEBUG_UART_BUF_SIZE - 1u;
    }
    memcpy(s_buf, prefix, prefix_len);

    /* 阶段 2：将用户内容格式化到剩余空间。
     * vsnprintf 写入最多 n-1 个字符后追加 '\0'，返回值为不截断时应写入的字符数。 */
    size_t remaining = DEBUG_UART_BUF_SIZE - prefix_len;
    int    written  = vsnprintf(s_buf + prefix_len, remaining, fmt, ap);

    size_t body_len;
    if (written < 0) {
        /* 编码错误——只发送前缀，正文计入丢弃 */
        body_len = 0u;
        dbg_count_drop();
    } else if ((size_t)written >= remaining) {
        /* 截断：vsnprintf 已在 s_buf[DEBUG_UART_BUF_SIZE-1] 写入 '\0' */
        body_len = remaining - 1u;
    } else {
        body_len = (size_t)written;
    }

    size_t total = prefix_len + body_len;
    /* 防御性检查：不超过缓冲区大小 */
    if (total >= DEBUG_UART_BUF_SIZE) {
        total = DEBUG_UART_BUF_SIZE - 1u;
    }
    /* 保留末尾 '\0'（主要供调试器查看；UART 只发送 total 字节） */
    s_buf[total] = '\0';

    dbg_uart_tx((const uint8_t *)s_buf, (uint16_t)total);
    dbg_unlock();
}


/* ================================================================== */
/* 公共 API                                                            */
/* ================================================================== */

Driver_Status DebugUART_Init(void)
{
    /* 不重新初始化外设，只验证 CubeMX 生成的句柄指向 USART1 */
    if (huart1.Instance != USART1) {
        return DRV_ERR_PARAM;
    }

    /* 创建模块内部互斥锁（已存在则复用，保证 Init 幂等） */
    if (s_dbg_mutex == NULL) {
        s_dbg_mutex = osMutexNew(&s_dbg_mutex_attr);
        if (s_dbg_mutex == NULL) {
            s_initialized = false;
            return DRV_ERR_NOT_INIT;
        }
    }

    s_dropped     = 0u;
    s_initialized = true;
    return DRV_OK;
}

void Debug_Log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    dbg_emit(DBG_INFO, fmt, ap);
    va_end(ap);
}

void Debug_Log_Level(DEBUG_LEVEL_t lvl, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    dbg_emit(lvl, fmt, ap);
    va_end(ap);
}

uint32_t Debug_GetDroppedCount(void)
{
    return s_dropped;
}


/* ================================================================== */
/* printf 重定向                                                       */
/*                                                                      */
/* C 库 printf 系列通过与 Debug_Log_Level 相同的丢弃计数和调度器感知   */
/* 加锁机制发送。提供两种实现：                                         */
/*   - ARMCC v5/v6 / IAR / armclang armlibc：实现 fputc。              */
/*   - GCC / armclang + newlib：实现弱符号 _write。                    */
/* 两者均通过 dbg_putc_locked 集中处理互斥锁和 HAL_UART_Transmit。    */
/* ================================================================== */

/**
 * @brief 通过 USART1 发送单字节，与 Debug_Log_Level 使用相同的加锁和丢弃策略。
 *
 * 供两种 stdio 重定向共用，保证多任务下不乱序，丢失字节被计数，
 * DebugUART_Init 之前调用安全。成功返回 true，否则返回 false。
 */
static bool dbg_putc_locked(uint8_t ch)
{
    if (!s_initialized) {
        dbg_count_drop();
        return false;
    }
    if (!dbg_lock()) {
        dbg_count_drop();
        return false;
    }
    HAL_StatusTypeDef hs = HAL_UART_Transmit(&huart1, &ch, 1u, HAL_TIMEOUT_MS);
    dbg_unlock();
    if (hs != HAL_OK) {
        dbg_count_drop();
        return false;
    }
    return true;
}

#if defined(__ARMCC_VERSION) || defined(__CC_ARM) || defined(__ICCARM__)
/* Keil ArmCompiler（AC5/AC6）和 IAR 通过 fputc 重定向 C 库输出。
 * 注意：此处不调用 dbg_lock，因为 dbg_putc_locked 内部已加锁。 */
int fputc(int ch, FILE *f)
{
    (void)f;
    if (!dbg_putc_locked((uint8_t)ch)) {
        return EOF;
    }
    return ch;
}
#endif /* Keil AC5 / AC6 / IAR */

#if defined(__GNUC__) && !defined(__ARMCC_VERSION)
/* 纯 GCC（arm-none-eabi-gcc + newlib）通过弱符号 _write 重定向 printf。
 * armclang 也定义 __GNUC__，但其库使用上方的 fputc，故此处排除。
 * 标记为 weak 允许应用层覆盖（如重定向到 USB-CDC、SD 卡日志等）。 */
__attribute__((weak)) int _write(int fd, const char *buf, int len)
{
    (void)fd;
    if (buf == NULL || len <= 0) {
        return 0;
    }
    int sent = 0;
    for (int i = 0; i < len; ++i) {
        if (!dbg_putc_locked((uint8_t)buf[i])) {
            /* 告知 newlib 实际发出的字节数，以便上层感知部分写入 */
            return sent;
        }
        sent++;
    }
    return sent;
}
#endif /* GCC (non-armclang) */
