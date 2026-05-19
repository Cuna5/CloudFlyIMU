/**
 * @file    debug_uart.c
 * @brief   Debug UART driver (USART1) implementation.
 *
 * Spec: .kiro/specs/hardware-base-drivers/{requirements,design,tasks}.md
 *       (task 2.1, Requirements 6.1..6.8).
 *
 * Responsibilities:
 *   - @ref DebugUART_Init verifies the CubeMX-generated @c huart1 handle
 *     points at USART1 and creates an internal serialisation mutex used
 *     by every TX path.  The peripheral itself is *not* re-initialised
 *     (@c MX_USART1_UART_Init in @c Core/Src/usart.c already did that).
 *   - @ref Debug_Log / @ref Debug_Log_Level format into a 256-byte
 *     module-internal buffer with @c vsnprintf, prepend a level prefix
 *     (`[ERR] / [WARN] / [INF] / [DBG] `), and dispatch the buffer to
 *     @c HAL_UART_Transmit with @ref HAL_TIMEOUT_MS.  Output longer than
 *     255 bytes is truncated and a terminating `'\0'` is preserved at
 *     byte 255.
 *   - The runtime filter @c DEBUG_LEVEL_MIN drops messages whose level
 *     is more verbose (numerically greater) than the configured floor.
 *   - When the FreeRTOS scheduler is running, the internal mutex
 *     serialises the formatting buffer and HAL call so multiple tasks
 *     can `printf` concurrently without interleaving.  Before
 *     @c osKernelStart the mutex is bypassed (boot is single-threaded).
 *   - @c printf is redirected to this driver via @c fputc (ARMCC, IAR)
 *     and a weak @c _write (GCC / armclang with newlib) implementation,
 *     so any user-side `printf("...")` ends up on USART1 too.
 *
 * Failure handling: if any of the preconditions for actually emitting
 * the bytes fails (module not init, mutex acquire timeout, HAL TX
 * error, lvl filtered out by `DEBUG_LEVEL_MIN`) the line is silently
 * dropped — *never* is `Debug_Log_Level` called recursively from inside
 * itself.  An internal counter accumulates the drops for diagnostic
 * inspection via @ref Debug_GetDroppedCount.
 */
#include "hardware.h"   /* Driver_Status, DEBUG_LEVEL_t, DEBUG_LEVEL_MIN,
                         * HAL_TIMEOUT_MS, plus debug_uart.h via facade.    */
#include "usart.h"      /* extern UART_HandleTypeDef huart1                */

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/* ================================================================== */
/* Module-private constants                                            */
/* ================================================================== */

/** Format / transmit buffer size, in bytes (Requirement 6.3). */
#define DEBUG_UART_BUF_SIZE         256u


/* ================================================================== */
/* Module-private state                                                */
/* ================================================================== */

/** Static formatting buffer.  Sized exactly per Requirement 6.3 so a
 *  255-byte payload + terminator fits without dynamic allocation.   */
static char            s_buf[DEBUG_UART_BUF_SIZE];

/** Internal serialisation mutex; created lazily in @ref DebugUART_Init. */
static osMutexId_t     s_dbg_mutex;

/** Mutex name attribute kept in static storage so the CMSIS-RTOS
 *  implementation can hold a long-lived pointer.                    */
static const osMutexAttr_t s_dbg_mutex_attr = {
    .name      = "DebugUART",
    .attr_bits = 0u,
    .cb_mem    = NULL,
    .cb_size   = 0u,
};

/** True after a successful @ref DebugUART_Init.  Read by every public
 *  API and the C-library redirection hooks.                          */
static bool            s_initialized;

/** Saturating counter of log lines silently dropped (Requirement
 *  diagnostic counter described in design.md §debug_uart).            */
static uint32_t        s_dropped;


/* ================================================================== */
/* Helpers                                                              */
/* ================================================================== */

static inline bool dbg_scheduler_running(void)
{
    return osKernelGetState() == osKernelRunning;
}

/**
 * @brief Acquire the serialisation mutex if needed.
 *
 * Returns @c true on success (lock taken or skipped because the
 * scheduler is not yet running).  On osMutexAcquire timeout/error the
 * caller MUST drop the line, so a @c false return is returned.
 */
static bool dbg_lock(void)
{
    if (!dbg_scheduler_running()) {
        return true;
    }
    if (s_dbg_mutex == NULL) {
        /* Defensive: should never happen once Init succeeded, but make
         * the failure visible (no lock + no UART) instead of an infinite
         * busy-wait. */
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

/** Increment @ref s_dropped with saturation at @c UINT32_MAX. */
static void dbg_count_drop(void)
{
    if (s_dropped != UINT32_MAX) {
        s_dropped++;
    }
}

/** Pick the level prefix.  Returns "" for unknown levels so a stray
 *  cast from upper-layer code can't crash the buffer math.            */
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
 * @brief Transmit @p len bytes from @p data on USART1.
 *
 * Pre-conditions:
 *   - The serialisation mutex is held (or the scheduler is not running).
 *   - The module is initialised — the caller is responsible for that
 *     check, because the redirection hooks need a fast path that skips
 *     the format step.
 *
 * On HAL failure the dropped counter is incremented and the function
 * returns silently.  We never recurse back into @ref Debug_Log_Level
 * from here.
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
 * @brief Build the level prefix + vformatted body in @ref s_buf and
 *        transmit it.
 *
 * @param lvl  Severity level; treated as filter input as well — values
 *             greater than @c DEBUG_LEVEL_MIN cause an early return
 *             without calling vsnprintf or HAL.
 * @param fmt  printf format string (must not be NULL).
 * @param ap   var-args paired with @p fmt.
 *
 * Mutex / kernel-state handling lives in this function so both the
 * @c Debug_Log* entry points and the @c printf redirection can share it.
 */
static void dbg_emit(DEBUG_LEVEL_t lvl, const char *fmt, va_list ap)
{
    if (!s_initialized || fmt == NULL) {
        dbg_count_drop();
        return;
    }
    /* Requirement 6.7: drop messages whose level is too verbose. */
    if ((int)lvl > (int)DEBUG_LEVEL_MIN) {
        return;
    }

    if (!dbg_lock()) {
        dbg_count_drop();
        return;
    }

    /* Stage 1: copy the level prefix into the front of the buffer.  Use
     * `memcpy` rather than `snprintf` to avoid a second pass through the
     * formatting machinery (the prefix never contains '%'). */
    const char *prefix     = dbg_level_prefix(lvl);
    size_t      prefix_len = strlen(prefix);
    if (prefix_len >= DEBUG_UART_BUF_SIZE) {
        /* Should not happen with the current prefixes (≤ 7 bytes), but
         * be defensive against a future expansion. */
        prefix_len = DEBUG_UART_BUF_SIZE - 1u;
    }
    memcpy(s_buf, prefix, prefix_len);

    /* Stage 2: format the user portion into the remainder.  The vsnprintf
     * contract (C99 §7.19.6.12): writes at most `n - 1` characters then
     * appends '\0', and returns the number of characters that *would*
     * have been written had the buffer been large enough — we use that
     * to detect truncation. */
    size_t remaining = DEBUG_UART_BUF_SIZE - prefix_len;
    int    written  = vsnprintf(s_buf + prefix_len, remaining, fmt, ap);

    size_t body_len;
    if (written < 0) {
        /* Encoding error — emit just the prefix so the level tag is
         * still visible, and count the drop for the body. */
        body_len = 0u;
        dbg_count_drop();
    } else if ((size_t)written >= remaining) {
        /* Requirement 6.4: truncate to 255 chars; vsnprintf already
         * stored the leading 'remaining - 1' bytes plus a '\0' at
         * `s_buf[DEBUG_UART_BUF_SIZE - 1]`, which is exactly the
         * "preserve trailing '\0'" contract. */
        body_len = remaining - 1u;
    } else {
        body_len = (size_t)written;
    }

    size_t total = prefix_len + body_len;
    /* Belt-and-braces: never exceed buffer size, never let body_len wrap. */
    if (total >= DEBUG_UART_BUF_SIZE) {
        total = DEBUG_UART_BUF_SIZE - 1u;
    }
    /* Always keep a final '\0' (mostly for debuggers; UART TX only sends
     * `total` bytes). */
    s_buf[total] = '\0';

    dbg_uart_tx((const uint8_t *)s_buf, (uint16_t)total);
    dbg_unlock();
}


/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

Driver_Status DebugUART_Init(void)
{
    /* Requirement 6.1: do NOT re-init the peripheral; just verify the
     * handle CubeMX gave us actually points at USART1. */
    if (huart1.Instance != USART1) {
        return DRV_ERR_PARAM;
    }

    /* Requirement 6.5: per-driver mutex (created here, freed never —
     * the module is meant to live for the lifetime of the firmware).
     * If a previous Init created one, reuse it so re-init is idempotent. */
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
/* printf redirection                                                  */
/*                                                                      */
/* The C-library `printf` family is funneled through the same dropped-  */
/* line accounting and scheduler-aware locking as Debug_Log_Level.      */
/* Two implementations are provided side-by-side, gated on the          */
/* compiler:                                                            */
/*   - ARMCC v5 / IAR / armclang's "armlink" stdio: implements `fputc`. */
/*   - GCC / armclang with newlib: implements weak `_write`.            */
/* Both push bytes through @c dbg_putc_locked which centralises the     */
/* mutex + HAL_UART_Transmit logic, mirroring @c dbg_emit's contract.   */
/* ================================================================== */

/**
 * @brief Send a single byte through USART1 with the same locking +
 *        drop-counting policy as @ref Debug_Log_Level.
 *
 * Used by both stdio redirections so they get a consistent treatment
 * (no garbled output between tasks; lost bytes counted; safe before
 * @c DebugUART_Init).  Returns @c true if the byte was queued to
 * the HAL successfully, @c false otherwise.
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
/* Keil ArmCompiler (both AC5 and AC6 with armlibc/microlib) and IAR
 * funnel C-library output through @c fputc as the single retarget point.
 * @c __ARMCC_VERSION covers AC5 (e.g. 5060960) and AC6 (e.g. 6160000+);
 * @c __CC_ARM is defined only by AC5, and @c __ICCARM__ by IAR — listing
 * all three keeps the hook compiled in regardless of the toolset.
 *
 * Note: we deliberately do *not* call @c dbg_lock here, because
 * @c dbg_putc_locked already does — @c fputc is one byte per call and we
 * want at least byte-level safety even if the user calls @c printf in a
 * tight loop. */
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
/* Plain GCC (arm-none-eabi-gcc with newlib) funnels stdio through
 * `_write(int fd, const char *buf, int len)` — implementing it (weakly)
 * is the canonical way to retarget @c printf.  Excluded for armclang
 * (which also defines @c __GNUC__) because armclang's library uses the
 * @c fputc hook above instead.  Marked @c weak so an application can
 * override the routing later (USB-CDC, SD-card log, etc.). */
__attribute__((weak)) int _write(int fd, const char *buf, int len)
{
    (void)fd;
    if (buf == NULL || len <= 0) {
        return 0;
    }
    int sent = 0;
    for (int i = 0; i < len; ++i) {
        if (!dbg_putc_locked((uint8_t)buf[i])) {
            /* Tell newlib how many bytes actually made it out so it can
             * surface a partial-write to the caller. */
            return sent;
        }
        sent++;
    }
    return sent;
}
#endif /* GCC (non-armclang) */
