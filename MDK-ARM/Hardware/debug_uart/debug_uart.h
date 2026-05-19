/**
 * @file    debug_uart.h
 * @brief   Debug UART driver (USART1) public API.
 *
 * Implements the Debug_UART_Driver block of the @c hardware-base-drivers
 * spec (Requirement 6).  The driver is the *first* one initialised by
 * @ref Hardware_Init: every other module relies on it for error logging,
 * and the C-library `printf` family is redirected to it via the
 * @c fputc (ARMCC) / @c _write (GCC) hooks defined in @c debug_uart.c.
 *
 * Like every other driver header in @c MDK-ARM/Hardware/, this file is
 * part of the aggregate facade @c common/hardware.h: the aggregate header
 * defines @ref Driver_Status / @ref DEBUG_LEVEL_t / @c HAL_TIMEOUT_MS /
 * @c DEBUG_LEVEL_MIN and then `#include`s this file.  We therefore do
 * *not* include @c hardware.h here to avoid a circular include — the
 * implementation in @c debug_uart.c includes the aggregate first.
 *
 * Validates: Requirements 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 6.8.
 */
#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Driver_Status and DEBUG_LEVEL_t are provided transitively by
 * `common/hardware.h`, which includes this header last.  Compiling
 * `debug_uart.c` directly therefore includes `hardware.h` first to make
 * those types visible. */

/**
 * @brief Initialise the debug UART driver.
 *
 * Verifies that the CubeMX-generated @c huart1 handle is bound to USART1
 * (peripheral re-init is *not* performed — `MX_USART1_UART_Init` has
 * already done that), creates the internal serialisation mutex via
 * @c osMutexNew, resets the dropped-line counter, and marks the module
 * as ready.
 *
 * @retval DRV_OK            on success.
 * @retval DRV_ERR_PARAM     `huart1.Instance` does not point at USART1.
 * @retval DRV_ERR_NOT_INIT  @c osMutexNew failed (e.g. out of RTOS
 *                           memory); the module is left disabled.
 *
 * Validates: Requirements 6.1, 6.5.
 */
Driver_Status DebugUART_Init(void);

/**
 * @brief Format and transmit a log line at the @ref DBG_INFO level.
 *
 * Equivalent to @c Debug_Log_Level(DBG_INFO, fmt, ...).  Provided as a
 * convenience for casual @c printf-style tracing.
 *
 * @param fmt  C99 printf format string.  Must not be NULL.
 *
 * Validates: Requirements 6.3, 6.4.
 */
void Debug_Log(const char *fmt, ...);

/**
 * @brief Format and transmit a log line at the requested severity.
 *
 * Output is prefixed by a fixed-width tag according to @p lvl:
 *   - @ref DBG_ERR   → @c "[ERR] "
 *   - @ref DBG_WARN  → @c "[WARN] "
 *   - @ref DBG_INFO  → @c "[INF] "
 *   - @ref DBG_DEBUG → @c "[DBG] "
 *
 * Lines whose @p lvl is *more verbose* (numerically greater) than
 * @c DEBUG_LEVEL_MIN are silently dropped — no formatting, no UART
 * traffic.
 *
 * The total payload (prefix + formatted user content) is written to a
 * 256-byte module-internal buffer through @c vsnprintf.  Output longer
 * than 255 bytes is truncated and a terminating @c '\0' is preserved at
 * byte 255; only the truncated content is transmitted.
 *
 * The function takes the internal serialisation mutex while the FreeRTOS
 * scheduler is running, and bypasses the lock entirely before
 * @c osKernelStart.  If @c HAL_UART_Transmit fails the line is dropped
 * silently and an internal counter is incremented (queryable via
 * @ref Debug_GetDroppedCount).
 *
 * @param lvl  Severity level; values outside @ref DEBUG_LEVEL_t are
 *             treated as too-verbose and the call becomes a no-op.
 * @param fmt  C99 printf format string.  Must not be NULL.
 *
 * Validates: Requirements 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 6.8.
 */
void Debug_Log_Level(DEBUG_LEVEL_t lvl, const char *fmt, ...);

/**
 * @brief Read the internal counter of log lines silently dropped because
 *        the formatted output could not be transmitted (HAL error,
 *        mutex acquisition timeout, or module-not-initialised path).
 *
 * Useful for diagnostic dashboards to detect a saturated UART.  The
 * counter saturates at @c UINT32_MAX and is reset only by a fresh
 * @ref DebugUART_Init.
 */
uint32_t Debug_GetDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H */
