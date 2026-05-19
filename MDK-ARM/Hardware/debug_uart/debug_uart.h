/**
 * @file    debug_uart.h
 * @brief   调试串口驱动（USART1）公共 API。
 *
 * 本驱动是 Hardware_Init 中第一个初始化的模块，其他所有模块依赖它输出错误日志。
 * C 库 printf 系列函数通过 debug_uart.c 中的 fputc（ARMCC）/ _write（GCC）
 * 钩子重定向到 USART1。
 *
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 */
#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Driver_Status 和 DEBUG_LEVEL_t 由 common/hardware.h 传递定义，
 * debug_uart.c 中通过先包含 hardware.h 使这些类型可见。 */

/**
 * @brief 初始化调试串口驱动。
 *
 * 验证 CubeMX 生成的 huart1 句柄绑定到 USART1（不重新初始化外设），
 * 通过 osMutexNew 创建内部串行化互斥锁，重置丢弃计数器，标记模块就绪。
 *
 * @return DRV_OK 成功；DRV_ERR_PARAM huart1.Instance 不指向 USART1；
 *         DRV_ERR_NOT_INIT osMutexNew 失败（RTOS 内存不足），模块保持禁用。
 */
Driver_Status DebugUART_Init(void);

/**
 * @brief 以 DBG_INFO 等级格式化并发送一行日志。
 *
 * 等价于 Debug_Log_Level(DBG_INFO, fmt, ...)，用于简便的 printf 风格跟踪。
 *
 * @param fmt  C99 printf 格式字符串，不得为 NULL。
 */
void Debug_Log(const char *fmt, ...);

/**
 * @brief 以指定严重等级格式化并发送一行日志。
 *
 * 输出前缀按 lvl 固定：
 *   - DBG_ERR   → "[ERR] "
 *   - DBG_WARN  → "[WARN] "
 *   - DBG_INFO  → "[INF] "
 *   - DBG_DEBUG → "[DBG] "
 *
 * lvl 比 DEBUG_LEVEL_MIN 更冗长（数值更大）的消息被静默丢弃。
 * 总输出（前缀 + 用户内容）写入 256 字节静态缓冲区，超长内容截断，
 * 末尾保留 '\0'。
 * FreeRTOS 调度器运行时持内部互斥锁；调度器未启动时跳过加锁。
 * HAL_UART_Transmit 失败时静默丢弃并累加计数器。
 *
 * @param lvl  严重等级；超出 DEBUG_LEVEL_t 范围的值视为过于冗长，调用变为空操作。
 * @param fmt  C99 printf 格式字符串，不得为 NULL。
 */
void Debug_Log_Level(DEBUG_LEVEL_t lvl, const char *fmt, ...);

/**
 * @brief 读取因未初始化、互斥锁超时或 HAL 发送失败而静默丢弃的日志行计数。
 *
 * 可用于诊断仪表盘检测 UART 饱和。计数器在 UINT32_MAX 处饱和，
 * 仅在 DebugUART_Init 时重置。
 */
uint32_t Debug_GetDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H */
