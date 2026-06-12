/**
 * @file    oled.h
 * @brief   SSD1306 OLED 显示驱动（SPI2，128×64）公共 API。
 *
 * 提供帧缓冲区操作（像素/线/矩形/位图/文字），
 * 调用 OLED_Update 后将缓冲区一次性刷新到屏幕。
 * 本头文件通过 common/hardware.h 聚合导出，应用层不应直接包含本文件。
 */
/* Do not include directly - use hardware.h */
#ifndef OLED_H
#define OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 屏幕分辨率（可在编译时覆盖） */
#ifndef OLED_WIDTH
#define OLED_WIDTH   128U
#endif

#ifndef OLED_HEIGHT
#define OLED_HEIGHT  64U
#endif

/* SSD1306 初始化参数（可在编译时覆盖） */
#ifndef OLED_DISPLAY_CLOCK
#define OLED_DISPLAY_CLOCK  0x80U
#endif

#ifndef OLED_MEMORY_MODE
#define OLED_MEMORY_MODE    0x02U   /* 页寻址模式 */
#endif

#ifndef OLED_COM_PINS
#define OLED_COM_PINS       0x12U
#endif

#ifndef OLED_CONTRAST
#define OLED_CONTRAST       0xCFU
#endif

#ifndef OLED_PRECHARGE
#define OLED_PRECHARGE      0xF1U
#endif

#ifndef OLED_VCOMH
#define OLED_VCOMH          0x20U
#endif

/** 像素颜色枚举。 */
typedef enum {
    OLED_COLOR_OFF    = 0,  /**< 关闭像素（黑）。 */
    OLED_COLOR_ON     = 1,  /**< 点亮像素（白）。 */
    OLED_COLOR_INVERT = 2,  /**< 翻转像素。 */
} OLED_Color_t;

/* ------------------------------------------------------------------ */
/* 初始化与刷新                                                         */
/* ------------------------------------------------------------------ */

/** @brief 初始化 SSD1306，发送初始化命令序列，清空帧缓冲区。 */
Driver_Status OLED_Init(void);

/** @brief 将帧缓冲区内容通过 SPI 刷新到屏幕。 */
Driver_Status OLED_Update(void);

/** @brief 清空帧缓冲区（全黑），不自动刷新屏幕。 */
Driver_Status OLED_Clear(void);

/** @brief 用指定颜色填充整个帧缓冲区，不自动刷新屏幕。 */
Driver_Status OLED_Fill(OLED_Color_t color);

/* ------------------------------------------------------------------ */
/* 显示控制                                                             */
/* ------------------------------------------------------------------ */

/** @brief 开启或关闭显示输出（不影响帧缓冲区内容）。 */
Driver_Status OLED_DisplayOn(bool on);

/** @brief 反色显示开关。 */
Driver_Status OLED_Invert(bool invert);

/** @brief 设置对比度（0x00~0xFF）。 */
Driver_Status OLED_SetContrast(uint8_t contrast);

/* ------------------------------------------------------------------ */
/* 绘图 API（操作帧缓冲区，需调用 OLED_Update 才可见）                 */
/* ------------------------------------------------------------------ */

/** @brief 绘制单个像素。 */
Driver_Status OLED_DrawPixel(uint8_t x, uint8_t y, OLED_Color_t color);

/** @brief 绘制直线（Bresenham 算法）。 */
Driver_Status OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                            OLED_Color_t color);

/** @brief 绘制空心矩形。 */
Driver_Status OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                            OLED_Color_t color);

/** @brief 绘制填充矩形。 */
Driver_Status OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                            OLED_Color_t color);

/**
 * @brief 绘制单色位图（列优先，每字节 8 行）。
 * @param bitmap  位图数据，长度 = w * ((h+7)/8)。
 */
Driver_Status OLED_DrawBitmap(uint8_t x, uint8_t y,
                              const uint8_t *bitmap, uint8_t w, uint8_t h,
                              OLED_Color_t color);

/* ------------------------------------------------------------------ */
/* 文字 API                                                             */
/* ------------------------------------------------------------------ */

/** @brief 设置文字光标位置（像素坐标）。 */
Driver_Status OLED_SetCursor(uint8_t x, uint8_t y);

/** @brief 在当前光标位置写入单个 ASCII 字符（5×7 点阵）。 */
Driver_Status OLED_WriteChar(char ch, OLED_Color_t color);

/** @brief 在当前光标位置写入 ASCII 字符串，支持 '\n' 换行。 */
Driver_Status OLED_WriteString(const char *str, OLED_Color_t color);

/** @brief printf 风格格式化后写入字符串（内部缓冲区 64 字节）。 */
Driver_Status OLED_Printf(OLED_Color_t color, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
