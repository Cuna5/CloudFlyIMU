/**
 * @file    oled.c
 * @brief   SSD1306 OLED 显示驱动（SPI2，128×64）实现。
 *
 * 采用帧缓冲区策略：所有绘图操作写入 s_buffer，
 * 调用 OLED_Update 后通过 SPI 一次性刷新到屏幕（页寻址模式）。
 * FreeRTOS 调度器运行时通过 s_oled_mutex 串行化并发访问。
 */
#include "hardware.h"
#include "main.h"
#include "spi.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define OLED_PAGE_COUNT        (OLED_HEIGHT / 8U)           /* 页数 = 高度/8 */
#define OLED_FRAMEBUFFER_SIZE  (OLED_WIDTH * OLED_PAGE_COUNT)
#define OLED_CHAR_WIDTH        6U   /* 字符宽度（5 像素 + 1 间距） */
#define OLED_CHAR_HEIGHT       8U   /* 字符高度 */

static bool s_initialized = false;
static uint8_t s_buffer[OLED_FRAMEBUFFER_SIZE];  /* 帧缓冲区 */
static uint8_t s_cursor_x = 0U;
static uint8_t s_cursor_y = 0U;
static osMutexId_t s_oled_mutex;
static const osMutexAttr_t s_oled_mutex_attr = {
    .name = "OLEDMutex",
};

static const uint8_t s_font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18},
    {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00},
    {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C},
    {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00},
    {0x08,0x08,0x2A,0x1C,0x08}, {0x08,0x1C,0x2A,0x08,0x08},
};

static void oled_delay_ms(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

static Driver_Status oled_lock(void)
{
    if (osKernelGetState() != osKernelRunning) {
        return DRV_OK;
    }

    if (s_oled_mutex == NULL) {
        s_oled_mutex = osMutexNew(&s_oled_mutex_attr);
        if (s_oled_mutex == NULL) {
            return DRV_ERR_NOT_INIT;
        }
    }

    return (osMutexAcquire(s_oled_mutex, HAL_TIMEOUT_MS) == osOK)
        ? DRV_OK
        : DRV_ERR_TIMEOUT;
}

static void oled_unlock(void)
{
    if ((osKernelGetState() == osKernelRunning) && (s_oled_mutex != NULL)) {
        (void)osMutexRelease(s_oled_mutex);
    }
}

static Driver_Status oled_write(bool data_mode, const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return DRV_ERR_PARAM;
    }
    if (hspi2.Instance != SPI2) {
        return DRV_ERR_NOT_INIT;
    }

    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin,
                      data_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hs = HAL_SPI_Transmit(&hspi2, (uint8_t *)data, len, HAL_TIMEOUT_MS);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
    return Driver_MapHalStatus(hs);
}

static Driver_Status oled_cmd(uint8_t cmd)
{
    return oled_write(false, &cmd, 1U);
}

static Driver_Status oled_cmd_list(const uint8_t *cmds, uint16_t len)
{
    return oled_write(false, cmds, len);
}

static void oled_draw_pixel_unlocked(uint8_t x, uint8_t y, OLED_Color_t color)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return;
    }

    uint16_t index = (uint16_t)x + ((uint16_t)(y / 8U) * OLED_WIDTH);
    uint8_t mask = (uint8_t)(1U << (y & 7U));

    if (color == OLED_COLOR_ON) {
        s_buffer[index] |= mask;
    } else if (color == OLED_COLOR_OFF) {
        s_buffer[index] &= (uint8_t)~mask;
    } else {
        s_buffer[index] ^= mask;
    }
}

static void oled_draw_char_unlocked(char ch, OLED_Color_t color)
{
    if ((ch < ' ') || (ch > '~')) {
        ch = '?';
    }

    OLED_Color_t bg = (color == OLED_COLOR_OFF) ? OLED_COLOR_ON : OLED_COLOR_OFF;

    for (uint8_t col = 0U; col < OLED_CHAR_WIDTH; col++) {
        uint8_t bits = (col < 5U) ? s_font5x7[(uint8_t)ch - 32U][col] : 0U;
        for (uint8_t row = 0U; row < OLED_CHAR_HEIGHT; row++) {
            bool pixel_on = (bits & (uint8_t)(1U << row)) != 0U;
            oled_draw_pixel_unlocked((uint8_t)(s_cursor_x + col),
                                     (uint8_t)(s_cursor_y + row),
                                     pixel_on ? color : bg);
        }
    }

    s_cursor_x = (uint8_t)(s_cursor_x + OLED_CHAR_WIDTH);
}

Driver_Status OLED_Init(void)
{
    if (hspi2.Instance != SPI2) {
        return DRV_ERR_NOT_INIT;
    }

    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
    oled_delay_ms(20U);
    HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
    oled_delay_ms(100U);

    static const uint8_t init_cmds[] = {
        0xAE,       /* display off */
        0xD5, OLED_DISPLAY_CLOCK, /* display clock */
        0xA8, (OLED_HEIGHT - 1U), /* multiplex */
        0xD3, 0x00, /* display offset */
        0x40,       /* start line */
        0x8D, 0x14, /* charge pump on */
        0x20, OLED_MEMORY_MODE, /* page addressing */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan direction remap */
        0xDA, OLED_COM_PINS, /* COM pins */
        0x81, OLED_CONTRAST, /* contrast */
        0xD9, OLED_PRECHARGE, /* precharge */
        0xDB, OLED_VCOMH, /* VCOMH */
        0xA4,       /* resume RAM content */
        0xA6,       /* normal display */
        0x2E,       /* deactivate scroll */
        0xAF,       /* display on */
    };

    Driver_Status st = oled_cmd_list(init_cmds, sizeof(init_cmds));
    if (st != DRV_OK) {
        s_initialized = false;
        return st;
    }

    memset(s_buffer, 0, sizeof(s_buffer));
    s_cursor_x = 0U;
    s_cursor_y = 0U;
    s_initialized = true;
    return OLED_Update();
}

Driver_Status OLED_Update(void)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }

    for (uint8_t page = 0U; page < OLED_PAGE_COUNT; page++) {
        uint8_t cmds[] = {
            (uint8_t)(0xB0U | page),
            0x00,
            0x10,
        };
        st = oled_cmd_list(cmds, sizeof(cmds));
        if (st == DRV_OK) {
            st = oled_write(true, &s_buffer[(uint16_t)page * OLED_WIDTH], OLED_WIDTH);
        }
        if (st != DRV_OK) {
            break;
        }
    }

    oled_unlock();
    return st;
}

Driver_Status OLED_Clear(void)
{
    return OLED_Fill(OLED_COLOR_OFF);
}

Driver_Status OLED_Fill(OLED_Color_t color)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }

    if (color == OLED_COLOR_INVERT) {
        for (uint16_t i = 0U; i < OLED_FRAMEBUFFER_SIZE; i++) {
            s_buffer[i] = (uint8_t)~s_buffer[i];
        }
    } else {
        memset(s_buffer, (color == OLED_COLOR_ON) ? 0xFF : 0x00, sizeof(s_buffer));
    }
    s_cursor_x = 0U;
    s_cursor_y = 0U;

    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_DisplayOn(bool on)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }
    st = oled_cmd(on ? 0xAF : 0xAE);
    oled_unlock();
    return st;
}

Driver_Status OLED_Invert(bool invert)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }
    st = oled_cmd(invert ? 0xA7 : 0xA6);
    oled_unlock();
    return st;
}

Driver_Status OLED_SetContrast(uint8_t contrast)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }
    uint8_t cmds[] = {0x81, contrast};
    st = oled_cmd_list(cmds, sizeof(cmds));
    oled_unlock();
    return st;
}

Driver_Status OLED_DrawPixel(uint8_t x, uint8_t y, OLED_Color_t color)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }
    oled_draw_pixel_unlocked(x, y, color);
    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                            OLED_Color_t color)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }

    int16_t dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t dy = (y0 < y1) ? (y0 - y1) : (y1 - y0);
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = (int16_t)(dx + dy);

    for (;;) {
        if ((x0 >= 0) && (x0 < (int16_t)OLED_WIDTH) &&
            (y0 >= 0) && (y0 < (int16_t)OLED_HEIGHT)) {
            oled_draw_pixel_unlocked((uint8_t)x0, (uint8_t)y0, color);
        }
        if ((x0 == x1) && (y0 == y1)) {
            break;
        }
        int16_t e2 = (int16_t)(2 * err);
        if (e2 >= dy) {
            err = (int16_t)(err + dy);
            x0 = (int16_t)(x0 + sx);
        }
        if (e2 <= dx) {
            err = (int16_t)(err + dx);
            y0 = (int16_t)(y0 + sy);
        }
    }

    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                            OLED_Color_t color)
{
    if ((w == 0U) || (h == 0U)) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = OLED_DrawLine(x, y, (int16_t)(x + w - 1U), y, color);
    if (st != DRV_OK) return st;
    st = OLED_DrawLine(x, (int16_t)(y + h - 1U),
                       (int16_t)(x + w - 1U), (int16_t)(y + h - 1U), color);
    if (st != DRV_OK) return st;
    st = OLED_DrawLine(x, y, x, (int16_t)(y + h - 1U), color);
    if (st != DRV_OK) return st;
    return OLED_DrawLine((int16_t)(x + w - 1U), y,
                         (int16_t)(x + w - 1U), (int16_t)(y + h - 1U), color);
}

Driver_Status OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                            OLED_Color_t color)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if ((w == 0U) || (h == 0U)) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }

    for (uint8_t yy = 0U; yy < h; yy++) {
        for (uint8_t xx = 0U; xx < w; xx++) {
            uint16_t px = (uint16_t)x + xx;
            uint16_t py = (uint16_t)y + yy;
            if ((px < OLED_WIDTH) && (py < OLED_HEIGHT)) {
                oled_draw_pixel_unlocked((uint8_t)px, (uint8_t)py, color);
            }
        }
    }

    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_DrawBitmap(uint8_t x, uint8_t y,
                              const uint8_t *bitmap, uint8_t w, uint8_t h,
                              OLED_Color_t color)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if ((bitmap == NULL) || (w == 0U) || (h == 0U)) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }

    for (uint8_t yy = 0U; yy < h; yy++) {
        for (uint8_t xx = 0U; xx < w; xx++) {
            uint16_t bit_index = (uint16_t)yy * w + xx;
            uint8_t b = bitmap[bit_index / 8U];
            if ((b & (uint8_t)(0x80U >> (bit_index & 7U))) != 0U) {
                uint16_t px = (uint16_t)x + xx;
                uint16_t py = (uint16_t)y + yy;
                if ((px < OLED_WIDTH) && (py < OLED_HEIGHT)) {
                    oled_draw_pixel_unlocked((uint8_t)px, (uint8_t)py, color);
                }
            }
        }
    }

    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_SetCursor(uint8_t x, uint8_t y)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }
    s_cursor_x = x;
    s_cursor_y = y;
    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_WriteChar(char ch, OLED_Color_t color)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }

    if ((s_cursor_x + OLED_CHAR_WIDTH) > OLED_WIDTH) {
        s_cursor_x = 0U;
        s_cursor_y = (uint8_t)(s_cursor_y + OLED_CHAR_HEIGHT);
    }
    if ((s_cursor_y + OLED_CHAR_HEIGHT) > OLED_HEIGHT) {
        oled_unlock();
        return DRV_ERR_PARAM;
    }

    oled_draw_char_unlocked(ch, color);
    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_WriteString(const char *str, OLED_Color_t color)
{
    if (!s_initialized) {
        return DRV_ERR_NOT_INIT;
    }
    if (str == NULL) {
        return DRV_ERR_PARAM;
    }

    Driver_Status st = oled_lock();
    if (st != DRV_OK) {
        return st;
    }

    while (*str != '\0') {
        if (*str == '\n') {
            s_cursor_x = 0U;
            s_cursor_y = (uint8_t)(s_cursor_y + OLED_CHAR_HEIGHT);
        } else {
            if ((s_cursor_x + OLED_CHAR_WIDTH) > OLED_WIDTH) {
                s_cursor_x = 0U;
                s_cursor_y = (uint8_t)(s_cursor_y + OLED_CHAR_HEIGHT);
            }
            if ((s_cursor_y + OLED_CHAR_HEIGHT) > OLED_HEIGHT) {
                oled_unlock();
                return DRV_ERR_PARAM;
            }
            oled_draw_char_unlocked(*str, color);
        }
        str++;
    }

    oled_unlock();
    return DRV_OK;
}

Driver_Status OLED_Printf(OLED_Color_t color, const char *fmt, ...)
{
    if (fmt == NULL) {
        return DRV_ERR_PARAM;
    }

    char text[64];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);

    return OLED_WriteString(text, color);
}
