/* Do not include directly - use hardware.h */
#ifndef OLED_H
#define OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef OLED_WIDTH
#define OLED_WIDTH   128U
#endif

#ifndef OLED_HEIGHT
#define OLED_HEIGHT  64U
#endif

#ifndef OLED_DISPLAY_CLOCK
#define OLED_DISPLAY_CLOCK  0x80U
#endif

#ifndef OLED_MEMORY_MODE
#define OLED_MEMORY_MODE    0x02U
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

typedef enum {
    OLED_COLOR_OFF    = 0,
    OLED_COLOR_ON     = 1,
    OLED_COLOR_INVERT = 2,
} OLED_Color_t;

Driver_Status OLED_Init(void);
Driver_Status OLED_Update(void);
Driver_Status OLED_Clear(void);
Driver_Status OLED_Fill(OLED_Color_t color);

Driver_Status OLED_DisplayOn(bool on);
Driver_Status OLED_Invert(bool invert);
Driver_Status OLED_SetContrast(uint8_t contrast);

Driver_Status OLED_DrawPixel(uint8_t x, uint8_t y, OLED_Color_t color);
Driver_Status OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                            OLED_Color_t color);
Driver_Status OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                            OLED_Color_t color);
Driver_Status OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                            OLED_Color_t color);
Driver_Status OLED_DrawBitmap(uint8_t x, uint8_t y,
                              const uint8_t *bitmap, uint8_t w, uint8_t h,
                              OLED_Color_t color);

Driver_Status OLED_SetCursor(uint8_t x, uint8_t y);
Driver_Status OLED_WriteChar(char ch, OLED_Color_t color);
Driver_Status OLED_WriteString(const char *str, OLED_Color_t color);
Driver_Status OLED_Printf(OLED_Color_t color, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
