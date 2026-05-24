/* Do not include directly — use hardware.h */
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

Driver_Status LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
