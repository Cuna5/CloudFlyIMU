/**
 * @file    sd_storage.h
 * @brief   SD/FatFs backed application parameter storage.
 */
#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float heat_kp;
    float heat_ki;
    float heat_kd;
    float heat_target_c;

    uint8_t reserved[108];
} AppParams_t;

Driver_Status SDStorage_Init(void);
bool SDStorage_IsReady(void);

Driver_Status Params_Load(AppParams_t *params);
Driver_Status Params_Save(const AppParams_t *params);

#ifdef __cplusplus
}
#endif

#endif /* SD_STORAGE_H */
