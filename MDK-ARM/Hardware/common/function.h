#ifndef FUNCTION_H
#define FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hardware.h"

void Task_Sensor(void);
void Task_Heat(void);
void Task_MahonyFusion(void);
void Task_EKFFusion(void);
void Task_Debug(void);

#ifdef __cplusplus
}
#endif

#endif /* FUNCTION_H */
