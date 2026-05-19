/**
 * @file    pid.h
 * @brief   通用位置式 PID 控制器（带积分限幅）。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_min, output_max;
} PID_t;

/**
 * @brief 初始化 PID 状态和参数。
 * @param integral_limit  积分累加器的对称限幅值。
 * @param out_min/out_max 输出限幅范围。
 */
void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float integral_limit, float out_min, float out_max);

/**
 * @brief 计算一步 PID 输出。
 * @param dt  距上次调用的时间，秒。
 * @return 限幅后的输出，范围 [out_min, out_max]。
 */
float PID_Update(PID_t *pid, float setpoint, float measured, float dt);

/** @brief 清零积分和上次误差，不改变增益。 */
void  PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
