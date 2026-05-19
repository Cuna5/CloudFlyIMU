#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float integral_limit, float out_min, float out_max)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral_limit = integral_limit;
    pid->output_min     = out_min;
    pid->output_max     = out_max;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measured, float dt)
{
    const float error = setpoint - measured;

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    const float derivative = (dt > 0.0f) ? ((error - pid->prev_error) / dt) : 0.0f;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
