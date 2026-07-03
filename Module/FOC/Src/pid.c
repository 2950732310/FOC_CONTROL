#include "pid.h"

void PIController_Init(PIController_t *pid, float kp, float ki, float out_min, float out_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->integral = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PIController_Reset(PIController_t *pid)
{
    if (pid != NULL)
    {
        pid->integral = 0.0f;
    }
}

float PIController_Update(PIController_t *pid, float error, float dt)
{
    float out;

    if (pid == NULL || dt <= 0.0f)
    {
        return 0.0f;
    }

    pid->integral += error * dt;
    out = pid->kp * error + pid->ki * pid->integral;

    if (out > pid->out_max)
    {
        out = pid->out_max;
    }
    else if (out < pid->out_min)
    {
        out = pid->out_min;
    }

    return out;
}
