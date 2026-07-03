#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct
{
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} PIController_t;

void PIController_Init(PIController_t *pid, float kp, float ki, float out_min, float out_max);
void PIController_Reset(PIController_t *pid);
float PIController_Update(PIController_t *pid, float error, float dt);

#endif
