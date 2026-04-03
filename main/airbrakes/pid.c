#include "pid.h"

void init_pid(PIDController *pid)
{

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->differentiator = 0.0f;

    pid->output = 0.0f;
}

float update_pid(PIDController *pid, float error)
{
    float proportional = pid->kp * error;

    pid->integral = pid->integral + 0.5f * pid->ki * pid->T * (error + pid->prev_error);

    pid->differentiator =
        (2.0f * pid->kd * (error - pid->prev_error) +
        (2.0f * pid->T - pid->kd) * pid->differentiator) /
        (2.0f * pid->T + pid->kd);

    pid->output = proportional + pid->integral + pid->differentiator;

    if (pid->output > pid->limit_max) {
        pid->output = pid->limit_max;
    } else if (pid->output < pid->limit_min) {
        pid->output = pid->limit_min;
    }

    pid->prev_error = error;
    return pid->output;
}