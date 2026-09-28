#include "speed_pid.h"

static float clamp_symmetric(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void SpeedPid_Init(SpeedPid *pid,
                   float kp,
                   float ki,
                   float kd,
                   float output_limit,
                   float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    SpeedPid_Reset(pid);
}

void SpeedPid_Reset(SpeedPid *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous = 0U;
}

float SpeedPid_Update(SpeedPid *pid,
                      float target,
                      float measured,
                      float dt_seconds)
{
    float error = target - measured;
    float derivative = 0.0f;

    if (dt_seconds > 0.0f)
    {
        pid->integral = clamp_symmetric(pid->integral + error * dt_seconds,
                                        pid->integral_limit);
        if (pid->has_previous != 0U)
        {
            derivative = (error - pid->previous_error) / dt_seconds;
        }
    }

    pid->previous_error = error;
    pid->has_previous = 1U;

    return clamp_symmetric(pid->kp * error +
                               pid->ki * pid->integral +
                               pid->kd * derivative,
                           pid->output_limit);
}
