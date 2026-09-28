#include "motor_safety.h"

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

MotorOutput MotorSafety_MakeOutput(int16_t command,
                                   uint16_t pwm_period,
                                   uint8_t invert_direction,
                                   uint8_t emergency_brake)
{
    MotorOutput output;

    if (emergency_brake != 0U || command == 0)
    {
        output.mode = MOTOR_MODE_BRAKE;
        output.in1 = 1U;
        output.in2 = 1U;
        output.pwm_compare = pwm_period;
        return output;
    }

    output.mode = MOTOR_MODE_DRIVE;
    output.in1 = (command > 0) ? 1U : 0U;
    output.in2 = (uint8_t)!output.in1;
    if (invert_direction != 0U)
    {
        uint8_t temporary = output.in1;
        output.in1 = output.in2;
        output.in2 = temporary;
    }

    output.pwm_compare = (uint16_t)abs_i16(command);
    if (output.pwm_compare > pwm_period)
    {
        output.pwm_compare = pwm_period;
    }
    return output;
}

void StallMonitor_Init(StallMonitor *monitor,
                       int16_t minimum_target,
                       int16_t maximum_stalled_speed,
                       uint16_t required_ticks)
{
    monitor->minimum_target = minimum_target;
    monitor->maximum_stalled_speed = maximum_stalled_speed;
    monitor->required_ticks = required_ticks;
    monitor->bad_ticks = 0U;
}

uint8_t StallMonitor_Update(StallMonitor *monitor,
                            int16_t target_speed,
                            int16_t measured_speed)
{
    if (abs_i16(target_speed) >= monitor->minimum_target &&
        abs_i16(measured_speed) <= monitor->maximum_stalled_speed)
    {
        if (monitor->bad_ticks < monitor->required_ticks)
        {
            ++monitor->bad_ticks;
        }
    }
    else
    {
        monitor->bad_ticks = 0U;
    }

    return (monitor->required_ticks != 0U &&
            monitor->bad_ticks >= monitor->required_ticks)
               ? 1U
               : 0U;
}

void MotorDirectionGuard_Init(MotorDirectionGuard *guard)
{
    guard->last_sign = 0;
    guard->brake_ticks_remaining = 0U;
    guard->brake_active = 0U;
}

int16_t MotorDirectionGuard_Apply(MotorDirectionGuard *guard,
                                  int16_t command,
                                  uint8_t brake_ticks)
{
    int8_t sign = command > 0 ? 1 : (command < 0 ? -1 : 0);

    guard->brake_active = 0U;
    if (guard->brake_ticks_remaining != 0U)
    {
        --guard->brake_ticks_remaining;
        guard->brake_active = 1U;
        return 0;
    }
    if (sign == 0)
    {
        return 0;
    }
    if (guard->last_sign != 0 && sign != guard->last_sign)
    {
        guard->last_sign = sign;
        if (brake_ticks != 0U)
        {
            guard->brake_ticks_remaining = (uint8_t)(brake_ticks - 1U);
            guard->brake_active = 1U;
        }
        return 0;
    }
    guard->last_sign = sign;
    return command;
}

uint8_t MotorDirectionGuard_IsBraking(const MotorDirectionGuard *guard)
{
    return guard->brake_active;
}

void MotorPwmRamp_Init(MotorPwmRamp *ramp)
{
    ramp->command = 0;
}

void MotorPwmRamp_Reset(MotorPwmRamp *ramp)
{
    ramp->command = 0;
}

int16_t MotorPwmRamp_Apply(MotorPwmRamp *ramp,
                           int16_t requested_command,
                           uint16_t maximum_step)
{
    int32_t difference = (int32_t)requested_command - ramp->command;

    if (maximum_step == 0U)
    {
        return ramp->command;
    }
    if (difference > (int32_t)maximum_step)
    {
        ramp->command = (int16_t)(ramp->command + maximum_step);
    }
    else if (difference < -(int32_t)maximum_step)
    {
        ramp->command = (int16_t)(ramp->command - maximum_step);
    }
    else
    {
        ramp->command = requested_command;
    }
    return ramp->command;
}
