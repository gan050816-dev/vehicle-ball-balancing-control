#ifndef MOTOR_SAFETY_H
#define MOTOR_SAFETY_H

#include <stdint.h>

typedef enum
{
    MOTOR_MODE_DRIVE = 0,
    MOTOR_MODE_BRAKE
} MotorMode;

typedef struct
{
    MotorMode mode;
    uint8_t in1;
    uint8_t in2;
    uint16_t pwm_compare;
} MotorOutput;

typedef struct
{
    int16_t minimum_target;
    int16_t maximum_stalled_speed;
    uint16_t required_ticks;
    uint16_t bad_ticks;
} StallMonitor;

typedef struct
{
    int8_t last_sign;
    uint8_t brake_ticks_remaining;
    uint8_t brake_active;
} MotorDirectionGuard;

typedef struct
{
    int16_t command;
} MotorPwmRamp;

MotorOutput MotorSafety_MakeOutput(int16_t command,
                                   uint16_t pwm_period,
                                   uint8_t invert_direction,
                                   uint8_t emergency_brake);
void StallMonitor_Init(StallMonitor *monitor,
                       int16_t minimum_target,
                       int16_t maximum_stalled_speed,
                       uint16_t required_ticks);
uint8_t StallMonitor_Update(StallMonitor *monitor,
                            int16_t target_speed,
                            int16_t measured_speed);
void MotorDirectionGuard_Init(MotorDirectionGuard *guard);
int16_t MotorDirectionGuard_Apply(MotorDirectionGuard *guard,
                                  int16_t command,
                                  uint8_t brake_ticks);
uint8_t MotorDirectionGuard_IsBraking(const MotorDirectionGuard *guard);
void MotorPwmRamp_Init(MotorPwmRamp *ramp);
void MotorPwmRamp_Reset(MotorPwmRamp *ramp);
int16_t MotorPwmRamp_Apply(MotorPwmRamp *ramp,
                           int16_t requested_command,
                           uint16_t maximum_step);

#endif
