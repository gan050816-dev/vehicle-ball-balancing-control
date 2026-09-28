#ifndef LINE_DRIVE_SERVICE_H
#define LINE_DRIVE_SERVICE_H

#include <stdint.h>

#include "drive_logic.h"
#include "motor_safety.h"
#include "speed_pid.h"

typedef enum
{
    LINE_DRIVE_MODE_T2 = 0,
    LINE_DRIVE_MODE_REQUIREMENT_4,
    LINE_DRIVE_MODE_REQUIREMENT_5
} LineDriveMode;

typedef enum
{
    LINE_DRIVE_IDLE = 0,
    LINE_DRIVE_RUNNING,
    LINE_DRIVE_STOPPED,
    LINE_DRIVE_FAULT
} LineDriveState;

typedef enum
{
    LINE_DRIVE_STOP_NONE = 0,
    LINE_DRIVE_STOP_FINISH_LINE,
    LINE_DRIVE_STOP_CALIBRATED_TIME,
    LINE_DRIVE_STOP_TIMEOUT,
    LINE_DRIVE_STOP_LINE_LOST,
    LINE_DRIVE_STOP_STALL,
    LINE_DRIVE_STOP_EMERGENCY
} LineDriveStopReason;

typedef struct
{
    LineDriveMode mode;
    LineDriveState state;
    LineDriveStopReason stop_reason;
    uint32_t elapsed_ticks;
    uint16_t lost_line_ticks;
    uint8_t finish_confirm_ticks;
    int16_t previous_line_error;
    int16_t last_nonzero_line_error;
    int8_t applied_steering_sign;
    int8_t pending_steering_sign;
    uint8_t steering_reversal_ticks;
    int32_t previous_left_count;
    int32_t previous_right_count;
    int16_t requested_forward_speed_mm_s;
    int16_t forward_speed_mm_s;
    int16_t applied_differential_mm_s;
    int16_t target_left_mm_s;
    int16_t target_right_mm_s;
    int16_t measured_left_mm_s;
    int16_t measured_right_mm_s;
    uint16_t peak_acceleration_mm_s2;
    DriveMotionProfile forward_profile;
    SpeedPid left_pid;
    SpeedPid right_pid;
    StallMonitor left_stall;
    StallMonitor right_stall;
    MotorDirectionGuard left_direction_guard;
    MotorDirectionGuard right_direction_guard;
    MotorPwmRamp left_pwm_ramp;
    MotorPwmRamp right_pwm_ramp;
    MotorOutput left_output;
    MotorOutput right_output;
} LineDriveService;

void LineDriveService_Init(LineDriveService *service);
uint8_t LineDriveService_Start(LineDriveService *service,
                               LineDriveMode mode,
                               int32_t left_count,
                               int32_t right_count);
void LineDriveService_Update(LineDriveService *service,
                             uint8_t line_mask,
                             int32_t left_count,
                             int32_t right_count);
void LineDriveService_EmergencyStop(LineDriveService *service);
int16_t LineDriveService_GetForwardAccelerationMmS2(
    const LineDriveService *service);
uint32_t LineDriveService_GetElapsedMs(const LineDriveService *service);

#endif
