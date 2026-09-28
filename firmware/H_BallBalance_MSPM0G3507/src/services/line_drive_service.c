#include "line_drive_service.h"

#include "grayscale_logic.h"

#define CONTROL_TICK_MS 10U
#define PWM_PERIOD_TICKS 1600U
#define MAX_WHEEL_SPEED_MM_S 1000
#define T2_FORWARD_JERK_MM_S3 2000U
#define BALANCE_FORWARD_JERK_MM_S3 800U
#define T2_FORWARD_ACCEL_MM_S2 800U
#define BALANCE_FORWARD_ACCEL_MM_S2 300U
#define T2_CRUISE_SPEED_MM_S 410
#define T2_APPROACH_SPEED_MM_S 100
#define T4_FORWARD_SPEED_MM_S 208
#define T5_T6_FORWARD_SPEED_MM_S 230
#define T2_DECEL_START_TICKS 1350U
#define T2_FINISH_ENABLE_TICKS 1400U
#define T2_TIMEOUT_TICKS 2000U
#define FINISH_ACTIVE_SENSOR_COUNT 4U
#define FINISH_CONFIRM_TICKS 3U
#define LOST_LINE_SPEED_MM_S 120
#define LINE_CENTER_DEADBAND 20
#define T2_HIGH_GAIN_ERROR 60
#define T2_LOW_KP_NUMERATOR 4
#define T2_HIGH_KP_NUMERATOR 9
#define T2_KD_NUMERATOR 2
#define T2_PD_DENOMINATOR 4
#define T2_LOW_DERIVATIVE_LIMIT 20
#define T2_HIGH_DERIVATIVE_LIMIT 40
#define T2_MIN_INNER_WHEEL_SPEED_MM_S 20
#define T2_MAX_DIFFERENTIAL_MM_S 270
#define T2_DIFFERENTIAL_SLEW_STEP_MM_S 40
#define BALANCE_MIN_INNER_WHEEL_SPEED_MM_S 20
#define BALANCE_MAX_DIFFERENTIAL_MM_S 100
#define BALANCE_DIFFERENTIAL_SLEW_STEP_MM_S 20
#define BALANCE_LINE_KP_NUMERATOR 3
#define BALANCE_LINE_KD_NUMERATOR 1
#define BALANCE_PD_DENOMINATOR 4
#define BALANCE_DERIVATIVE_LIMIT 20
#define T2_LOST_LINE_DIFFERENTIAL_MM_S 270
#define BALANCE_LOST_LINE_DIFFERENTIAL_MM_S 120
#define LOST_LINE_STOP_TICKS 100U
#define STEERING_REVERSAL_CONFIRM_TICKS 2U
#define PWM_RAMP_STEP_TICKS 80U
#define DIRECTION_BRAKE_TICKS 3U
#define LEFT_MOTOR_INVERT 0U
#define RIGHT_MOTOR_INVERT 0U
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN 1
#define STALL_MIN_TARGET_MM_S 100
#define STALL_MAX_SPEED_MM_S 30
#define STALL_REQUIRED_TICKS 100U

static int16_t LineDriveService_LimitSymmetric(int32_t value,
                                                int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

static uint16_t LineDriveService_Absolute(int16_t value)
{
    return (uint16_t)(value < 0 ? -value : value);
}

static int16_t LineDriveService_ShapeT2Differential(
    LineDriveService *service,
    int16_t requested)
{
    int16_t maximum_differential;
    int16_t limited_request;
    int16_t change;

    if (service->forward_speed_mm_s <= T2_MIN_INNER_WHEEL_SPEED_MM_S)
    {
        service->applied_differential_mm_s = 0;
        return 0;
    }

    maximum_differential = (int16_t)(
        service->forward_speed_mm_s - T2_MIN_INNER_WHEEL_SPEED_MM_S);
    if (maximum_differential > T2_MAX_DIFFERENTIAL_MM_S)
    {
        maximum_differential = T2_MAX_DIFFERENTIAL_MM_S;
    }
    limited_request = LineDriveService_LimitSymmetric(
        requested, maximum_differential);
    change = LineDriveService_LimitSymmetric(
        (int32_t)limited_request - service->applied_differential_mm_s,
        T2_DIFFERENTIAL_SLEW_STEP_MM_S);
    service->applied_differential_mm_s =
        (int16_t)(service->applied_differential_mm_s + change);
    service->applied_differential_mm_s =
        LineDriveService_LimitSymmetric(
            service->applied_differential_mm_s,
            maximum_differential);
    return service->applied_differential_mm_s;
}

static int16_t LineDriveService_ShapeBalanceDifferential(
    LineDriveService *service,
    int16_t requested)
{
    int16_t maximum_differential;
    int16_t limited_request;
    int16_t change;

    if (service->forward_speed_mm_s <=
        BALANCE_MIN_INNER_WHEEL_SPEED_MM_S)
    {
        service->applied_differential_mm_s = 0;
        return 0;
    }

    maximum_differential = (int16_t)(
        service->forward_speed_mm_s -
        BALANCE_MIN_INNER_WHEEL_SPEED_MM_S);
    if (maximum_differential > BALANCE_MAX_DIFFERENTIAL_MM_S)
    {
        maximum_differential = BALANCE_MAX_DIFFERENTIAL_MM_S;
    }
    limited_request = LineDriveService_LimitSymmetric(
        requested, maximum_differential);
    change = LineDriveService_LimitSymmetric(
        (int32_t)limited_request - service->applied_differential_mm_s,
        BALANCE_DIFFERENTIAL_SLEW_STEP_MM_S);
    service->applied_differential_mm_s =
        (int16_t)(service->applied_differential_mm_s + change);
    service->applied_differential_mm_s =
        LineDriveService_LimitSymmetric(
            service->applied_differential_mm_s,
            maximum_differential);
    return service->applied_differential_mm_s;
}

static void LineDriveService_Brake(LineDriveService *service)
{
    service->requested_forward_speed_mm_s = 0;
    service->forward_speed_mm_s = 0;
    service->applied_differential_mm_s = 0;
    service->target_left_mm_s = 0;
    service->target_right_mm_s = 0;
    service->applied_steering_sign = 0;
    service->pending_steering_sign = 0;
    service->steering_reversal_ticks = 0U;
    DriveMotionProfile_Reset(&service->forward_profile);
    SpeedPid_Reset(&service->left_pid);
    SpeedPid_Reset(&service->right_pid);
    MotorPwmRamp_Reset(&service->left_pwm_ramp);
    MotorPwmRamp_Reset(&service->right_pwm_ramp);
    MotorDirectionGuard_Init(&service->left_direction_guard);
    MotorDirectionGuard_Init(&service->right_direction_guard);
    service->left_output = MotorSafety_MakeOutput(
        0, PWM_PERIOD_TICKS, LEFT_MOTOR_INVERT, 1U);
    service->right_output = MotorSafety_MakeOutput(
        0, PWM_PERIOD_TICKS, RIGHT_MOTOR_INVERT, 1U);
}

static void LineDriveService_Stop(LineDriveService *service,
                                  LineDriveState state,
                                  LineDriveStopReason reason)
{
    service->state = state;
    service->stop_reason = reason;
    LineDriveService_Brake(service);
}

static int16_t LineDriveService_LineDifferential(
    LineDriveService *service,
    uint8_t line_mask)
{
    int16_t error;
    int16_t derivative;
    int16_t derivative_limit;
    int16_t kp_numerator;
    int16_t kd_numerator;
    int16_t denominator;
    int16_t absolute_error;
    int32_t command;
    uint8_t is_t2 = service->mode == LINE_DRIVE_MODE_T2 ? 1U : 0U;

    if (line_mask == 0U)
    {
        return Grayscale_LostLineCommand(
            service->last_nonzero_line_error,
            is_t2 != 0U ? T2_LOST_LINE_DIFFERENTIAL_MM_S :
                          BALANCE_LOST_LINE_DIFFERENTIAL_MM_S);
    }

    error = Grayscale_LineError(line_mask);
    if (error >= -LINE_CENTER_DEADBAND && error <= LINE_CENTER_DEADBAND)
    {
        error = 0;
    }
    absolute_error = error < 0 ? (int16_t)-error : error;
    if (is_t2 != 0U)
    {
        if (absolute_error >= T2_HIGH_GAIN_ERROR)
        {
            kp_numerator = T2_HIGH_KP_NUMERATOR;
            derivative_limit = T2_HIGH_DERIVATIVE_LIMIT;
        }
        else
        {
            kp_numerator = T2_LOW_KP_NUMERATOR;
            derivative_limit = T2_LOW_DERIVATIVE_LIMIT;
        }
        kd_numerator = T2_KD_NUMERATOR;
        denominator = T2_PD_DENOMINATOR;
    }
    else
    {
        kp_numerator = BALANCE_LINE_KP_NUMERATOR;
        kd_numerator = BALANCE_LINE_KD_NUMERATOR;
        denominator = BALANCE_PD_DENOMINATOR;
        derivative_limit = BALANCE_DERIVATIVE_LIMIT;
    }

    derivative = LineDriveService_LimitSymmetric(
        (int32_t)error - service->previous_line_error,
        derivative_limit);
    service->previous_line_error = error;
    if (error != 0)
    {
        service->last_nonzero_line_error = error;
    }
    command = ((int32_t)kp_numerator * error) +
              ((int32_t)kd_numerator * derivative);
    return (int16_t)(command / denominator);
}

static int16_t LineDriveService_FilterSteering(
    LineDriveService *service,
    int16_t requested,
    uint8_t immediate)
{
    int8_t requested_sign =
        requested > 0 ? 1 : (requested < 0 ? -1 : 0);

    if (requested_sign == 0)
    {
        service->pending_steering_sign = 0;
        service->steering_reversal_ticks = 0U;
        return 0;
    }
    if (immediate != 0U || service->applied_steering_sign == 0 ||
        requested_sign == service->applied_steering_sign)
    {
        service->applied_steering_sign = requested_sign;
        service->pending_steering_sign = 0;
        service->steering_reversal_ticks = 0U;
        return requested;
    }
    if (service->pending_steering_sign != requested_sign)
    {
        service->pending_steering_sign = requested_sign;
        service->steering_reversal_ticks = 1U;
        return 0;
    }
    if (service->steering_reversal_ticks <
        STEERING_REVERSAL_CONFIRM_TICKS)
    {
        ++service->steering_reversal_ticks;
    }
    if (service->steering_reversal_ticks <
        STEERING_REVERSAL_CONFIRM_TICKS)
    {
        return 0;
    }
    service->applied_steering_sign = requested_sign;
    service->pending_steering_sign = 0;
    service->steering_reversal_ticks = 0U;
    return requested;
}

static int16_t LineDriveService_RequestedSpeed(
    const LineDriveService *service)
{
    if (service->mode == LINE_DRIVE_MODE_T2)
    {
        return service->elapsed_ticks >= T2_DECEL_START_TICKS ?
            T2_APPROACH_SPEED_MM_S : T2_CRUISE_SPEED_MM_S;
    }
    return service->mode == LINE_DRIVE_MODE_REQUIREMENT_4 ?
        T4_FORWARD_SPEED_MM_S : T5_T6_FORWARD_SPEED_MM_S;
}

static uint16_t LineDriveService_MaximumAcceleration(
    const LineDriveService *service)
{
    return service->mode == LINE_DRIVE_MODE_T2 ?
        T2_FORWARD_ACCEL_MM_S2 : BALANCE_FORWARD_ACCEL_MM_S2;
}

static uint16_t LineDriveService_MaximumJerk(
    const LineDriveService *service)
{
    return service->mode == LINE_DRIVE_MODE_T2 ?
        T2_FORWARD_JERK_MM_S3 : BALANCE_FORWARD_JERK_MM_S3;
}

static void LineDriveService_UpdateTargets(LineDriveService *service,
                                           uint8_t line_mask)
{
    int16_t differential;
    DriveWheelTargets targets;

    service->requested_forward_speed_mm_s =
        line_mask == 0U ? LOST_LINE_SPEED_MM_S :
                          LineDriveService_RequestedSpeed(service);
    differential = LineDriveService_LineDifferential(service, line_mask);
    differential = LineDriveService_FilterSteering(
        service, differential, line_mask == 0U ? 1U : 0U);
    service->forward_speed_mm_s = DriveMotionProfile_Update(
        &service->forward_profile,
        service->requested_forward_speed_mm_s,
        LineDriveService_MaximumAcceleration(service),
        LineDriveService_MaximumJerk(service),
        CONTROL_TICK_MS);
    if (service->mode != LINE_DRIVE_MODE_T2)
    {
        differential = LineDriveService_ShapeBalanceDifferential(
            service, differential);
    }
    else
    {
        differential = LineDriveService_ShapeT2Differential(
            service, differential);
    }
    targets = DriveLogic_MixDifferential(
        service->forward_speed_mm_s,
        differential,
        MAX_WHEEL_SPEED_MM_S);
    service->target_left_mm_s = targets.left_mm_s;
    service->target_right_mm_s = targets.right_mm_s;
}

static void LineDriveService_UpdateOutputs(LineDriveService *service)
{
    int16_t left_pwm = (int16_t)SpeedPid_Update(
        &service->left_pid,
        (float)service->target_left_mm_s,
        (float)service->measured_left_mm_s,
        0.01f);
    int16_t right_pwm = (int16_t)SpeedPid_Update(
        &service->right_pid,
        (float)service->target_right_mm_s,
        (float)service->measured_right_mm_s,
        0.01f);

    left_pwm = MotorDirectionGuard_Apply(
        &service->left_direction_guard,
        left_pwm,
        DIRECTION_BRAKE_TICKS);
    right_pwm = MotorDirectionGuard_Apply(
        &service->right_direction_guard,
        right_pwm,
        DIRECTION_BRAKE_TICKS);

    if (MotorDirectionGuard_IsBraking(
            &service->left_direction_guard) != 0U)
    {
        MotorPwmRamp_Reset(&service->left_pwm_ramp);
        left_pwm = 0;
    }
    else
    {
        left_pwm = MotorPwmRamp_Apply(
            &service->left_pwm_ramp,
            left_pwm,
            PWM_RAMP_STEP_TICKS);
    }
    if (MotorDirectionGuard_IsBraking(
            &service->right_direction_guard) != 0U)
    {
        MotorPwmRamp_Reset(&service->right_pwm_ramp);
        right_pwm = 0;
    }
    else
    {
        right_pwm = MotorPwmRamp_Apply(
            &service->right_pwm_ramp,
            right_pwm,
            PWM_RAMP_STEP_TICKS);
    }

    if (StallMonitor_Update(
            &service->left_stall,
            service->target_left_mm_s,
            service->measured_left_mm_s) != 0U ||
        StallMonitor_Update(
            &service->right_stall,
            service->target_right_mm_s,
            service->measured_right_mm_s) != 0U)
    {
        LineDriveService_Stop(
            service, LINE_DRIVE_FAULT, LINE_DRIVE_STOP_STALL);
        return;
    }

    service->left_output = MotorSafety_MakeOutput(
        left_pwm, PWM_PERIOD_TICKS, LEFT_MOTOR_INVERT, 0U);
    service->right_output = MotorSafety_MakeOutput(
        right_pwm, PWM_PERIOD_TICKS, RIGHT_MOTOR_INVERT, 0U);
}

void LineDriveService_Init(LineDriveService *service)
{
    service->mode = LINE_DRIVE_MODE_T2;
    service->state = LINE_DRIVE_IDLE;
    service->stop_reason = LINE_DRIVE_STOP_NONE;
    service->elapsed_ticks = 0U;
    service->lost_line_ticks = 0U;
    service->finish_confirm_ticks = 0U;
    service->previous_line_error = 0;
    service->last_nonzero_line_error = 0;
    service->previous_left_count = 0;
    service->previous_right_count = 0;
    service->measured_left_mm_s = 0;
    service->measured_right_mm_s = 0;
    service->peak_acceleration_mm_s2 = 0U;
    DriveMotionProfile_Reset(&service->forward_profile);
    SpeedPid_Init(
        &service->left_pid, 0.8f, 2.0f, 0.002f,
        PWM_PERIOD_TICKS, 600.0f);
    SpeedPid_Init(
        &service->right_pid, 0.8f, 2.0f, 0.002f,
        PWM_PERIOD_TICKS, 600.0f);
    StallMonitor_Init(
        &service->left_stall,
        STALL_MIN_TARGET_MM_S,
        STALL_MAX_SPEED_MM_S,
        STALL_REQUIRED_TICKS);
    StallMonitor_Init(
        &service->right_stall,
        STALL_MIN_TARGET_MM_S,
        STALL_MAX_SPEED_MM_S,
        STALL_REQUIRED_TICKS);
    MotorDirectionGuard_Init(&service->left_direction_guard);
    MotorDirectionGuard_Init(&service->right_direction_guard);
    MotorPwmRamp_Init(&service->left_pwm_ramp);
    MotorPwmRamp_Init(&service->right_pwm_ramp);
    LineDriveService_Brake(service);
}

uint8_t LineDriveService_Start(LineDriveService *service,
                               LineDriveMode mode,
                               int32_t left_count,
                               int32_t right_count)
{
    if (mode > LINE_DRIVE_MODE_REQUIREMENT_5)
    {
        return 0U;
    }

    LineDriveService_Brake(service);
    service->mode = mode;
    service->state = LINE_DRIVE_RUNNING;
    service->stop_reason = LINE_DRIVE_STOP_NONE;
    service->elapsed_ticks = 0U;
    service->lost_line_ticks = 0U;
    service->finish_confirm_ticks = 0U;
    service->previous_line_error = 0;
    service->last_nonzero_line_error = 0;
    service->previous_left_count = left_count;
    service->previous_right_count = right_count;
    service->measured_left_mm_s = 0;
    service->measured_right_mm_s = 0;
    service->peak_acceleration_mm_s2 = 0U;
    service->left_stall.bad_ticks = 0U;
    service->right_stall.bad_ticks = 0U;
    return 1U;
}

void LineDriveService_Update(LineDriveService *service,
                             uint8_t line_mask,
                             int32_t left_count,
                             int32_t right_count)
{
    int32_t left_delta = left_count - service->previous_left_count;
    int32_t right_delta = right_count - service->previous_right_count;
    uint16_t acceleration;

    service->previous_left_count = left_count;
    service->previous_right_count = right_count;
    service->measured_left_mm_s =
        (int16_t)DriveLogic_CountsToMmPerSecond(
            LEFT_ENCODER_SIGN * left_delta,
            CONTROL_TICK_MS);
    service->measured_right_mm_s =
        (int16_t)DriveLogic_CountsToMmPerSecond(
            RIGHT_ENCODER_SIGN * right_delta,
            CONTROL_TICK_MS);

    if (service->state != LINE_DRIVE_RUNNING)
    {
        LineDriveService_Brake(service);
        return;
    }

    if (line_mask == 0U)
    {
        if (++service->lost_line_ticks >= LOST_LINE_STOP_TICKS)
        {
            LineDriveService_Stop(
                service, LINE_DRIVE_FAULT,
                LINE_DRIVE_STOP_LINE_LOST);
            return;
        }
    }
    else
    {
        service->lost_line_ticks = 0U;
    }

    ++service->elapsed_ticks;

    /* Only T2 uses the A-line wide-line finish detector. Requirements 4 and
     * 5 have no B marker or B grace period and therefore ignore this input. */
    if (service->mode == LINE_DRIVE_MODE_T2 &&
        service->elapsed_ticks >= T2_FINISH_ENABLE_TICKS &&
        Grayscale_ActiveCount(line_mask) >=
            FINISH_ACTIVE_SENSOR_COUNT)
    {
        if (++service->finish_confirm_ticks >= FINISH_CONFIRM_TICKS)
        {
            LineDriveService_Stop(
                service, LINE_DRIVE_STOPPED,
                LINE_DRIVE_STOP_FINISH_LINE);
            return;
        }
    }
    else
    {
        service->finish_confirm_ticks = 0U;
    }

    if (service->mode == LINE_DRIVE_MODE_T2 &&
        service->elapsed_ticks >= T2_TIMEOUT_TICKS)
    {
        LineDriveService_Stop(
            service, LINE_DRIVE_FAULT,
            LINE_DRIVE_STOP_TIMEOUT);
        return;
    }

    LineDriveService_UpdateTargets(service, line_mask);
    acceleration = LineDriveService_Absolute(
        (int16_t)service->forward_profile.acceleration_mm_s2);
    if (acceleration > service->peak_acceleration_mm_s2)
    {
        service->peak_acceleration_mm_s2 = acceleration;
    }
    LineDriveService_UpdateOutputs(service);
}

void LineDriveService_EmergencyStop(LineDriveService *service)
{
    LineDriveService_Stop(
        service, LINE_DRIVE_FAULT, LINE_DRIVE_STOP_EMERGENCY);
}

int16_t LineDriveService_GetForwardAccelerationMmS2(
    const LineDriveService *service)
{
    return (int16_t)service->forward_profile.acceleration_mm_s2;
}

uint32_t LineDriveService_GetElapsedMs(const LineDriveService *service)
{
    return service->elapsed_ticks * CONTROL_TICK_MS;
}
