#include "balance_drive_run.h"

_Static_assert(
    (int32_t)STEPPER_LEVEL_ANGLE_CDEG + BALL_PD_MIN_ANGLE_CDEG >=
        STEPPER_NORMAL_MIN_ANGLE_CDEG,
    "minimum balance angle must remain inside stepper safety range");
_Static_assert(
    (int32_t)STEPPER_LEVEL_ANGLE_CDEG + BALL_PD_MAX_ANGLE_CDEG <=
        STEPPER_NORMAL_MAX_ANGLE_CDEG,
    "maximum balance angle must remain inside stepper safety range");
_Static_assert(
    ((BALL_PD_ACCEL_FF_NUMERATOR *
      BALANCE_DRIVE_STARTUP_MAX_EXTRA_MM_S2) +
     (BALL_PD_ACCEL_FF_DENOMINATOR / 2)) /
            BALL_PD_ACCEL_FF_DENOMINATOR <=
        BALL_PD_SLEW_LIMIT_CDEG,
    "startup preview increment must fit one PD slew step");

static void BalanceDriveRun_SetFault(
    BalanceDriveRun *run,
    BalanceDriveFault fault)
{
    run->state = BALANCE_DRIVE_FAULT;
    run->fault = fault;
}

static int32_t BalanceDriveRun_ClampInt32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int16_t BalanceDriveRun_PreviewStartupAcceleration(
    BalanceDriveRun *run,
    int16_t forward_acceleration_mm_s2,
    uint32_t now_ms)
{
    int32_t preview_acceleration = forward_acceleration_mm_s2;

    if (run->startup_preview_active != 0U)
    {
        uint32_t elapsed_ms = now_ms - run->previous_acceleration_ms;

        if (forward_acceleration_mm_s2 > 0)
        {
            int32_t acceleration_change =
                (int32_t)forward_acceleration_mm_s2 -
                run->previous_forward_acceleration_mm_s2;

            run->startup_acceleration_seen = 1U;
            if (elapsed_ms != 0U)
            {
                int32_t preview_change = (int32_t)(
                    ((int64_t)acceleration_change *
                     BALANCE_DRIVE_STARTUP_PREVIEW_MS) /
                    elapsed_ms);

                preview_change = BalanceDriveRun_ClampInt32(
                    preview_change,
                    -BALANCE_DRIVE_STARTUP_MAX_EXTRA_MM_S2,
                    BALANCE_DRIVE_STARTUP_MAX_EXTRA_MM_S2);
                preview_acceleration += preview_change;
                preview_acceleration = BalanceDriveRun_ClampInt32(
                    preview_acceleration,
                    0,
                    BALANCE_DRIVE_STARTUP_MAX_ACCEL_MM_S2);
            }
        }
        else if (run->startup_acceleration_seen != 0U)
        {
            run->startup_preview_active = 0U;
        }
    }

    run->previous_acceleration_ms = now_ms;
    run->previous_forward_acceleration_mm_s2 =
        forward_acceleration_mm_s2;
    run->feedforward_acceleration_mm_s2 =
        (int16_t)preview_acceleration;
    return run->feedforward_acceleration_mm_s2;
}

void BalanceDriveRun_Init(BalanceDriveRun *run)
{
    run->state = BALANCE_DRIVE_IDLE;
    run->fault = BALANCE_DRIVE_FAULT_NONE;
    BallPd_Init(&run->controller);
    run->last_control_ms = 0U;
    run->previous_acceleration_ms = 0U;
    run->angle_command_cdeg = 0;
    run->previous_forward_acceleration_mm_s2 = 0;
    run->feedforward_acceleration_mm_s2 = 0;
    run->maximum_positive_x_tenths_mm = 0;
    run->maximum_negative_x_tenths_mm = 0;
    run->vision_sequence_seen = 0U;
    run->last_vision_sequence = 0U;
    run->startup_preview_active = 0U;
    run->startup_acceleration_seen = 0U;
    run->update_count = 0U;
}

uint8_t BalanceDriveRun_Start(
    BalanceDriveRun *run,
    StepperService *stepper,
    const BalanceVisionState *vision,
    uint32_t now_ms)
{
    if (vision == 0 || vision->valid == 0U ||
        StepperService_GetStatus(stepper) !=
            STEPPER_STATUS_READY ||
        !StepperService_BeginAxisAngleTracking(stepper))
    {
        BalanceDriveRun_SetFault(
            run, BALANCE_DRIVE_FAULT_START_NOT_READY);
        return 0U;
    }

    BallPd_Init(&run->controller);
    run->state = BALANCE_DRIVE_ACTIVE;
    run->fault = BALANCE_DRIVE_FAULT_NONE;
    run->last_control_ms = now_ms - BALANCE_DRIVE_CONTROL_PERIOD_MS;
    run->previous_acceleration_ms = now_ms;
    run->angle_command_cdeg = 0;
    run->previous_forward_acceleration_mm_s2 = 0;
    run->feedforward_acceleration_mm_s2 = 0;
    run->maximum_positive_x_tenths_mm = 0;
    run->maximum_negative_x_tenths_mm = 0;
    run->vision_sequence_seen = 0U;
    run->last_vision_sequence = 0U;
    run->startup_preview_active = 1U;
    run->startup_acceleration_seen = 0U;
    run->update_count = 0U;
    return 1U;
}

void BalanceDriveRun_Update(
    BalanceDriveRun *run,
    StepperService *stepper,
    const BalanceVisionState *vision,
    int16_t forward_acceleration_mm_s2,
    uint32_t now_ms)
{
    int16_t target_angle_cdeg;
    int16_t feedforward_acceleration_mm_s2;
    int32_t absolute_angle_cdeg;

    if (run->state != BALANCE_DRIVE_ACTIVE)
    {
        return;
    }
    if (StepperService_GetStatus(stepper) == STEPPER_STATUS_FAULT)
    {
        BalanceDriveRun_SetFault(
            run, BALANCE_DRIVE_FAULT_STEPPER);
        return;
    }
    if (vision == 0 || vision->valid == 0U ||
        vision->age_ms > BALANCE_DRIVE_VISION_TIMEOUT_MS)
    {
        BalanceDriveRun_SetFault(
            run, BALANCE_DRIVE_FAULT_VISION_STALE);
        return;
    }
    if ((uint32_t)(now_ms - run->last_control_ms) <
        BALANCE_DRIVE_CONTROL_PERIOD_MS)
    {
        return;
    }
    if (run->vision_sequence_seen != 0U &&
        vision->sequence == run->last_vision_sequence)
    {
        return;
    }

    if (vision->x_tenths_mm >
        run->maximum_positive_x_tenths_mm)
    {
        run->maximum_positive_x_tenths_mm =
            vision->x_tenths_mm;
    }
    if (vision->x_tenths_mm <
        run->maximum_negative_x_tenths_mm)
    {
        run->maximum_negative_x_tenths_mm =
            vision->x_tenths_mm;
    }

    feedforward_acceleration_mm_s2 =
        BalanceDriveRun_PreviewStartupAcceleration(
            run, forward_acceleration_mm_s2, now_ms);
    target_angle_cdeg = BallPd_Update(
        &run->controller,
        vision->predicted_x_tenths_mm,
        vision->velocity_tenths_mm_s,
        feedforward_acceleration_mm_s2);
    absolute_angle_cdeg =
        (int32_t)STEPPER_LEVEL_ANGLE_CDEG + target_angle_cdeg;
    if (absolute_angle_cdeg < 0 ||
        absolute_angle_cdeg > UINT16_MAX ||
        !StepperService_TrackAbsoluteAxisAngleCdeg(
            stepper,
            (uint16_t)absolute_angle_cdeg,
            BALANCE_DRIVE_TRACK_SPEED_RPM,
            BALANCE_DRIVE_TRACK_ACCELERATION))
    {
        BalanceDriveRun_SetFault(
            run, BALANCE_DRIVE_FAULT_COMMAND_REJECTED);
        return;
    }
    run->angle_command_cdeg = target_angle_cdeg;
    run->vision_sequence_seen = 1U;
    run->last_vision_sequence = vision->sequence;
    run->last_control_ms = now_ms;
    run->update_count++;
}

uint8_t BalanceDriveRun_Stop(
    BalanceDriveRun *run,
    StepperService *stepper)
{
    if (run->state != BALANCE_DRIVE_ACTIVE)
    {
        return 0U;
    }
    if (!StepperService_EndAxisAngleTrackingAtLevel(
            stepper,
            BALANCE_DRIVE_TRACK_SPEED_RPM,
            BALANCE_DRIVE_TRACK_ACCELERATION))
    {
        BalanceDriveRun_SetFault(
            run, BALANCE_DRIVE_FAULT_COMMAND_REJECTED);
        return 0U;
    }
    run->state = BALANCE_DRIVE_STOPPED;
    run->angle_command_cdeg = 0;
    return 1U;
}

void BalanceDriveRun_EmergencyStop(
    BalanceDriveRun *run,
    StepperService *stepper)
{
    (void)StepperService_Stop(stepper);
    (void)StepperService_Disable(stepper);
    BalanceDriveRun_SetFault(run, BALANCE_DRIVE_FAULT_STEPPER);
}

void BalanceDriveRun_ReportVisionFault(BalanceDriveRun *run)
{
    if (run->state == BALANCE_DRIVE_ACTIVE)
    {
        BalanceDriveRun_SetFault(
            run, BALANCE_DRIVE_FAULT_VISION_STALE);
    }
}
