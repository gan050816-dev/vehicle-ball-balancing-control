#include "task4_run.h"

_Static_assert(
    (int32_t)STEPPER_LEVEL_ANGLE_CDEG + BALL_PD_MIN_ANGLE_CDEG >=
        STEPPER_NORMAL_MIN_ANGLE_CDEG,
    "minimum T4 angle must remain inside the stepper safety range");
_Static_assert(
    (int32_t)STEPPER_LEVEL_ANGLE_CDEG + BALL_PD_MAX_ANGLE_CDEG <=
        STEPPER_NORMAL_MAX_ANGLE_CDEG,
    "maximum T4 angle must remain inside the stepper safety range");

static void SetFault(Task4Run *task, Task4Fault fault)
{
    task->state = TASK4_RUN_FAULT;
    task->fault = fault;
}

void Task4Run_Init(Task4Run *task)
{
    task->state = TASK4_RUN_IDLE;
    task->fault = TASK4_FAULT_NONE;
    BallPd_Init(&task->controller);
    task->last_control_ms = 0U;
    task->angle_command_cdeg = 0;
    task->vision_sequence_seen = 0U;
    task->last_vision_sequence = 0U;
    task->update_count = 0U;
}

uint8_t Task4Run_Start(Task4Run *task,
                       StepperService *stepper,
                       const VisionState *vision,
                       uint32_t now_ms)
{
    if (vision == 0 || vision->valid == 0U ||
        StepperService_GetStatus(stepper) !=
            STEPPER_STATUS_READY ||
        !StepperService_BeginTracking(stepper))
    {
        SetFault(task, TASK4_FAULT_START_NOT_READY);
        return 0U;
    }

    BallPd_Init(&task->controller);
    task->state = TASK4_RUN_ACTIVE;
    task->fault = TASK4_FAULT_NONE;
    task->last_control_ms = now_ms - TASK4_CONTROL_PERIOD_MS;
    task->angle_command_cdeg = 0;
    task->vision_sequence_seen = 0U;
    task->last_vision_sequence = 0U;
    task->update_count = 0U;
    return 1U;
}

void Task4Run_Update(Task4Run *task,
                     StepperService *stepper,
                     const VisionState *vision,
                     int16_t forward_acceleration_mm_s2,
                     uint32_t now_ms)
{
    int16_t target_angle_cdeg;
    int32_t absolute_angle_cdeg;

    if (task->state != TASK4_RUN_ACTIVE)
    {
        return;
    }
    if (StepperService_GetStatus(stepper) ==
        STEPPER_STATUS_FAULT)
    {
        SetFault(task, TASK4_FAULT_STEPPER);
        return;
    }
    if (vision == 0 || vision->valid == 0U ||
        vision->age_ms > TASK4_VISION_TIMEOUT_MS)
    {
        SetFault(task, TASK4_FAULT_VISION_STALE);
        return;
    }
    if ((uint32_t)(now_ms - task->last_control_ms) <
        TASK4_CONTROL_PERIOD_MS)
    {
        return;
    }
    if (task->vision_sequence_seen != 0U &&
        vision->sequence == task->last_vision_sequence)
    {
        return;
    }

    target_angle_cdeg = BallPd_Update(
        &task->controller,
        vision->predicted_x_tenths_mm,
        vision->velocity_tenths_mm_s,
        forward_acceleration_mm_s2);
    absolute_angle_cdeg =
        (int32_t)STEPPER_LEVEL_ANGLE_CDEG + target_angle_cdeg;
    if (absolute_angle_cdeg < 0 || absolute_angle_cdeg > UINT16_MAX ||
        !StepperService_TrackAbsoluteAngleCdeg(
            stepper, (uint16_t)absolute_angle_cdeg,
            TASK4_TRACK_SPEED_RPM,
            TASK4_TRACK_ACCELERATION))
    {
        SetFault(task, TASK4_FAULT_COMMAND_REJECTED);
        return;
    }
    task->angle_command_cdeg = target_angle_cdeg;
    task->vision_sequence_seen = 1U;
    task->last_vision_sequence = vision->sequence;
    task->last_control_ms = now_ms;
    task->update_count++;
}

uint8_t Task4Run_Stop(Task4Run *task,
                      StepperService *stepper)
{
    if (task->state != TASK4_RUN_ACTIVE)
    {
        return 0U;
    }
    if (!StepperService_EndTrackingAtLevel(
            stepper, TASK4_TRACK_SPEED_RPM,
            TASK4_TRACK_ACCELERATION))
    {
        SetFault(task, TASK4_FAULT_COMMAND_REJECTED);
        return 0U;
    }
    task->state = TASK4_RUN_STOPPED;
    task->angle_command_cdeg = 0;
    return 1U;
}

void Task4Run_EmergencyStop(Task4Run *task,
                            StepperService *stepper)
{
    (void)StepperService_Stop(stepper);
    (void)StepperService_Disable(stepper);
    SetFault(task, TASK4_FAULT_STEPPER);
}
