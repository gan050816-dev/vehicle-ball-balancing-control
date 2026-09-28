#include "task3_open_loop.h"

#if (TASK3_REVERSE_FRICTION_DIAGNOSTIC != 0U) && \
    (TASK3_REVERSE_FRICTION_DIAGNOSTIC != 1U)
#error "TASK3_REVERSE_FRICTION_DIAGNOSTIC must be 0 or 1"
#endif

#if (TASK3_TIMING_CALIBRATION != 0U) && \
    (TASK3_TIMING_CALIBRATION != 1U)
#error "TASK3_TIMING_CALIBRATION must be 0 or 1"
#endif

#if TASK3_REVERSE_FRICTION_DIAGNOSTIC && \
    TASK3_TIMING_CALIBRATION
#error "Reverse diagnostic and timing calibration cannot both be enabled"
#endif

#if TASK3_REVERSE_FRICTION_DIAGNOSTIC
#define TASK3_FIRST_TARGET_TENTHS_MM \
    TASK3_RACK_TOWARD_HINGE_TENTHS_MM
#define TASK3_REVERSE_TARGET_TENTHS_MM \
    TASK3_RACK_AWAY_FROM_HINGE_TENTHS_MM
#define TASK3_BRAKE_TARGET_TENTHS_MM \
    TASK3_RACK_TOWARD_HINGE_TENTHS_MM
#else
#define TASK3_FIRST_TARGET_TENTHS_MM \
    TASK3_RACK_AWAY_FROM_HINGE_TENTHS_MM
#define TASK3_REVERSE_TARGET_TENTHS_MM \
    TASK3_RACK_TOWARD_HINGE_TENTHS_MM
#define TASK3_BRAKE_TARGET_TENTHS_MM \
    TASK3_RACK_AWAY_FROM_HINGE_TENTHS_MM
#endif

static Task3UpdateResult Task3OpenLoop_CommandPhase(
    Task3OpenLoop *task,
    Task3Phase phase,
    uint16_t target_tenths_mm,
    uint16_t *output_target_tenths_mm)
{
    if (task->phase == phase)
    {
        return TASK3_UPDATE_NONE;
    }

    task->phase = phase;
    *output_target_tenths_mm = target_tenths_mm;
    return TASK3_UPDATE_COMMAND_POSITION;
}

static void Task3OpenLoop_ResetTiming(Task3OpenLoop *task)
{
    task->reverse_command_ms = 0U;
    task->last_sample_received_at_ms = 0U;
    task->last_sample_x_tenths_mm = 0;
    task->latest_velocity_tenths_mm_per_s = 0;
    task->last_sample_sequence = 0U;
    task->brake_nonnegative_sample_count = 0U;
    task->sample_seen = false;
    task->timing.positive_reached = false;
    task->timing.negative_reached = false;
    task->timing.positive_from_start_ms = 0U;
    task->timing.negative_from_start_ms = 0U;
    task->timing.negative_from_reverse_ms = 0U;
    task->timing.positive_velocity_tenths_mm_per_s = 0;
    task->timing.negative_velocity_tenths_mm_per_s = 0;
}

static bool Task3OpenLoop_ConsumeNewSample(
    Task3OpenLoop *task,
    const Task3BallSample *sample)
{
    uint32_t sample_delta_ms;

    if (sample == 0 || !sample->valid ||
        (int32_t)(sample->received_at_ms - task->start_ms) < 0)
    {
        return false;
    }

    if (task->sample_seen &&
        sample->sequence == task->last_sample_sequence)
    {
        return false;
    }

    if (task->sample_seen)
    {
        sample_delta_ms =
            sample->received_at_ms -
            task->last_sample_received_at_ms;
        if (sample_delta_ms != 0U)
        {
            int32_t delta_x =
                (int32_t)sample->x_tenths_mm -
                (int32_t)task->last_sample_x_tenths_mm;

            task->latest_velocity_tenths_mm_per_s =
                (delta_x * 1000) /
                (int32_t)sample_delta_ms;
        }
    }
    else
    {
        task->latest_velocity_tenths_mm_per_s = 0;
    }

    task->sample_seen = true;
    task->last_sample_sequence = sample->sequence;
    task->last_sample_received_at_ms = sample->received_at_ms;
    task->last_sample_x_tenths_mm = sample->x_tenths_mm;
    return true;
}

void Task3OpenLoop_Init(Task3OpenLoop *task)
{
    task->start_ms = 0U;
    task->phase = TASK3_PHASE_NOT_STARTED;
    task->active = false;
    Task3OpenLoop_ResetTiming(task);
}

void Task3OpenLoop_Start(Task3OpenLoop *task, uint32_t now_ms)
{
    task->start_ms = now_ms;
    task->phase = TASK3_PHASE_NOT_STARTED;
    task->active = true;
    Task3OpenLoop_ResetTiming(task);
}

void Task3OpenLoop_Cancel(Task3OpenLoop *task)
{
    task->active = false;
    task->phase = TASK3_PHASE_NOT_STARTED;
}

Task3UpdateResult Task3OpenLoop_Update(
    Task3OpenLoop *task,
    uint32_t now_ms,
    const Task3BallSample *sample,
    uint16_t *target_tenths_mm)
{
    uint32_t elapsed_ms;

    if (!task->active || target_tenths_mm == 0)
    {
        return TASK3_UPDATE_NONE;
    }

    elapsed_ms = now_ms - task->start_ms;

    /*
     * 如果主循环曾长时间停顿而错过轨迹，优先回到水平，不补发已经
     * 过期的倾斜命令。
     */
    if (elapsed_ms >= TASK3_COMPLETE_MS)
    {
        if (task->phase != TASK3_PHASE_LEVEL_HOLD)
        {
            return Task3OpenLoop_CommandPhase(
                task,
                TASK3_PHASE_LEVEL_HOLD,
                TASK3_RACK_LEVEL_TENTHS_MM,
                target_tenths_mm);
        }

#if TASK3_TIMING_CALIBRATION
        if (!task->timing.positive_reached ||
            !task->timing.negative_reached)
        {
            return TASK3_UPDATE_NONE;
        }
#endif
        task->active = false;
        return TASK3_UPDATE_DONE;
    }

#if TASK3_TIMING_CALIBRATION
    if (task->phase == TASK3_PHASE_NOT_STARTED)
    {
        return Task3OpenLoop_CommandPhase(
            task,
            TASK3_PHASE_FIRST_MOTION,
            TASK3_TIMING_DRIVE_TARGET_TENTHS_MM,
            target_tenths_mm);
    }

    if (Task3OpenLoop_ConsumeNewSample(task, sample))
    {
        if (task->phase == TASK3_PHASE_FIRST_MOTION &&
            sample->x_tenths_mm >=
                TASK3_TIMING_POSITIVE_X_TENTHS_MM)
        {
            task->timing.positive_reached = true;
            task->timing.positive_from_start_ms =
                sample->received_at_ms - task->start_ms;
            task->timing.positive_velocity_tenths_mm_per_s =
                task->latest_velocity_tenths_mm_per_s;
            task->reverse_command_ms = now_ms;
            return Task3OpenLoop_CommandPhase(
                task,
                TASK3_PHASE_REVERSE_MOTION,
                TASK3_RACK_TOWARD_HINGE_TENTHS_MM,
                target_tenths_mm);
        }

        if (task->phase == TASK3_PHASE_REVERSE_MOTION &&
            sample->x_tenths_mm <=
                TASK3_TIMING_BRAKE_TRIGGER_X_TENTHS_MM &&
            sample->x_tenths_mm >
                TASK3_TIMING_NEGATIVE_X_TENTHS_MM &&
            task->latest_velocity_tenths_mm_per_s <=
                TASK3_TIMING_BRAKE_TRIGGER_VELOCITY_TENTHS_MM_PER_S)
        {
            task->brake_nonnegative_sample_count = 0U;
            return Task3OpenLoop_CommandPhase(
                task,
                TASK3_PHASE_BRAKE_MOTION,
                TASK3_TIMING_BRAKE_TARGET_TENTHS_MM,
                target_tenths_mm);
        }

        if ((task->phase == TASK3_PHASE_REVERSE_MOTION ||
             task->phase == TASK3_PHASE_BRAKE_MOTION) &&
            sample->x_tenths_mm <=
                TASK3_TIMING_NEGATIVE_X_TENTHS_MM)
        {
            task->timing.negative_reached = true;
            task->timing.negative_from_start_ms =
                sample->received_at_ms - task->start_ms;
            task->timing.negative_from_reverse_ms =
                sample->received_at_ms -
                task->reverse_command_ms;
            task->timing.negative_velocity_tenths_mm_per_s =
                task->latest_velocity_tenths_mm_per_s;
            return Task3OpenLoop_CommandPhase(
                task,
                TASK3_PHASE_LEVEL_HOLD,
                TASK3_RACK_LEVEL_TENTHS_MM,
                target_tenths_mm);
        }

        if (task->phase == TASK3_PHASE_BRAKE_MOTION)
        {
            if (task->latest_velocity_tenths_mm_per_s >= 0)
            {
                if (task->brake_nonnegative_sample_count <
                    TASK3_TIMING_BRAKE_RELEASE_SAMPLES)
                {
                    task->brake_nonnegative_sample_count++;
                }

                if (task->brake_nonnegative_sample_count >=
                    TASK3_TIMING_BRAKE_RELEASE_SAMPLES)
                {
                    return Task3OpenLoop_CommandPhase(
                        task,
                        TASK3_PHASE_LEVEL_HOLD,
                        TASK3_RACK_LEVEL_TENTHS_MM,
                        target_tenths_mm);
                }
            }
            else
            {
                task->brake_nonnegative_sample_count = 0U;
            }
        }
    }

    return TASK3_UPDATE_NONE;
#else
    (void)sample;
    if (elapsed_ms < TASK3_FIRST_REVERSE_MS)
    {
        return Task3OpenLoop_CommandPhase(
            task,
            TASK3_PHASE_FIRST_MOTION,
            TASK3_FIRST_TARGET_TENTHS_MM,
            target_tenths_mm);
    }

    if (elapsed_ms < TASK3_SECOND_REVERSE_MS)
    {
        return Task3OpenLoop_CommandPhase(
            task,
            TASK3_PHASE_REVERSE_MOTION,
            TASK3_REVERSE_TARGET_TENTHS_MM,
            target_tenths_mm);
    }

    if (elapsed_ms < TASK3_RETURN_LEVEL_MS)
    {
        return Task3OpenLoop_CommandPhase(
            task,
            TASK3_PHASE_BRAKE_MOTION,
            TASK3_BRAKE_TARGET_TENTHS_MM,
            target_tenths_mm);
    }

    return Task3OpenLoop_CommandPhase(
        task,
        TASK3_PHASE_LEVEL_HOLD,
        TASK3_RACK_LEVEL_TENTHS_MM,
        target_tenths_mm);
#endif
}

bool Task3OpenLoop_IsActive(const Task3OpenLoop *task)
{
    return task->active;
}

void Task3OpenLoop_GetTimingResult(
    const Task3OpenLoop *task,
    Task3TimingResult *result)
{
    if (result != 0)
    {
        *result = task->timing;
    }
}
