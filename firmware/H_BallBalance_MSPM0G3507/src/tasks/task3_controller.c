#include "task3_controller.h"

static int32_t Task3Controller_AbsInt32(int32_t value)
{
    return value < 0 ? -(value + 1) + 1 : value;
}

static int32_t Task3Controller_ClampInt32(
    int32_t value, int32_t minimum, int32_t maximum)
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

static int16_t Task3Controller_GetPositiveDriveAngle(
    const Task3Controller *controller)
{
    int32_t minimum_angle_cdeg =
        controller->config.minimum_angle_cdeg >
                TASK3_POSITIVE_DRIVE_ANGLE_CDEG
            ? controller->config.minimum_angle_cdeg
            : TASK3_POSITIVE_DRIVE_ANGLE_CDEG;
    int32_t maximum_angle_cdeg =
        controller->config.maximum_angle_cdeg <
                TASK3_POSITIVE_BRAKE_ANGLE_CDEG
            ? controller->config.maximum_angle_cdeg
            : TASK3_POSITIVE_BRAKE_ANGLE_CDEG;

    return (int16_t)Task3Controller_ClampInt32(
        controller->config.positive_drive_angle_cdeg,
        minimum_angle_cdeg,
        maximum_angle_cdeg);
}

static int16_t Task3Controller_ClampPositiveAngle(
    const Task3Controller *controller,
    int16_t angle_cdeg)
{
    int32_t minimum_angle_cdeg =
        controller->config.minimum_angle_cdeg >
                TASK3_POSITIVE_DRIVE_ANGLE_CDEG
            ? controller->config.minimum_angle_cdeg
            : TASK3_POSITIVE_DRIVE_ANGLE_CDEG;
    int32_t maximum_angle_cdeg =
        controller->config.maximum_angle_cdeg <
                TASK3_POSITIVE_BRAKE_ANGLE_CDEG
            ? controller->config.maximum_angle_cdeg
            : TASK3_POSITIVE_BRAKE_ANGLE_CDEG;

    return (int16_t)Task3Controller_ClampInt32(
        angle_cdeg,
        minimum_angle_cdeg,
        maximum_angle_cdeg);
}

static int16_t Task3Controller_GetPositiveBrakeAngle(
    const Task3Controller *controller,
    int32_t velocity_tenths_mm_per_s)
{
    int32_t requested_angle_cdeg =
        velocity_tenths_mm_per_s >=
                TASK3_POSITIVE_HIGH_SPEED_BRAKE_MIN_VELOCITY_TENTHS_MM_PER_S
            ? TASK3_POSITIVE_HIGH_SPEED_BRAKE_ANGLE_CDEG
            : TASK3_POSITIVE_BRAKE_ANGLE_CDEG;

    return (int16_t)Task3Controller_ClampInt32(
        requested_angle_cdeg,
        TASK3_BALANCE_ANGLE_CDEG,
        controller->config.maximum_angle_cdeg);
}

static int32_t Task3Controller_PredictPositiveStopX(
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s,
    int16_t requested_angle_cdeg)
{
    int32_t delay_ms = TASK3_POSITIVE_PREDICTION_DELAY_MS;
    int32_t forward_angle_cdeg = requested_angle_cdeg < 0
        ? -(int32_t)requested_angle_cdeg
        : 0;
    int32_t forward_acceleration_tenths_mm_per_s2 =
        (forward_angle_cdeg *
             TASK3_POSITIVE_ACCEL_PER_DEG_TENTHS_MM_PER_S2 +
         99) /
        100;
    int32_t forward_velocity_tenths_mm_per_s =
        velocity_tenths_mm_per_s > 0
            ? velocity_tenths_mm_per_s
            : 0;
    int32_t delayed_velocity_tenths_mm_per_s =
        forward_velocity_tenths_mm_per_s +
        (forward_acceleration_tenths_mm_per_s2 * delay_ms) /
            1000;
    int64_t delayed_distance_tenths_mm =
        ((int64_t)forward_velocity_tenths_mm_per_s * delay_ms) /
            1000 +
        ((int64_t)forward_acceleration_tenths_mm_per_s2 *
         delay_ms * delay_ms) /
            2000000;
    int64_t braking_distance_tenths_mm =
        ((int64_t)delayed_velocity_tenths_mm_per_s *
         delayed_velocity_tenths_mm_per_s) /
        (2 * TASK3_POSITIVE_MIN_BRAKE_TENTHS_MM_PER_S2);

    return (int32_t)((int64_t)x_tenths_mm +
                     delayed_distance_tenths_mm +
                     braking_distance_tenths_mm);
}

static void Task3Controller_ResetTiming(
    Task3Controller *controller)
{
    controller->timing.positive_reached = false;
    controller->timing.negative_reached = false;
    controller->timing.positive_from_start_ms = 0U;
    controller->timing.negative_from_start_ms = 0U;
    controller->timing.negative_from_reverse_ms = 0U;
    controller->timing.positive_velocity_tenths_mm_per_s = 0;
    controller->timing.negative_velocity_tenths_mm_per_s = 0;
}

static void Task3Controller_ResetPositiveBoost(
    Task3Controller *controller)
{
    controller->positive_low_speed_sample_count = 0U;
    controller->positive_boost_active = false;
}

static void Task3Controller_ResetNegativeFriction(
    Task3Controller *controller)
{
    controller->negative_friction_sample_count = 0U;
    controller->negative_friction_direction = 0;
    controller->negative_friction_active = false;
}

static bool Task3Controller_NeedsPositiveBoost(
    Task3Controller *controller,
    const VisionState *state)
{
    bool before_boost_limit =
        state->x_filtered_tenths_mm <
            TASK3_POSITIVE_BOOST_MAX_X_TENTHS_MM;

    if (controller->positive_boost_active)
    {
        if (!before_boost_limit ||
            (state->velocity_valid &&
             state->velocity_filtered_tenths_mm_per_s >=
                 controller->config.boost_exit_velocity_tenths_mm_per_s))
        {
            Task3Controller_ResetPositiveBoost(controller);
            return false;
        }
        return true;
    }

    if (before_boost_limit && state->velocity_valid &&
        Task3Controller_AbsInt32(
            state->velocity_filtered_tenths_mm_per_s) <=
            controller->config.boost_enter_velocity_tenths_mm_per_s)
    {
        if (controller->positive_low_speed_sample_count <
            TASK3_POSITIVE_BOOST_SAMPLE_COUNT)
        {
            controller->positive_low_speed_sample_count++;
        }
    }
    else
    {
        controller->positive_low_speed_sample_count = 0U;
    }

    if (controller->positive_low_speed_sample_count >=
        TASK3_POSITIVE_BOOST_SAMPLE_COUNT)
    {
        controller->positive_boost_active = true;
        return true;
    }
    return false;
}

static int16_t Task3Controller_ApplyNegativeFriction(
    Task3Controller *controller,
    int16_t target_x_tenths_mm,
    const VisionState *state,
    int16_t base_angle_cdeg)
{
    int32_t lag_tenths_mm =
        (int32_t)state->x_filtered_tenths_mm - target_x_tenths_mm;
    bool final_reference =
        target_x_tenths_mm == TASK3_NEGATIVE_X_TENTHS_MM;
    int8_t requested_direction = 1;
    int32_t enter_max_velocity_tenths_mm_per_s = final_reference
        ? TASK3_NEGATIVE_FINAL_FRICTION_ENTER_MAX_VELOCITY_TENTHS_MM_PER_S
        : TASK3_NEGATIVE_FRICTION_ENTER_MAX_VELOCITY_TENTHS_MM_PER_S;
    uint8_t required_sample_count = final_reference
        ? TASK3_NEGATIVE_FINAL_FRICTION_SAMPLE_COUNT
        : TASK3_NEGATIVE_FRICTION_SAMPLE_COUNT;
    bool can_enter;

    if (final_reference)
    {
        if (state->x_filtered_tenths_mm <
            TASK3_TARGET_MIN_X_TENTHS_MM)
        {
            requested_direction = -1;
        }
        else if (state->x_filtered_tenths_mm <=
                 TASK3_TARGET_MAX_X_TENTHS_MM)
        {
            requested_direction = 0;
        }
        can_enter = requested_direction != 0 &&
            state->velocity_valid &&
            Task3Controller_AbsInt32(
                state->velocity_filtered_tenths_mm_per_s) <=
                enter_max_velocity_tenths_mm_per_s;
    }
    else
    {
        can_enter = state->velocity_valid &&
            state->x_filtered_tenths_mm >
                TASK3_NEGATIVE_FRICTION_STOP_X_TENTHS_MM &&
            lag_tenths_mm >=
                TASK3_NEGATIVE_FRICTION_MIN_LAG_TENTHS_MM &&
            Task3Controller_AbsInt32(
                state->velocity_filtered_tenths_mm_per_s) <=
                enter_max_velocity_tenths_mm_per_s;
    }

    if (controller->negative_friction_active)
    {
        if (!state->velocity_valid ||
            requested_direction !=
                controller->negative_friction_direction ||
            (!final_reference &&
             (state->x_filtered_tenths_mm <=
                  TASK3_NEGATIVE_FRICTION_STOP_X_TENTHS_MM ||
             lag_tenths_mm <=
                 TASK3_NEGATIVE_FRICTION_RELEASE_LAG_TENTHS_MM)) ||
            (int32_t)controller->negative_friction_direction *
                    state->velocity_filtered_tenths_mm_per_s <=
                TASK3_NEGATIVE_FRICTION_EXIT_VELOCITY_TENTHS_MM_PER_S)
        {
            Task3Controller_ResetNegativeFriction(controller);
        }
    }
    else if (can_enter)
    {
        if (controller->negative_friction_direction !=
            requested_direction)
        {
            controller->negative_friction_direction =
                requested_direction;
            controller->negative_friction_sample_count = 0U;
        }
        if (controller->negative_friction_sample_count <
            required_sample_count)
        {
            controller->negative_friction_sample_count++;
        }
        if (controller->negative_friction_sample_count >=
            required_sample_count)
        {
            controller->negative_friction_active = true;
            controller->negative_friction_sample_count = 0U;
        }
    }
    else
    {
        controller->negative_friction_sample_count = 0U;
    }

    if (!controller->negative_friction_active)
    {
        return base_angle_cdeg;
    }
    return (int16_t)Task3Controller_ClampInt32(
        (int32_t)base_angle_cdeg +
            (int32_t)controller->negative_friction_direction *
                TASK3_NEGATIVE_FRICTION_COMPENSATION_CDEG,
        controller->config.minimum_angle_cdeg,
        controller->config.maximum_angle_cdeg);
}

static void Task3Controller_RecordPositiveArrival(
    Task3Controller *controller,
    const VisionSample *sample,
    const VisionState *state)
{
    if (!controller->timing.positive_reached &&
        sample->x_tenths_mm >=
            TASK3_POSITIVE_ARRIVAL_MIN_X_TENTHS_MM &&
        sample->x_tenths_mm <=
            TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM)
    {
        controller->timing.positive_reached = true;
        controller->timing.positive_from_start_ms =
            sample->received_at_ms - controller->start_ms;
        controller->timing.positive_velocity_tenths_mm_per_s =
            state->velocity_valid
                ? state->velocity_filtered_tenths_mm_per_s
                : 0;
    }
}

static bool Task3Controller_IsStable(
    const VisionSample *sample,
    const VisionState *state)
{
    return state->velocity_valid &&
           sample->x_tenths_mm >=
               TASK3_TARGET_MIN_X_TENTHS_MM &&
           sample->x_tenths_mm <=
               TASK3_TARGET_MAX_X_TENTHS_MM &&
           Task3Controller_AbsInt32(
               state->velocity_filtered_tenths_mm_per_s) <=
               TASK3_STABLE_MAX_VELOCITY_TENTHS_MM_PER_S;
}

static void Task3Controller_RecordNegativeArrival(
    Task3Controller *controller,
    const VisionSample *sample,
    const VisionState *state)
{
    if (!controller->timing.negative_reached &&
        Task3Controller_IsStable(sample, state))
    {
        controller->timing.negative_reached = true;
        controller->timing.negative_from_start_ms =
            sample->received_at_ms - controller->start_ms;
        controller->timing.negative_from_reverse_ms =
            sample->received_at_ms - controller->reverse_command_ms;
        controller->timing.negative_velocity_tenths_mm_per_s =
            state->velocity_filtered_tenths_mm_per_s;
    }
}

static Task3ControllerUpdateResult
Task3Controller_UpdateStableCount(
    Task3Controller *controller,
    const VisionSample *sample,
    const VisionState *state)
{
    if (controller->timing.positive_reached &&
        controller->timing.negative_reached &&
        Task3Controller_IsStable(sample, state))
    {
        if (controller->stable_sample_count <
            TASK3_STABLE_SAMPLE_COUNT)
        {
            controller->stable_sample_count++;
        }
        if (controller->stable_sample_count >=
            TASK3_STABLE_SAMPLE_COUNT)
        {
            controller->active = false;
            return TASK3_CONTROLLER_UPDATE_DONE;
        }
    }
    else
    {
        controller->stable_sample_count = 0U;
    }
    return TASK3_CONTROLLER_UPDATE_NONE;
}

static Task3ControllerUpdateResult Task3Controller_CommandLevel(
    Task3Controller *controller,
    uint32_t now_ms,
    int16_t *target_angle_centi_degrees)
{
    controller->phase = TASK3_CONTROLLER_PHASE_LEVEL_TIMEOUT;
    controller->phase_started_ms = now_ms;
    *target_angle_centi_degrees = TASK3_BALANCE_ANGLE_CDEG;
    controller->last_command_angle_cdeg = TASK3_BALANCE_ANGLE_CDEG;
    controller->command_valid = true;
    return TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE;
}

void Task3Controller_GetDefaultConfig(Task3TuningConfig *config)
{
    if (config == 0)
    {
        return;
    }
    config->kp_millidegrees_per_mm = TASK3_PD_KP_NUMERATOR;
    config->kd_millidegrees_seconds_per_mm = TASK3_PD_KD_NUMERATOR;
    config->positive_drive_angle_cdeg =
        TASK3_POSITIVE_DRIVE_ANGLE_CDEG;
    config->positive_approach_x_tenths_mm =
        TASK3_POSITIVE_APPROACH_PREDICTED_X_TENTHS_MM;
    config->boost_enter_velocity_tenths_mm_per_s =
        TASK3_POSITIVE_BOOST_ENTER_VELOCITY_TENTHS_MM_PER_S;
    config->boost_exit_velocity_tenths_mm_per_s =
        TASK3_POSITIVE_BOOST_EXIT_VELOCITY_TENTHS_MM_PER_S;
    config->minimum_angle_cdeg = TASK3_MIN_ANGLE_CDEG;
    config->maximum_angle_cdeg = TASK3_MAX_ANGLE_CDEG;
    config->prediction_delay_ms = TASK3_PREDICTION_DELAY_MS;
    config->stepper_speed_rpm = TASK3_STEPPER_SPEED_RPM;
    config->stepper_acceleration = TASK3_STEPPER_ACCELERATION;
}

static bool Task3Controller_ConfigIsValid(
    const Task3TuningConfig *config)
{
    return config != 0 &&
           config->kp_millidegrees_per_mm >= TASK3_TUNING_KP_MIN &&
           config->kp_millidegrees_per_mm <= TASK3_TUNING_KP_MAX &&
           config->kd_millidegrees_seconds_per_mm >=
               TASK3_TUNING_KD_MIN &&
           config->kd_millidegrees_seconds_per_mm <=
               TASK3_TUNING_KD_MAX &&
           config->positive_drive_angle_cdeg >= TASK3_MIN_ANGLE_CDEG &&
           config->positive_drive_angle_cdeg <= TASK3_BALANCE_ANGLE_CDEG &&
           config->positive_approach_x_tenths_mm >=
               TASK3_TUNING_APPROACH_X_MIN_TENTHS_MM &&
           config->positive_approach_x_tenths_mm <=
               TASK3_TUNING_APPROACH_X_MAX_TENTHS_MM &&
           config->boost_enter_velocity_tenths_mm_per_s >= 0 &&
           config->boost_enter_velocity_tenths_mm_per_s <=
               TASK3_TUNING_BOOST_VELOCITY_MAX_TENTHS_MM_PER_S &&
           config->boost_exit_velocity_tenths_mm_per_s >=
               config->boost_enter_velocity_tenths_mm_per_s &&
           config->boost_exit_velocity_tenths_mm_per_s <=
               TASK3_TUNING_BOOST_VELOCITY_MAX_TENTHS_MM_PER_S &&
           config->minimum_angle_cdeg >= TASK3_MIN_ANGLE_CDEG &&
           config->minimum_angle_cdeg <= TASK3_BALANCE_ANGLE_CDEG &&
           config->maximum_angle_cdeg >= TASK3_BALANCE_ANGLE_CDEG &&
           config->maximum_angle_cdeg <= TASK3_MAX_ANGLE_CDEG &&
           config->minimum_angle_cdeg < config->maximum_angle_cdeg &&
           config->prediction_delay_ms <=
               TASK3_TUNING_PREDICTION_DELAY_MAX_MS &&
           config->stepper_speed_rpm >= 1U &&
           config->stepper_speed_rpm <= TASK3_STEPPER_SPEED_RPM &&
           config->stepper_acceleration >= 1U &&
           config->stepper_acceleration <= TASK3_STEPPER_ACCELERATION;
}

int32_t Task3Controller_PredictXWithConfig(
    const Task3TuningConfig *config,
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s)
{
    return (int32_t)x_tenths_mm +
           (velocity_tenths_mm_per_s *
            (int32_t)config->prediction_delay_ms) /
               1000;
}

int32_t Task3Controller_PredictX(
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s)
{
    Task3TuningConfig config;

    Task3Controller_GetDefaultConfig(&config);
    return Task3Controller_PredictXWithConfig(
        &config, x_tenths_mm, velocity_tenths_mm_per_s);
}

int16_t Task3Controller_ComputeAngleWithConfig(
    const Task3TuningConfig *config,
    int16_t target_x_tenths_mm,
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s)
{
    int32_t predicted_x = Task3Controller_PredictXWithConfig(
        config, x_tenths_mm, velocity_tenths_mm_per_s);
    int32_t error = (int32_t)target_x_tenths_mm - predicted_x;
    int64_t numerator =
        -(int64_t)error * config->kp_millidegrees_per_mm +
        (int64_t)velocity_tenths_mm_per_s *
            config->kd_millidegrees_seconds_per_mm;
    int32_t angle = (int32_t)(
        numerator / TASK3_PD_GAIN_DENOMINATOR);

    angle = Task3Controller_ClampInt32(
        angle, config->minimum_angle_cdeg,
        config->maximum_angle_cdeg);
    return (int16_t)angle;
}

int16_t Task3Controller_ComputeAngleCentiDegrees(
    int16_t target_x_tenths_mm,
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s)
{
    Task3TuningConfig config;

    Task3Controller_GetDefaultConfig(&config);
    return Task3Controller_ComputeAngleWithConfig(
        &config,
        target_x_tenths_mm,
        x_tenths_mm,
        velocity_tenths_mm_per_s);
}

void Task3Controller_ComputeNegativeReference(
    int16_t start_x_tenths_mm,
    uint32_t elapsed_ms,
    Task3NegativeReferenceState *state)
{
    int32_t ramp_up_distance_tenths_mm;
    int32_t cruise_start_x_tenths_mm;
    int32_t cruise_distance_tenths_mm;
    uint32_t cruise_duration_ms;
    uint32_t ramp_down_start_ms;

    if (state == 0)
    {
        return;
    }

    ramp_up_distance_tenths_mm =
        (TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S *
         (int32_t)TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS) /
        2000;
    cruise_start_x_tenths_mm =
        (int32_t)start_x_tenths_mm -
        ramp_up_distance_tenths_mm;
    cruise_distance_tenths_mm =
        cruise_start_x_tenths_mm -
        TASK3_NEGATIVE_REFERENCE_DECEL_START_X_TENTHS_MM;
    if (cruise_distance_tenths_mm < 0)
    {
        cruise_distance_tenths_mm = 0;
    }
    cruise_duration_ms = (uint32_t)(
        ((int64_t)cruise_distance_tenths_mm * 1000 +
         TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S - 1) /
        TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S);
    ramp_down_start_ms =
        TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS +
        cruise_duration_ms;

    if (elapsed_ms < TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS)
    {
        int64_t travel_tenths_mm =
            ((int64_t)TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S *
             elapsed_ms * elapsed_ms) /
            (2LL * TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS * 1000LL);

        state->x_tenths_mm = (int16_t)(
            (int32_t)start_x_tenths_mm - travel_tenths_mm);
        state->velocity_tenths_mm_per_s =
            -((int32_t)TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S *
              (int32_t)elapsed_ms) /
            (int32_t)TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS;
        return;
    }

    if (elapsed_ms < ramp_down_start_ms)
    {
        uint32_t cruise_elapsed_ms =
            elapsed_ms - TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS;
        int32_t cruise_travel_tenths_mm = (int32_t)(
            ((int64_t)TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S *
             cruise_elapsed_ms) /
            1000);

        if (cruise_travel_tenths_mm >
            cruise_distance_tenths_mm)
        {
            cruise_travel_tenths_mm =
                cruise_distance_tenths_mm;
        }
        state->x_tenths_mm = (int16_t)(
            cruise_start_x_tenths_mm -
            cruise_travel_tenths_mm);
        state->velocity_tenths_mm_per_s =
            -TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S;
        return;
    }

    if (elapsed_ms < ramp_down_start_ms +
                         TASK3_NEGATIVE_REFERENCE_RAMP_DOWN_MS)
    {
        uint32_t ramp_down_elapsed_ms =
            elapsed_ms - ramp_down_start_ms;
        int64_t travel_tenths_mm =
            ((int64_t)TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S *
             ramp_down_elapsed_ms *
             (2LL * TASK3_NEGATIVE_REFERENCE_RAMP_DOWN_MS -
              ramp_down_elapsed_ms)) /
            (2LL * TASK3_NEGATIVE_REFERENCE_RAMP_DOWN_MS * 1000LL);

        state->x_tenths_mm = (int16_t)(
            TASK3_NEGATIVE_REFERENCE_DECEL_START_X_TENTHS_MM -
            travel_tenths_mm);
        state->velocity_tenths_mm_per_s =
            -((int32_t)TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S *
              ((int32_t)TASK3_NEGATIVE_REFERENCE_RAMP_DOWN_MS -
               (int32_t)ramp_down_elapsed_ms)) /
            (int32_t)TASK3_NEGATIVE_REFERENCE_RAMP_DOWN_MS;
        return;
    }

    state->x_tenths_mm = TASK3_NEGATIVE_X_TENTHS_MM;
    state->velocity_tenths_mm_per_s = 0;
}

int16_t Task3Controller_ComputeNegativeReferenceX(
    int16_t start_x_tenths_mm, uint32_t elapsed_ms)
{
    Task3NegativeReferenceState state;

    Task3Controller_ComputeNegativeReference(
        start_x_tenths_mm, elapsed_ms, &state);
    return state.x_tenths_mm;
}

void Task3Controller_Init(Task3Controller *controller)
{
    VisionService_Init(&controller->vision);
    Task3Controller_ResetTiming(controller);
    controller->start_ms = 0U;
    controller->reverse_command_ms = 0U;
    controller->phase_started_ms = 0U;
    controller->negative_reference_start_x_tenths_mm =
        TASK3_POSITIVE_X_TENTHS_MM;
    controller->phase = TASK3_CONTROLLER_PHASE_NOT_STARTED;
    controller->stable_sample_count = 0U;
    Task3Controller_ResetPositiveBoost(controller);
    Task3Controller_ResetNegativeFriction(controller);
    controller->last_command_angle_cdeg = TASK3_BALANCE_ANGLE_CDEG;
    controller->command_valid = false;
    controller->active = false;
    Task3Controller_GetDefaultConfig(&controller->config);
}

void Task3Controller_Start(
    Task3Controller *controller, uint32_t now_ms)
{
    VisionService_Init(&controller->vision);
    Task3Controller_ResetTiming(controller);
    controller->start_ms = now_ms;
    controller->reverse_command_ms = 0U;
    controller->phase_started_ms = now_ms;
    controller->negative_reference_start_x_tenths_mm =
        TASK3_POSITIVE_X_TENTHS_MM;
    controller->phase = TASK3_CONTROLLER_PHASE_NOT_STARTED;
    controller->stable_sample_count = 0U;
    Task3Controller_ResetPositiveBoost(controller);
    Task3Controller_ResetNegativeFriction(controller);
    controller->last_command_angle_cdeg = TASK3_BALANCE_ANGLE_CDEG;
    controller->command_valid = false;
    controller->active = true;
}

void Task3Controller_Cancel(Task3Controller *controller)
{
    controller->active = false;
    controller->phase = TASK3_CONTROLLER_PHASE_NOT_STARTED;
    Task3Controller_ResetPositiveBoost(controller);
    Task3Controller_ResetNegativeFriction(controller);
}

Task3ControllerUpdateResult Task3Controller_Update(
    Task3Controller *controller,
    uint32_t now_ms,
    const VisionSample *sample,
    int16_t *target_angle_centi_degrees)
{
    uint32_t elapsed_ms;
    bool accepted_sample = false;
    VisionState state;
    int16_t target_x_tenths_mm;
    int32_t velocity_tenths_mm_per_s;
    int32_t reference_velocity_tenths_mm_per_s = 0;
    bool positive_boost_requested = false;
    Task3TuningConfig phase_config;
    const Task3TuningConfig *phase_config_pointer =
        &controller->config;

    if (!controller->active || target_angle_centi_degrees == 0)
    {
        return TASK3_CONTROLLER_UPDATE_NONE;
    }

    if (controller->phase ==
        TASK3_CONTROLLER_PHASE_LEVEL_TIMEOUT)
    {
        return TASK3_CONTROLLER_UPDATE_NONE;
    }

    elapsed_ms = now_ms - controller->start_ms;
    if (sample != 0 && sample->valid &&
        (int32_t)(sample->received_at_ms -
                  controller->start_ms) >= 0)
    {
        accepted_sample = VisionService_PushSample(
            &controller->vision, sample);
    }
    else if (sample != 0 &&
             (int32_t)(sample->received_at_ms -
                       controller->start_ms) >= 0)
    {
        (void)VisionService_PushSample(
            &controller->vision, sample);
    }

    if (!VisionService_GetState(
            &controller->vision, now_ms, &state))
    {
        if (!controller->vision.state_valid &&
            elapsed_ms < TASK3_VISION_ACQUIRE_TIMEOUT_MS)
        {
            return TASK3_CONTROLLER_UPDATE_NONE;
        }
        return Task3Controller_CommandLevel(
            controller, now_ms,
            target_angle_centi_degrees);
    }

    if (!accepted_sample)
    {
        return TASK3_CONTROLLER_UPDATE_NONE;
    }

    Task3Controller_RecordPositiveArrival(
        controller, sample, &state);
    if (controller->phase !=
            TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK &&
        controller->timing.positive_reached)
    {
        controller->phase =
            TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK;
        controller->phase_started_ms = now_ms;
        controller->reverse_command_ms = now_ms;
        controller->negative_reference_start_x_tenths_mm =
            (int16_t)Task3Controller_ClampInt32(
                state.x_filtered_tenths_mm,
                TASK3_POSITIVE_ARRIVAL_MIN_X_TENTHS_MM,
                TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM);
        controller->stable_sample_count = 0U;
        Task3Controller_ResetPositiveBoost(controller);
        Task3Controller_ResetNegativeFriction(controller);
    }
    else if (controller->phase ==
             TASK3_CONTROLLER_PHASE_NOT_STARTED)
    {
        controller->phase =
            TASK3_CONTROLLER_PHASE_POSITIVE_DRIVE;
        controller->phase_started_ms = now_ms;
        *target_angle_centi_degrees =
            Task3Controller_GetPositiveDriveAngle(controller);
        controller->last_command_angle_cdeg =
            *target_angle_centi_degrees;
        controller->command_valid = true;
        return TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE;
    }
    else if (controller->phase ==
             TASK3_CONTROLLER_PHASE_POSITIVE_DRIVE)
    {
        velocity_tenths_mm_per_s = state.velocity_valid
            ? state.velocity_filtered_tenths_mm_per_s
            : 0;
        if (Task3Controller_PredictXWithConfig(
                &controller->config,
                state.x_filtered_tenths_mm,
                velocity_tenths_mm_per_s) <
            controller->config.positive_approach_x_tenths_mm)
        {
            return TASK3_CONTROLLER_UPDATE_NONE;
        }
        controller->phase =
            TASK3_CONTROLLER_PHASE_POSITIVE_APPROACH;
        controller->phase_started_ms = now_ms;
    }

    if (controller->phase ==
        TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK)
    {
        Task3NegativeReferenceState reference;
        Task3ControllerUpdateResult stable_update;

        Task3Controller_ComputeNegativeReference(
            controller->negative_reference_start_x_tenths_mm,
            now_ms - controller->reverse_command_ms,
            &reference);
        target_x_tenths_mm = reference.x_tenths_mm;
        reference_velocity_tenths_mm_per_s =
            reference.velocity_tenths_mm_per_s;

        Task3Controller_RecordNegativeArrival(
            controller, sample, &state);
        if (target_x_tenths_mm == TASK3_NEGATIVE_X_TENTHS_MM)
        {
            stable_update = Task3Controller_UpdateStableCount(
                controller, sample, &state);
        }
        else
        {
            controller->stable_sample_count = 0U;
            stable_update = TASK3_CONTROLLER_UPDATE_NONE;
        }
        if (stable_update ==
            TASK3_CONTROLLER_UPDATE_DONE)
        {
            return stable_update;
        }
    }
    else
    {
        target_x_tenths_mm =
            TASK3_POSITIVE_CONTROL_X_TENTHS_MM;
        positive_boost_requested =
            controller->phase ==
                TASK3_CONTROLLER_PHASE_POSITIVE_APPROACH &&
            Task3Controller_NeedsPositiveBoost(
                controller, &state);
    }

    velocity_tenths_mm_per_s = state.velocity_valid
        ? state.velocity_filtered_tenths_mm_per_s
        : 0;
    if (controller->phase ==
        TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK)
    {
        velocity_tenths_mm_per_s -=
            reference_velocity_tenths_mm_per_s;
    }
    if (controller->phase ==
        TASK3_CONTROLLER_PHASE_POSITIVE_APPROACH)
    {
        phase_config = controller->config;
        phase_config.kp_millidegrees_per_mm =
            TASK3_POSITIVE_PD_KP_NUMERATOR;
        phase_config.kd_millidegrees_seconds_per_mm =
            TASK3_POSITIVE_PD_KD_NUMERATOR;
        phase_config.prediction_delay_ms =
            TASK3_POSITIVE_PREDICTION_DELAY_MS;
        phase_config_pointer = &phase_config;
    }
    *target_angle_centi_degrees =
        Task3Controller_ComputeAngleWithConfig(
            phase_config_pointer,
            target_x_tenths_mm,
            state.x_filtered_tenths_mm,
            velocity_tenths_mm_per_s);
    if (controller->phase ==
        TASK3_CONTROLLER_PHASE_POSITIVE_APPROACH)
    {
        if (positive_boost_requested)
        {
            *target_angle_centi_degrees =
                Task3Controller_GetPositiveDriveAngle(controller);
        }
        *target_angle_centi_degrees =
            Task3Controller_ClampPositiveAngle(
                controller,
                *target_angle_centi_degrees);
        if (!positive_boost_requested &&
            Task3Controller_PredictPositiveStopX(
                state.x_filtered_tenths_mm,
                velocity_tenths_mm_per_s,
                *target_angle_centi_degrees) >=
            TASK3_POSITIVE_STOP_LIMIT_X_TENTHS_MM)
        {
            *target_angle_centi_degrees =
                Task3Controller_GetPositiveBrakeAngle(
                    controller,
                    velocity_tenths_mm_per_s);
        }
    }
    if (controller->phase ==
        TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK)
    {
        *target_angle_centi_degrees =
            Task3Controller_ApplyNegativeFriction(
                controller,
                target_x_tenths_mm,
                &state,
                *target_angle_centi_degrees);
    }
    controller->last_command_angle_cdeg =
        *target_angle_centi_degrees;
    controller->command_valid = true;
    return TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING;
}

bool Task3Controller_IsActive(
    const Task3Controller *controller)
{
    return controller->active;
}

void Task3Controller_GetTimingResult(
    const Task3Controller *controller,
    Task3ControllerTimingResult *result)
{
    if (result != 0)
    {
        *result = controller->timing;
    }
}

void Task3Controller_GetConfig(
    const Task3Controller *controller, Task3TuningConfig *config)
{
    if (controller != 0 && config != 0)
    {
        *config = controller->config;
    }
}

bool Task3Controller_SetConfig(
    Task3Controller *controller, const Task3TuningConfig *config)
{
    if (controller == 0 || !Task3Controller_ConfigIsValid(config))
    {
        return false;
    }
    controller->config = *config;
    return true;
}

bool Task3Controller_GetVisionState(
    const Task3Controller *controller,
    uint32_t now_ms,
    VisionState *state)
{
    return controller != 0 && state != 0 &&
           VisionService_GetState(&controller->vision, now_ms, state);
}
