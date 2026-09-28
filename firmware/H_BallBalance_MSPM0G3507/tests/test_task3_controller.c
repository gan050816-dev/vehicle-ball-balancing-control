#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "task3_controller.h"

static VisionSample make_sample(
    uint8_t sequence,
    int16_t x_tenths_mm,
    uint32_t received_at_ms)
{
    VisionSample sample;

    sample.valid = true;
    sample.sequence = sequence;
    sample.x_tenths_mm = x_tenths_mm;
    sample.received_at_ms = received_at_ms;
    return sample;
}

static Task3ControllerUpdateResult update_sample(
    Task3Controller *controller,
    uint8_t sequence,
    int16_t x_tenths_mm,
    uint32_t received_at_ms,
    int16_t *target_angle_centi_degrees)
{
    VisionSample sample =
        make_sample(sequence, x_tenths_mm, received_at_ms);

    return Task3Controller_Update(
        controller,
        received_at_ms,
        &sample,
        target_angle_centi_degrees);
}

static void start_positive_drive(
    Task3Controller *controller,
    int16_t *target_angle_centi_degrees)
{
    Task3Controller_Init(controller);
    Task3Controller_Start(controller, 0U);
    assert(Task3Controller_Update(
               controller,
               0U,
               0,
               target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_NONE);
    assert(update_sample(
               controller,
               1U,
               0,
               40U,
               target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE);
    assert(*target_angle_centi_degrees ==
           TASK3_POSITIVE_DRIVE_ANGLE_CDEG);
    assert(controller->phase ==
           TASK3_CONTROLLER_PHASE_POSITIVE_DRIVE);
}

static void enter_positive_approach(
    Task3Controller *controller,
    int16_t *target_angle_centi_degrees)
{
    start_positive_drive(
        controller, target_angle_centi_degrees);
    assert(update_sample(
               controller,
               2U,
               80,
               80U,
               target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_NONE);
    assert(update_sample(
               controller,
               3U,
               160,
               120U,
               target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(controller->phase ==
           TASK3_CONTROLLER_PHASE_POSITIVE_APPROACH);
}

static void start_and_reach_positive(
    Task3Controller *controller,
    int16_t *target_angle_centi_degrees)
{
    uint8_t sequence;
    uint32_t now_ms;

    enter_positive_approach(
        controller, target_angle_centi_degrees);
    assert(update_sample(
               controller,
               4U,
               320,
               160U,
               target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(update_sample(
               controller,
               5U,
               500,
               200U,
               target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(!controller->timing.positive_reached);
    assert(controller->phase ==
           TASK3_CONTROLLER_PHASE_POSITIVE_APPROACH);
    sequence = 6U;
    now_ms = 240U;
    while (!controller->timing.positive_reached &&
           sequence <= 11U)
    {
        assert(update_sample(
                   controller,
                   sequence++,
                   500,
                   now_ms,
                   target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(controller->timing.positive_reached);
    assert(controller->phase ==
           TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK);
    assert(*target_angle_centi_degrees > 0);
    assert(*target_angle_centi_degrees <=
           TASK3_MAX_ANGLE_CDEG);
}

static void test_control_parameters(void)
{
    assert(TASK3_BALANCE_ANGLE_CDEG == 0);
    assert(TASK3_MIN_ANGLE_CDEG == -2865);
    assert(TASK3_MAX_ANGLE_CDEG == 1432);
    assert(TASK3_PD_KP_NUMERATOR == 250);
    assert(TASK3_PD_KD_NUMERATOR == 200);
    assert(TASK3_POSITIVE_PD_KP_NUMERATOR == 250);
    assert(TASK3_POSITIVE_PD_KD_NUMERATOR == 500);
    assert(TASK3_POSITIVE_DRIVE_ANGLE_CDEG == -2050);
    assert(TASK3_POSITIVE_BRAKE_ANGLE_CDEG == 400);
    assert(TASK3_POSITIVE_HIGH_SPEED_BRAKE_ANGLE_CDEG == 600);
    assert(TASK3_POSITIVE_HIGH_SPEED_BRAKE_MIN_VELOCITY_TENTHS_MM_PER_S ==
           400);
    assert(TASK3_POSITIVE_CONTROL_X_TENTHS_MM == 490);
    assert(TASK3_POSITIVE_STOP_LIMIT_X_TENTHS_MM == 480);
    assert(TASK3_POSITIVE_MIN_BRAKE_TENTHS_MM_PER_S2 == 500);
    assert(TASK3_POSITIVE_PREDICTION_DELAY_MS == 160U);
    assert(TASK3_POSITIVE_APPROACH_PREDICTED_X_TENTHS_MM ==
           100);
    assert(TASK3_POSITIVE_BOOST_ENTER_VELOCITY_TENTHS_MM_PER_S ==
           50);
    assert(TASK3_POSITIVE_BOOST_EXIT_VELOCITY_TENTHS_MM_PER_S ==
           100);
    assert(TASK3_POSITIVE_BOOST_SAMPLE_COUNT == 3U);
    assert(TASK3_POSITIVE_BOOST_MAX_X_TENTHS_MM == 480);
    assert(TASK3_POSITIVE_ARRIVAL_MIN_X_TENTHS_MM == 465);
    assert(TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM == 520);
    assert(TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S == 600);
    assert(TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS == 300U);
    assert(TASK3_NEGATIVE_REFERENCE_DECEL_START_X_TENTHS_MM == -350);
    assert(TASK3_NEGATIVE_REFERENCE_RAMP_DOWN_MS == 500U);
    assert(TASK3_NEGATIVE_FRICTION_COMPENSATION_CDEG == 150);
    assert(TASK3_NEGATIVE_FRICTION_MIN_LAG_TENTHS_MM == 50);
    assert(TASK3_NEGATIVE_FRICTION_RELEASE_LAG_TENTHS_MM == 20);
    assert(TASK3_NEGATIVE_FRICTION_STOP_X_TENTHS_MM == -490);
    assert(TASK3_NEGATIVE_FRICTION_ENTER_MAX_VELOCITY_TENTHS_MM_PER_S ==
           100);
    assert(TASK3_NEGATIVE_FINAL_FRICTION_ENTER_MAX_VELOCITY_TENTHS_MM_PER_S ==
           150);
    assert(TASK3_NEGATIVE_FRICTION_EXIT_VELOCITY_TENTHS_MM_PER_S ==
           -250);
    assert(TASK3_NEGATIVE_FRICTION_SAMPLE_COUNT == 3U);
    assert(TASK3_NEGATIVE_FINAL_FRICTION_SAMPLE_COUNT == 2U);
    assert(TASK3_TARGET_MIN_X_TENTHS_MM == -520);
    assert(TASK3_TARGET_MAX_X_TENTHS_MM == -470);
    assert(TASK3_STABLE_MAX_VELOCITY_TENTHS_MM_PER_S == 200);
    assert(TASK3_PD_KP_NUMERATOR == 250);
    assert(TASK3_PD_KD_NUMERATOR == 200);
    assert(TASK3_PD_GAIN_DENOMINATOR == 100);
    assert(TASK3_PREDICTION_DELAY_MS == 120U);
    assert(Task3Controller_PredictX(300, 700) == 384);
    assert(Task3Controller_ComputeAngleCentiDegrees(
               500, 0, 0) == -1250);
    assert(Task3Controller_ComputeAngleCentiDegrees(
               500, 300, 700) == 1110);
    assert(Task3Controller_ComputeAngleCentiDegrees(
               500, 220, 700) == 910);
    assert(Task3Controller_ComputeAngleCentiDegrees(
               -500, 500, 700) == 1432);
    assert(Task3Controller_ComputeAngleCentiDegrees(
               -500, 200, -2000) == -2850);
    assert(Task3Controller_ComputeAngleCentiDegrees(
               -500, 0, -2000) == TASK3_MIN_ANGLE_CDEG);
    assert(Task3Controller_ComputeAngleCentiDegrees(
               -500, -450, 0) == 125);
    assert(Task3Controller_ComputeNegativeReferenceX(500, 0U) == 500);
    assert(Task3Controller_ComputeNegativeReferenceX(500, 300U) == 410);
    assert(Task3Controller_ComputeNegativeReferenceX(500, 1567U) ==
           -350);
    assert(Task3Controller_ComputeNegativeReferenceX(500, 1817U) ==
           -462);
    assert(Task3Controller_ComputeNegativeReferenceX(500, 2067U) ==
           -500);
    assert(Task3Controller_ComputeNegativeReferenceX(
               500, UINT32_MAX) == -500);
}

static void test_negative_reference_position_and_velocity(void)
{
    Task3NegativeReferenceState state;
    int16_t previous_x = TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM;
    uint32_t elapsed_ms;

    Task3Controller_ComputeNegativeReference(500, 0U, &state);
    assert(state.x_tenths_mm == 500);
    assert(state.velocity_tenths_mm_per_s == 0);
    Task3Controller_ComputeNegativeReference(500, 40U, &state);
    assert(state.x_tenths_mm == 499);
    assert(state.velocity_tenths_mm_per_s == -80);
    Task3Controller_ComputeNegativeReference(500, 300U, &state);
    assert(state.x_tenths_mm == 410);
    assert(state.velocity_tenths_mm_per_s == -600);
    Task3Controller_ComputeNegativeReference(500, 1567U, &state);
    assert(state.x_tenths_mm == -350);
    assert(state.velocity_tenths_mm_per_s == -600);
    Task3Controller_ComputeNegativeReference(500, 1817U, &state);
    assert(state.x_tenths_mm == -462);
    assert(state.velocity_tenths_mm_per_s == -300);
    Task3Controller_ComputeNegativeReference(500, 2067U, &state);
    assert(state.x_tenths_mm == -500);
    assert(state.velocity_tenths_mm_per_s == 0);

    for (elapsed_ms = 0U; elapsed_ms <= 2234U; ++elapsed_ms)
    {
        Task3Controller_ComputeNegativeReference(
            TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM,
            elapsed_ms,
            &state);
        assert(state.x_tenths_mm <= previous_x);
        assert(state.x_tenths_mm >= TASK3_NEGATIVE_X_TENTHS_MM);
        assert(state.velocity_tenths_mm_per_s <= 0);
        assert(state.velocity_tenths_mm_per_s >=
               -TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S);
        previous_x = state.x_tenths_mm;
    }
    Task3Controller_ComputeNegativeReference(
        TASK3_POSITIVE_ARRIVAL_MIN_X_TENTHS_MM, UINT32_MAX, &state);
    assert(state.x_tenths_mm == TASK3_NEGATIVE_X_TENTHS_MM);
    assert(state.velocity_tenths_mm_per_s == 0);
    Task3Controller_ComputeNegativeReference(
        TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM, UINT32_MAX, &state);
    assert(state.x_tenths_mm == TASK3_NEGATIVE_X_TENTHS_MM);
    assert(state.velocity_tenths_mm_per_s == 0);
}

static void test_runtime_tuning_config(void)
{
    Task3Controller controller;
    Task3TuningConfig config;
    int16_t default_angle;
    int16_t tuned_angle;

    Task3Controller_Init(&controller);
    Task3Controller_GetConfig(&controller, &config);
    assert(config.kp_millidegrees_per_mm == TASK3_PD_KP_NUMERATOR);
    assert(config.kd_millidegrees_seconds_per_mm ==
           TASK3_PD_KD_NUMERATOR);
    default_angle = Task3Controller_ComputeAngleWithConfig(
        &config, 500, 0, 0);
    config.kp_millidegrees_per_mm = 200;
    config.kd_millidegrees_seconds_per_mm = 80;
    config.prediction_delay_ms = 100U;
    assert(Task3Controller_SetConfig(&controller, &config));
    Task3Controller_GetConfig(&controller, &config);
    tuned_angle = Task3Controller_ComputeAngleWithConfig(
        &config, 500, 0, 0);
    assert(tuned_angle != default_angle);
    assert(tuned_angle == -1000);

    config.maximum_angle_cdeg = TASK3_MAX_ANGLE_CDEG + 1;
    assert(!Task3Controller_SetConfig(&controller, &config));
    Task3Controller_GetConfig(&controller, &config);
    assert(config.maximum_angle_cdeg == TASK3_MAX_ANGLE_CDEG);
}

static void test_soft_angle_limit_also_caps_fixed_drive(void)
{
    Task3Controller controller;
    Task3TuningConfig config;
    int16_t target_angle_centi_degrees = 0;

    Task3Controller_Init(&controller);
    Task3Controller_GetConfig(&controller, &config);
    config.minimum_angle_cdeg = -100;
    assert(Task3Controller_SetConfig(&controller, &config));
    Task3Controller_Start(&controller, 0U);
    assert(update_sample(
               &controller, 1U, 0, 40U,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE);
    assert(target_angle_centi_degrees == -100);
}

static void test_waits_for_first_vision_sample(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 123;

    Task3Controller_Init(&controller);
    Task3Controller_Start(&controller, 0U);
    assert(Task3Controller_Update(
               &controller,
               499U,
               0,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_NONE);
    assert(target_angle_centi_degrees == 123);
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_NOT_STARTED);
    assert(Task3Controller_IsActive(&controller));
}

static void test_positive_drive_is_held_until_predicted_threshold(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;

    start_positive_drive(
        &controller, &target_angle_centi_degrees);
    target_angle_centi_degrees = 321;
    assert(update_sample(
               &controller,
               2U,
               80,
               80U,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_NONE);
    assert(target_angle_centi_degrees == 321);
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_POSITIVE_DRIVE);
}

static void test_prediction_starts_pd_before_raw_thirty_mm(void)
{
    Task3Controller controller;
    Task3TuningConfig positive_config;
    int16_t target_angle_centi_degrees = 0;

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    assert(controller.vision.x_filtered_tenths_mm == 100);
    assert(controller.vision.velocity_filtered_tenths_mm_per_s ==
           2000);
    Task3Controller_GetDefaultConfig(&positive_config);
    positive_config.prediction_delay_ms =
        TASK3_POSITIVE_PREDICTION_DELAY_MS;
    assert(Task3Controller_PredictXWithConfig(
               &positive_config,
               controller.vision.x_filtered_tenths_mm,
               controller.vision.velocity_filtered_tenths_mm_per_s) ==
           420);
    assert(target_angle_centi_degrees !=
           TASK3_POSITIVE_DRIVE_ANGLE_CDEG);
}

static void test_low_speed_approach_reenters_drive_and_recovers(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence = 4U;
    uint32_t now_ms = 160U;
    uint8_t index;

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    for (index = 0U;
         index < 20U && !controller.positive_boost_active;
         ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   160,
                   now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(controller.positive_boost_active);
    assert(target_angle_centi_degrees ==
           TASK3_POSITIVE_DRIVE_ANGLE_CDEG);

    assert(update_sample(
               &controller,
               sequence,
               200,
               now_ms,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(!controller.positive_boost_active);
    assert(target_angle_centi_degrees !=
           TASK3_POSITIVE_DRIVE_ANGLE_CDEG);
}

static void test_low_speed_boost_is_not_overridden_by_stop_prediction(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence = 4U;
    uint32_t now_ms = 160U;
    uint8_t index;

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    for (index = 0U;
         index < 20U && !controller.positive_boost_active;
         ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   340,
                   now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(controller.positive_boost_active);
    assert(controller.vision.x_filtered_tenths_mm <
           TASK3_POSITIVE_BOOST_MAX_X_TENTHS_MM);
    assert(target_angle_centi_degrees ==
           TASK3_POSITIVE_DRIVE_ANGLE_CDEG);
}

static void test_high_speed_approach_uses_stronger_brake(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    assert(update_sample(
               &controller,
               4U,
               320,
               160U,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(controller.vision.velocity_filtered_tenths_mm_per_s >=
           TASK3_POSITIVE_HIGH_SPEED_BRAKE_MIN_VELOCITY_TENTHS_MM_PER_S);
    assert(target_angle_centi_degrees ==
           TASK3_POSITIVE_HIGH_SPEED_BRAKE_ANGLE_CDEG);
}

static void test_positive_boost_stays_off_after_target_overshoot(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence = 4U;
    uint32_t now_ms = 160U;
    uint8_t index;

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    for (index = 0U; index < 16U; ++index)
    {
        Task3ControllerUpdateResult result = update_sample(
            &controller,
            sequence++,
            900,
            now_ms,
            &target_angle_centi_degrees);

        assert(result != TASK3_CONTROLLER_UPDATE_DONE);
        assert(!controller.positive_boost_active);
        now_ms += 40U;
    }
    assert(controller.vision.x_filtered_tenths_mm >=
           TASK3_POSITIVE_BOOST_MAX_X_TENTHS_MM);
    assert(!controller.positive_boost_active);
    assert(target_angle_centi_degrees !=
           TASK3_POSITIVE_DRIVE_ANGLE_CDEG);
}

static void test_positive_arrival_accepts_high_speed(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    assert(update_sample(
               &controller, 4U, 320, 160U,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(update_sample(
               &controller, 5U, 500, 200U,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(controller.timing.positive_reached);
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK);
    assert(controller.reverse_command_ms == 200U);
}

static void test_positive_arrival_requires_46_5_to_52_mm(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence;
    uint32_t now_ms;

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    assert(update_sample(
               &controller, 4U, 464, 160U,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(!controller.timing.positive_reached);
    sequence = 5U;
    now_ms = 200U;
    while (sequence <= 10U)
    {
        assert(update_sample(
                   &controller, sequence++, 464, now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(!controller.timing.positive_reached);
    assert(update_sample(
               &controller, sequence, 465, now_ms,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(controller.timing.positive_reached);
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK);
    assert(controller.negative_reference_start_x_tenths_mm >=
           TASK3_POSITIVE_ARRIVAL_MIN_X_TENTHS_MM);
    assert(controller.negative_reference_start_x_tenths_mm <=
           TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM);

    enter_positive_approach(
        &controller, &target_angle_centi_degrees);
    assert(update_sample(
               &controller, 4U, 521, 160U,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(!controller.timing.positive_reached);
    sequence = 6U;
    now_ms = 240U;
    while (sequence <= 10U)
    {
        assert(update_sample(
                   &controller, sequence++, 521, now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(!controller.timing.positive_reached);
    assert(update_sample(
               &controller, sequence, 520, now_ms,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    assert(controller.timing.positive_reached);
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK);
}

static void test_negative_pd_uses_reference_relative_velocity(void)
{
    Task3Controller controller;
    Task3TuningConfig config;
    Task3NegativeReferenceState reference;
    VisionState state;
    int16_t target_angle_centi_degrees = 0;
    int16_t relative_velocity_angle;
    int16_t absolute_velocity_angle;
    uint8_t sequence;
    uint32_t now_ms;
    uint32_t final_sample_ms;

    start_and_reach_positive(
        &controller, &target_angle_centi_degrees);
    sequence = (uint8_t)(controller.vision.last_sequence + 1U);
    now_ms = controller.vision.last_valid_received_at_ms + 40U;
    final_sample_ms = now_ms + 160U;
    while (now_ms <= final_sample_ms)
    {
        assert(update_sample(
                   &controller, sequence++, 480, now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(Task3Controller_GetVisionState(
        &controller, final_sample_ms, &state));
    Task3Controller_GetConfig(&controller, &config);
    Task3Controller_ComputeNegativeReference(
        controller.negative_reference_start_x_tenths_mm,
        final_sample_ms - controller.reverse_command_ms,
        &reference);
    relative_velocity_angle = Task3Controller_ComputeAngleWithConfig(
        &config,
        reference.x_tenths_mm,
        state.x_filtered_tenths_mm,
        state.velocity_filtered_tenths_mm_per_s -
            reference.velocity_tenths_mm_per_s);
    absolute_velocity_angle = Task3Controller_ComputeAngleWithConfig(
        &config,
        reference.x_tenths_mm,
        state.x_filtered_tenths_mm,
        state.velocity_filtered_tenths_mm_per_s);
    assert(target_angle_centi_degrees == relative_velocity_angle);
    assert(relative_velocity_angle != absolute_velocity_angle);
    assert(!controller.negative_friction_active);
}

static void test_negative_position_cannot_finish_before_positive(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence = 1U;
    uint32_t now_ms = 40U;
    uint8_t index;

    Task3Controller_Init(&controller);
    Task3Controller_Start(&controller, 0U);
    for (index = 0U; index < 12U; ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   -450,
                   now_ms,
                   &target_angle_centi_degrees) !=
               TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_POSITIVE_DRIVE);
    assert(controller.stable_sample_count == 0U);
    assert(Task3Controller_IsActive(&controller));
}

static void test_negative_window_rejects_minus_45_and_accepts_minus_47(void)
{
    Task3Controller controller;
    Task3ControllerTimingResult timing;
    int16_t target_angle_centi_degrees = 0;
    static const int16_t path_to_target[] = {
        300, 100, -100, -300, -450
    };
    uint8_t sequence = 12U;
    uint32_t now_ms = 480U;
    bool done = false;
    uint8_t index;

    start_and_reach_positive(
        &controller, &target_angle_centi_degrees);
    for (index = 0U;
         index < sizeof(path_to_target) / sizeof(path_to_target[0]);
         ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   path_to_target[index],
                   now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }

    for (index = 0U; index < 10U; ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   -450,
                   now_ms,
                   &target_angle_centi_degrees) !=
               TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    assert(controller.stable_sample_count == 0U);

    for (index = 0U; index < 60U; ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   -450,
                   now_ms,
                   &target_angle_centi_degrees) !=
               TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    assert(controller.stable_sample_count == 0U);
    assert(Task3Controller_IsActive(&controller));

    for (index = 0U; index < 20U; ++index)
    {
        Task3ControllerUpdateResult result = update_sample(
            &controller,
            sequence++,
            -470,
            now_ms,
            &target_angle_centi_degrees);

        now_ms += 40U;
        if (result == TASK3_CONTROLLER_UPDATE_DONE)
        {
            done = true;
            break;
        }
    }

    assert(done);
    assert(!Task3Controller_IsActive(&controller));
    Task3Controller_GetTimingResult(&controller, &timing);
    assert(timing.negative_reached);
}

static void test_negative_window_rejects_minus_54_and_accepts_minus_53(void)
{
    static const int16_t path_below_target[] = {
        300, 100, -100, -300, -450, -540
    };
    Task3Controller controller;
    Task3ControllerTimingResult timing;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence = 12U;
    uint32_t now_ms = 480U;
    bool done = false;
    uint8_t index;

    start_and_reach_positive(
        &controller, &target_angle_centi_degrees);
    for (index = 0U;
         index < sizeof(path_below_target) /
                     sizeof(path_below_target[0]);
         ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   path_below_target[index],
                   now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    for (index = 0U; index < 70U; ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   -540,
                   now_ms,
                   &target_angle_centi_degrees) !=
               TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    assert(controller.stable_sample_count == 0U);
    Task3Controller_GetTimingResult(&controller, &timing);
    assert(!timing.negative_reached);

    for (index = 0U; index < 20U; ++index)
    {
        Task3ControllerUpdateResult result = update_sample(
            &controller,
            sequence++,
            -530,
            now_ms,
            &target_angle_centi_degrees);

        now_ms += 40U;
        if (result == TASK3_CONTROLLER_UPDATE_DONE)
        {
            done = true;
            break;
        }
    }
    assert(done);
    Task3Controller_GetTimingResult(&controller, &timing);
    assert(timing.negative_reached);
}

static void test_negative_arrival_waits_for_speed_limit(void)
{
    static const int16_t path_to_crossing[] = {
        300, 100, -100, -300, -500
    };
    Task3Controller controller;
    Task3ControllerTimingResult timing;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence = 12U;
    uint32_t now_ms = 480U;
    uint8_t index;
    bool reached = false;

    start_and_reach_positive(
        &controller, &target_angle_centi_degrees);
    for (index = 0U;
         index < sizeof(path_to_crossing) / sizeof(path_to_crossing[0]);
         ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   path_to_crossing[index],
                   now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    Task3Controller_GetTimingResult(&controller, &timing);
    assert(!timing.negative_reached);

    for (index = 0U; index < 20U; ++index)
    {
        assert(update_sample(
                   &controller,
                   sequence++,
                   -500,
                   now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
        Task3Controller_GetTimingResult(&controller, &timing);
        if (timing.negative_reached)
        {
            reached = true;
            break;
        }
    }
    assert(reached);
    assert(timing.negative_from_start_ms > 640U);
    assert(timing.negative_from_reverse_ms > 0U);
    assert(timing.negative_from_start_ms >=
           timing.negative_from_reverse_ms);
    assert(timing.negative_velocity_tenths_mm_per_s >=
           -TASK3_STABLE_MAX_VELOCITY_TENTHS_MM_PER_S);
    assert(timing.negative_velocity_tenths_mm_per_s <=
           TASK3_STABLE_MAX_VELOCITY_TENTHS_MM_PER_S);
}

static void test_negative_friction_compensation_retriggers(void)
{
    Task3Controller controller;
    Task3TuningConfig config;
    Task3NegativeReferenceState reference;
    VisionState state;
    int16_t target_angle_centi_degrees = 0;
    int16_t base_angle_centi_degrees;
    uint8_t sequence = 12U;
    uint32_t now_ms = 480U;
    uint8_t index;

    start_and_reach_positive(
        &controller, &target_angle_centi_degrees);
    for (index = 0U; index < 30U &&
         !controller.negative_friction_active; ++index)
    {
        assert(update_sample(
                   &controller, sequence++, 300, now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(controller.negative_friction_active);
    assert(Task3Controller_GetVisionState(
        &controller, now_ms - 40U, &state));
    Task3Controller_GetConfig(&controller, &config);
    Task3Controller_ComputeNegativeReference(
        controller.negative_reference_start_x_tenths_mm,
        now_ms - 40U - controller.reverse_command_ms,
        &reference);
    base_angle_centi_degrees = Task3Controller_ComputeAngleWithConfig(
        &config,
        reference.x_tenths_mm,
        state.x_filtered_tenths_mm,
        state.velocity_filtered_tenths_mm_per_s -
            reference.velocity_tenths_mm_per_s);
    if (base_angle_centi_degrees >
        config.maximum_angle_cdeg -
            TASK3_NEGATIVE_FRICTION_COMPENSATION_CDEG)
    {
        assert(target_angle_centi_degrees ==
               config.maximum_angle_cdeg);
    }
    else
    {
        assert(target_angle_centi_degrees ==
               base_angle_centi_degrees +
                   TASK3_NEGATIVE_FRICTION_COMPENSATION_CDEG);
    }

    assert(update_sample(
               &controller, sequence++, 200, now_ms,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
    now_ms += 40U;
    assert(!controller.negative_friction_active);

    for (index = 0U; index < 30U &&
         !controller.negative_friction_active; ++index)
    {
        assert(update_sample(
                   &controller, sequence++, 200, now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(controller.negative_friction_active);

    for (index = 0U; index < 8U; ++index)
    {
        int16_t x_tenths_mm = (int16_t)(100 - (int16_t)index * 100);

        assert(update_sample(
                   &controller, sequence++, x_tenths_mm, now_ms,
                   &target_angle_centi_degrees) ==
               TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING);
        now_ms += 40U;
    }
    assert(controller.vision.x_filtered_tenths_mm <=
           TASK3_NEGATIVE_FRICTION_STOP_X_TENTHS_MM);
    assert(!controller.negative_friction_active);
}

static void test_final_reference_uses_bidirectional_compensation(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;
    uint8_t sequence = 12U;
    uint32_t now_ms = 480U;
    uint8_t index;

    start_and_reach_positive(
        &controller, &target_angle_centi_degrees);
    for (index = 0U; index < 30U; ++index)
    {
        Task3ControllerUpdateResult result = update_sample(
            &controller,
            sequence++,
            -465,
            now_ms,
            &target_angle_centi_degrees);

        assert(result != TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    controller.negative_friction_active = false;
    controller.negative_friction_sample_count = 0U;
    now_ms = 2600U;
    assert(controller.vision.x_filtered_tenths_mm >
           TASK3_TARGET_MAX_X_TENTHS_MM);
    assert(controller.vision.x_filtered_tenths_mm + 500 <
           TASK3_NEGATIVE_FRICTION_MIN_LAG_TENTHS_MM);
    for (index = 0U; index <
         TASK3_NEGATIVE_FINAL_FRICTION_SAMPLE_COUNT; ++index)
    {
        Task3ControllerUpdateResult result = update_sample(
            &controller,
            sequence++,
            -465,
            now_ms,
            &target_angle_centi_degrees);

        assert(result != TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    assert(controller.negative_friction_active);
    assert(controller.negative_friction_direction == 1);

    for (index = 0U; index < 3U; ++index)
    {
        Task3ControllerUpdateResult result = update_sample(
            &controller,
            sequence++,
            -480,
            now_ms,
            &target_angle_centi_degrees);

        assert(result != TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    assert(controller.vision.x_filtered_tenths_mm >=
           TASK3_TARGET_MIN_X_TENTHS_MM);
    assert(controller.vision.x_filtered_tenths_mm <=
           TASK3_TARGET_MAX_X_TENTHS_MM);
    assert(!controller.negative_friction_active);

    for (index = 0U; index < 12U &&
         !controller.negative_friction_active; ++index)
    {
        Task3ControllerUpdateResult result = update_sample(
            &controller,
            sequence++,
            -560,
            now_ms,
            &target_angle_centi_degrees);

        assert(result != TASK3_CONTROLLER_UPDATE_DONE);
        now_ms += 40U;
    }
    assert(controller.negative_friction_active);
    assert(controller.negative_friction_direction == -1);
}

static void test_initial_vision_timeout_commands_balance(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 123;

    Task3Controller_Init(&controller);
    Task3Controller_Start(&controller, 0U);
    assert(Task3Controller_Update(
               &controller,
               TASK3_VISION_ACQUIRE_TIMEOUT_MS,
               0,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE);
    assert(target_angle_centi_degrees ==
           TASK3_BALANCE_ANGLE_CDEG);
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_LEVEL_TIMEOUT);
}

static void test_stale_vision_commands_balance(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;

    start_positive_drive(
        &controller, &target_angle_centi_degrees);
    assert(Task3Controller_Update(
               &controller,
               40U + VISION_STALE_TIMEOUT_MS,
               0,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_NONE);
    assert(Task3Controller_Update(
               &controller,
               41U + VISION_STALE_TIMEOUT_MS,
               0,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE);
    assert(target_angle_centi_degrees ==
           TASK3_BALANCE_ANGLE_CDEG);
    assert(controller.phase ==
           TASK3_CONTROLLER_PHASE_LEVEL_TIMEOUT);
}

static void test_controller_has_no_total_runtime_timeout(void)
{
    Task3Controller controller;
    int16_t target_angle_centi_degrees = 0;
    VisionSample sample;

    Task3Controller_Init(&controller);
    Task3Controller_Start(&controller, 1000U);
    sample = make_sample(1U, 0, 61000U);
    assert(Task3Controller_Update(
               &controller,
               61000U,
               &sample,
               &target_angle_centi_degrees) ==
           TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE);
    assert(target_angle_centi_degrees ==
           TASK3_POSITIVE_DRIVE_ANGLE_CDEG);
    assert(Task3Controller_IsActive(&controller));
}

int main(void)
{
    test_control_parameters();
    test_negative_reference_position_and_velocity();
    test_runtime_tuning_config();
    test_soft_angle_limit_also_caps_fixed_drive();
    test_waits_for_first_vision_sample();
    test_positive_drive_is_held_until_predicted_threshold();
    test_prediction_starts_pd_before_raw_thirty_mm();
    test_low_speed_approach_reenters_drive_and_recovers();
    test_low_speed_boost_is_not_overridden_by_stop_prediction();
    test_high_speed_approach_uses_stronger_brake();
    test_positive_boost_stays_off_after_target_overshoot();
    test_positive_arrival_accepts_high_speed();
    test_positive_arrival_requires_46_5_to_52_mm();
    test_negative_pd_uses_reference_relative_velocity();
    test_negative_position_cannot_finish_before_positive();
    test_negative_window_rejects_minus_45_and_accepts_minus_47();
    test_negative_window_rejects_minus_54_and_accepts_minus_53();
    test_negative_arrival_waits_for_speed_limit();
    test_negative_friction_compensation_retriggers();
    test_final_reference_uses_bidirectional_compensation();
    test_initial_vision_timeout_commands_balance();
    test_stale_vision_commands_balance();
    test_controller_has_no_total_runtime_timeout();
    return 0;
}
