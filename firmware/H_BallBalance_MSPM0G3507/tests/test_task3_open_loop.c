#include <assert.h>
#include <stdint.h>

#include "task3_open_loop.h"

static void assert_command(
    Task3OpenLoop *task,
    uint32_t now_ms,
    const Task3BallSample *sample,
    uint16_t expected_target)
{
    uint16_t target = 0U;

    assert(Task3OpenLoop_Update(task, now_ms, sample, &target) ==
           TASK3_UPDATE_COMMAND_POSITION);
    assert(target == expected_target);
}

static Task3BallSample make_sample(
    uint8_t sequence,
    int16_t x_tenths_mm,
    uint32_t received_at_ms)
{
    Task3BallSample sample;

    sample.valid = true;
    sample.sequence = sequence;
    sample.x_tenths_mm = x_tenths_mm;
    sample.received_at_ms = received_at_ms;
    return sample;
}

static void test_timing_calibration_records_both_arrivals(void)
{
    Task3OpenLoop task;
    Task3TimingResult timing;
    Task3BallSample sample;
    uint16_t target = 0U;

    assert(TASK3_REVERSE_FRICTION_DIAGNOSTIC == 0U);
    assert(TASK3_TIMING_CALIBRATION == 1U);
    assert(TASK3_TIMING_DRIVE_TARGET_TENTHS_MM == 89U);
    assert(TASK3_TIMING_POSITIVE_X_TENTHS_MM == 500);
    assert(TASK3_TIMING_NEGATIVE_X_TENTHS_MM == -500);
    assert(TASK3_TIMING_BRAKE_TRIGGER_X_TENTHS_MM == 600);
    assert(
        TASK3_TIMING_BRAKE_TRIGGER_VELOCITY_TENTHS_MM_PER_S ==
        -200);
    assert(TASK3_TIMING_BRAKE_TARGET_TENTHS_MM == 49U);
    assert(TASK3_TIMING_BRAKE_RELEASE_SAMPLES == 2U);
    assert(TASK3_RACK_AWAY_FROM_HINGE_TENTHS_MM == 99U);
    assert(TASK3_RACK_LEVEL_TENTHS_MM == 149U);
    assert(TASK3_RACK_TOWARD_HINGE_TENTHS_MM == 199U);
    assert(TASK3_FIRST_REVERSE_MS == 650U);
    assert(TASK3_SECOND_REVERSE_MS == 2600U);
    assert(TASK3_RETURN_LEVEL_MS == 3100U);
    assert(TASK3_COMPLETE_MS == 4500U);
    assert(TASK3_TIMEOUT_MS == 5000U);

    Task3OpenLoop_Init(&task);
    assert(!Task3OpenLoop_IsActive(&task));
    Task3OpenLoop_Start(&task, 1000U);
    assert(Task3OpenLoop_IsActive(&task));

    assert_command(
        &task, 1000U, 0, TASK3_TIMING_DRIVE_TARGET_TENTHS_MM);

    sample = make_sample(1U, 0, 1040U);
    assert(Task3OpenLoop_Update(
               &task, 1040U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(2U, 490, 1080U);
    assert(Task3OpenLoop_Update(
               &task, 1080U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(3U, 500, 1120U);
    assert_command(
        &task, 1120U, &sample,
        TASK3_RACK_TOWARD_HINGE_TENTHS_MM);

    Task3OpenLoop_GetTimingResult(&task, &timing);
    assert(timing.positive_reached);
    assert(!timing.negative_reached);
    assert(timing.positive_from_start_ms == 120U);
    assert(timing.positive_velocity_tenths_mm_per_s == 250);

    sample = make_sample(3U, -500, 1160U);
    assert(Task3OpenLoop_Update(
               &task, 1160U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(4U, 600, 1200U);
    assert(Task3OpenLoop_Update(
               &task, 1200U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(5U, 590, 1240U);
    assert_command(
        &task, 1240U, &sample,
        TASK3_TIMING_BRAKE_TARGET_TENTHS_MM);

    sample = make_sample(6U, -500, 1320U);
    assert_command(
        &task, 1320U, &sample, TASK3_RACK_LEVEL_TENTHS_MM);

    Task3OpenLoop_GetTimingResult(&task, &timing);
    assert(timing.negative_reached);
    assert(timing.negative_from_start_ms == 320U);
    assert(timing.negative_from_reverse_ms == 200U);
    assert(timing.negative_velocity_tenths_mm_per_s == -13625);

    assert(Task3OpenLoop_Update(&task, 5499U, 0, &target) ==
           TASK3_UPDATE_NONE);
    assert(Task3OpenLoop_Update(&task, 5500U, 0, &target) ==
           TASK3_UPDATE_DONE);
    assert(!Task3OpenLoop_IsActive(&task));
}

static void test_prebrake_waits_for_return_and_releases_after_reversal(void)
{
    Task3OpenLoop task;
    Task3TimingResult timing;
    Task3BallSample sample;
    uint16_t target = 0U;

    Task3OpenLoop_Init(&task);
    Task3OpenLoop_Start(&task, 0U);
    assert_command(
        &task, 0U, 0, TASK3_TIMING_DRIVE_TARGET_TENTHS_MM);

    sample = make_sample(1U, 500, 40U);
    assert_command(
        &task, 40U, &sample,
        TASK3_RACK_TOWARD_HINGE_TENTHS_MM);

    sample = make_sample(2U, 580, 80U);
    assert(Task3OpenLoop_Update(
               &task, 80U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(3U, 600, 120U);
    assert(Task3OpenLoop_Update(
               &task, 120U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(4U, 590, 160U);
    assert_command(
        &task, 160U, &sample,
        TASK3_TIMING_BRAKE_TARGET_TENTHS_MM);

    sample = make_sample(5U, 590, 200U);
    assert(Task3OpenLoop_Update(
               &task, 200U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(5U, 600, 240U);
    assert(Task3OpenLoop_Update(
               &task, 240U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(6U, 592, 280U);
    assert_command(
        &task, 280U, &sample, TASK3_RACK_LEVEL_TENTHS_MM);

    Task3OpenLoop_GetTimingResult(&task, &timing);
    assert(timing.positive_reached);
    assert(!timing.negative_reached);
    assert(Task3OpenLoop_IsActive(&task));
}

static void test_direct_negative_arrival_returns_level(void)
{
    Task3OpenLoop task;
    Task3TimingResult timing;
    Task3BallSample sample;
    uint16_t target = 0U;

    Task3OpenLoop_Init(&task);
    Task3OpenLoop_Start(&task, 0U);
    assert_command(
        &task, 0U, 0, TASK3_TIMING_DRIVE_TARGET_TENTHS_MM);

    sample = make_sample(1U, 0, 40U);
    assert(Task3OpenLoop_Update(
               &task, 40U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(2U, 500, 80U);
    assert_command(
        &task, 80U, &sample,
        TASK3_RACK_TOWARD_HINGE_TENTHS_MM);

    sample = make_sample(3U, -500, 120U);
    assert_command(
        &task, 120U, &sample, TASK3_RACK_LEVEL_TENTHS_MM);

    Task3OpenLoop_GetTimingResult(&task, &timing);
    assert(timing.positive_reached);
    assert(timing.negative_reached);
    assert(timing.negative_velocity_tenths_mm_per_s == -25000);
}

static void test_missing_arrival_returns_level_but_does_not_finish(void)
{
    Task3OpenLoop task;
    uint16_t target = 0U;

    Task3OpenLoop_Init(&task);
    Task3OpenLoop_Start(&task, 0U);
    assert_command(
        &task, 0U, 0, TASK3_TIMING_DRIVE_TARGET_TENTHS_MM);

    assert_command(
        &task, TASK3_COMPLETE_MS, 0,
        TASK3_RACK_LEVEL_TENTHS_MM);
    assert(Task3OpenLoop_Update(
               &task, TASK3_TIMEOUT_MS, 0, &target) ==
           TASK3_UPDATE_NONE);
    assert(Task3OpenLoop_IsActive(&task));
}

static void test_sequence_wrap_is_new_sample(void)
{
    Task3OpenLoop task;
    Task3BallSample sample;
    uint16_t target = 0U;

    Task3OpenLoop_Init(&task);
    Task3OpenLoop_Start(&task, 1000U);
    assert_command(
        &task, 1000U, 0, TASK3_TIMING_DRIVE_TARGET_TENTHS_MM);

    sample = make_sample(255U, 490, 1040U);
    assert(Task3OpenLoop_Update(
               &task, 1040U, &sample, &target) ==
           TASK3_UPDATE_NONE);
    sample = make_sample(0U, 500, 1080U);
    assert_command(
        &task, 1080U, &sample,
        TASK3_RACK_TOWARD_HINGE_TENTHS_MM);
}

static void test_invalid_and_prestart_samples_are_ignored(void)
{
    Task3OpenLoop task;
    Task3BallSample sample;
    uint16_t target = 0U;

    Task3OpenLoop_Init(&task);
    Task3OpenLoop_Start(&task, 1000U);
    assert_command(
        &task, 1000U, 0, TASK3_TIMING_DRIVE_TARGET_TENTHS_MM);

    sample = make_sample(1U, 500, 1040U);
    sample.valid = false;
    assert(Task3OpenLoop_Update(
               &task, 1040U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(2U, 500, 999U);
    assert(Task3OpenLoop_Update(
               &task, 1040U, &sample, &target) ==
           TASK3_UPDATE_NONE);

    sample = make_sample(3U, 500, 1080U);
    assert_command(
        &task, 1080U, &sample,
        TASK3_RACK_TOWARD_HINGE_TENTHS_MM);
}

static void test_cancel_prevents_further_commands(void)
{
    Task3OpenLoop task;
    uint16_t target = 0U;

    Task3OpenLoop_Init(&task);
    Task3OpenLoop_Start(&task, 0U);
    Task3OpenLoop_Cancel(&task);
    assert(Task3OpenLoop_Update(&task, 0U, 0, &target) ==
           TASK3_UPDATE_NONE);
    assert(!Task3OpenLoop_IsActive(&task));
}

int main(void)
{
    test_timing_calibration_records_both_arrivals();
    test_prebrake_waits_for_return_and_releases_after_reversal();
    test_direct_negative_arrival_returns_level();
    test_missing_arrival_returns_level_but_does_not_finish();
    test_sequence_wrap_is_new_sample();
    test_invalid_and_prestart_samples_are_ignored();
    test_cancel_prevents_further_commands();
    return 0;
}
