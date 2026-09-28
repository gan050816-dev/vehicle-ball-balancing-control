#include <assert.h>
#include <stdint.h>

#include "line_drive_service.h"

#define CENTER_LINE_MASK 0x18U
#define WIDE_LINE_MASK 0xFFU
#define T2_TRACK_LENGTH_MICROMETERS 6141592LL
#define BALANCE_TRACK_LENGTH_MICROMETERS 6141592LL

static void update_with_motion(LineDriveService *service,
                               uint8_t line_mask,
                               int32_t *left_count,
                               int32_t *right_count)
{
    *left_count += 3;
    *right_count += 3;
    LineDriveService_Update(
        service, line_mask, *left_count, *right_count);
}

static void test_requirement_4_holds_208_after_8s(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    uint32_t tick;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_REQUIREMENT_4,
        left_count, right_count));

    for (tick = 0U; tick < 799U; ++tick)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
    }
    assert(service.elapsed_ticks == 799U);
    assert(service.requested_forward_speed_mm_s == 208);

    update_with_motion(
        &service, CENTER_LINE_MASK, &left_count, &right_count);
    assert(service.elapsed_ticks == 800U);
    assert(service.requested_forward_speed_mm_s == 208);

    while (service.elapsed_ticks < 1000U)
    {
        update_with_motion(
            &service, WIDE_LINE_MASK, &left_count, &right_count);
        assert(service.state == LINE_DRIVE_RUNNING);
    }
    assert(LineDriveService_GetElapsedMs(&service) == 10000U);
    assert(service.state == LINE_DRIVE_RUNNING);
    assert(service.stop_reason == LINE_DRIVE_STOP_NONE);
    assert(service.requested_forward_speed_mm_s == 208);
    assert(service.peak_acceleration_mm_s2 == 300U);
}

static void test_requirement_5_holds_230_and_keeps_running(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_REQUIREMENT_5,
        left_count, right_count));
    while (service.elapsed_ticks < 3000U)
    {
        uint8_t mask = service.elapsed_ticks >= 900U ?
            WIDE_LINE_MASK : CENTER_LINE_MASK;

        update_with_motion(
            &service, mask, &left_count, &right_count);
        assert(service.state == LINE_DRIVE_RUNNING);
    }
    assert(LineDriveService_GetElapsedMs(&service) == 30000U);
    assert(service.state == LINE_DRIVE_RUNNING);
    assert(service.stop_reason == LINE_DRIVE_STOP_NONE);
    assert(service.requested_forward_speed_mm_s == 230);
    assert(service.peak_acceleration_mm_s2 == 300U);
}

static void assert_balance_differential_is_smoothed(
    LineDriveMode mode,
    uint8_t line_mask,
    int16_t cruise_speed_mm_s,
    int16_t expected_left_at_cruise,
    int16_t expected_right_at_cruise)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    int16_t previous_differential = 0;
    uint16_t tick;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, mode, left_count, right_count));

    for (tick = 0U; tick < 60U; ++tick)
    {
        update_with_motion(
            &service, line_mask, &left_count, &right_count);
        assert(service.target_left_mm_s >= 0);
        assert(service.target_right_mm_s >= 0);
        assert(service.applied_differential_mm_s -
                   previous_differential <= 20);
        assert(service.applied_differential_mm_s -
                   previous_differential >= -20);
        if (service.forward_speed_mm_s > 20)
        {
            assert(service.target_left_mm_s >= 20);
            assert(service.target_right_mm_s >= 20);
        }
        assert((int32_t)service.target_left_mm_s +
                   service.target_right_mm_s ==
               (int32_t)service.forward_speed_mm_s * 2);
        previous_differential = service.applied_differential_mm_s;
    }

    while (service.forward_speed_mm_s < cruise_speed_mm_s)
    {
        update_with_motion(
            &service, line_mask, &left_count, &right_count);
    }
    assert(service.target_left_mm_s == expected_left_at_cruise);
    assert(service.target_right_mm_s == expected_right_at_cruise);
}

static void test_balance_modes_smooth_and_limit_differential(void)
{
    assert_balance_differential_is_smoothed(
        LINE_DRIVE_MODE_REQUIREMENT_4, 0x80U, 208, 308, 108);
    assert_balance_differential_is_smoothed(
        LINE_DRIVE_MODE_REQUIREMENT_4, 0x01U, 208, 108, 308);
    assert_balance_differential_is_smoothed(
        LINE_DRIVE_MODE_REQUIREMENT_5, 0x80U, 230, 330, 130);
    assert_balance_differential_is_smoothed(
        LINE_DRIVE_MODE_REQUIREMENT_5, 0x01U, 230, 130, 330);
}

static void test_requirement_5_constant_speed_keeps_lap_margin(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    int64_t distance_micrometers = 0;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_REQUIREMENT_5,
        left_count, right_count));
    while (distance_micrometers <
               BALANCE_TRACK_LENGTH_MICROMETERS &&
           service.elapsed_ticks < 3000U)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
        distance_micrometers +=
            (int64_t)service.forward_speed_mm_s * 10;
    }

    assert(distance_micrometers >= BALANCE_TRACK_LENGTH_MICROMETERS);
    assert(service.elapsed_ticks >= 2700U);
    assert(service.elapsed_ticks <= 2750U);
}

static void test_t2_startup_keeps_both_wheels_forward(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    int16_t previous_differential = 0;
    uint16_t tick;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_T2, left_count, right_count));
    for (tick = 0U; tick < 100U; ++tick)
    {
        update_with_motion(
            &service, 0x80U, &left_count, &right_count);
        assert(service.target_left_mm_s >= 0);
        assert(service.target_right_mm_s >= 0);
        assert((int32_t)service.target_left_mm_s +
                   service.target_right_mm_s ==
               (int32_t)service.forward_speed_mm_s * 2);
        assert(service.applied_differential_mm_s -
                   previous_differential <= 40);
        assert(service.applied_differential_mm_s -
                   previous_differential >= -40);
        if (service.forward_speed_mm_s > 20)
        {
            assert(service.target_left_mm_s >= 20);
            assert(service.target_right_mm_s >= 20);
        }
        previous_differential = service.applied_differential_mm_s;
    }
}

static void test_t2_still_uses_a_line_finish(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_T2, left_count, right_count));
    while (service.elapsed_ticks < 1397U)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
    }
    update_with_motion(
        &service, WIDE_LINE_MASK, &left_count, &right_count);
    update_with_motion(
        &service, WIDE_LINE_MASK, &left_count, &right_count);
    assert(service.elapsed_ticks == 1399U);
    assert(service.finish_confirm_ticks == 0U);
    assert(service.state == LINE_DRIVE_RUNNING);
    update_with_motion(
        &service, WIDE_LINE_MASK, &left_count, &right_count);
    assert(service.elapsed_ticks == 1400U);
    assert(service.finish_confirm_ticks == 1U);
    assert(service.requested_forward_speed_mm_s == 100);
    assert(service.state == LINE_DRIVE_RUNNING);
    update_with_motion(
        &service, WIDE_LINE_MASK, &left_count, &right_count);
    assert(service.state == LINE_DRIVE_RUNNING);
    update_with_motion(
        &service, WIDE_LINE_MASK, &left_count, &right_count);
    assert(service.state == LINE_DRIVE_STOPPED);
    assert(service.stop_reason == LINE_DRIVE_STOP_FINISH_LINE);
}

static void test_t2_profile_starts_deceleration_at_13_5_seconds(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    int64_t distance_micrometers = 0;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_T2, left_count, right_count));

    while (service.elapsed_ticks < 1349U)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
        distance_micrometers +=
            (int64_t)service.forward_speed_mm_s * 10;
    }
    assert(service.requested_forward_speed_mm_s == 410);
    assert(service.forward_speed_mm_s == 410);

    update_with_motion(
        &service, CENTER_LINE_MASK, &left_count, &right_count);
    distance_micrometers +=
        (int64_t)service.forward_speed_mm_s * 10;
    assert(service.elapsed_ticks == 1350U);
    assert(service.requested_forward_speed_mm_s == 100);

    while (service.forward_speed_mm_s != 100)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
        distance_micrometers +=
            (int64_t)service.forward_speed_mm_s * 10;
    }
    assert(service.elapsed_ticks >= 1420U);
    assert(service.elapsed_ticks <= 1435U);
    assert(service.forward_speed_mm_s == 100);
    assert(service.state == LINE_DRIVE_RUNNING);

    while (service.elapsed_ticks < 1999U)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
        distance_micrometers +=
            (int64_t)service.forward_speed_mm_s * 10;
    }
    assert(distance_micrometers >= 6110000LL);
    assert(distance_micrometers <= 6125000LL);
    assert(distance_micrometers < T2_TRACK_LENGTH_MICROMETERS);
}

static void test_t2_high_error_uses_stronger_turning_gain(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    uint8_t tick;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_T2, left_count, right_count));
    while (service.elapsed_ticks < 200U)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
    }
    update_with_motion(
        &service, 0x80U, &left_count, &right_count);
    assert(service.forward_speed_mm_s == 410);
    assert(service.applied_differential_mm_s == 40);
    assert(service.target_left_mm_s == 450);
    assert(service.target_right_mm_s == 370);

    for (tick = 0U; tick < 8U; ++tick)
    {
        update_with_motion(
            &service, 0x80U, &left_count, &right_count);
    }
    assert(service.applied_differential_mm_s == 270);
    assert(service.target_left_mm_s == 680);
    assert(service.target_right_mm_s == 140);

    update_with_motion(
        &service, 0x40U, &left_count, &right_count);
    update_with_motion(
        &service, 0x40U, &left_count, &right_count);
    assert(service.applied_differential_mm_s == 270);
    assert(service.target_left_mm_s == 680);
    assert(service.target_right_mm_s == 140);

    while (service.elapsed_ticks < 1500U ||
           service.forward_speed_mm_s != 100)
    {
        update_with_motion(
            &service, 0x80U, &left_count, &right_count);
    }
    assert(service.applied_differential_mm_s == 80);
    assert(service.target_left_mm_s == 180);
    assert(service.target_right_mm_s == 20);
}

static void test_t2_reversal_slews_without_wheel_target_crossing_zero(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    int16_t previous_differential;
    uint8_t tick;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_T2, left_count, right_count));
    while (service.elapsed_ticks < 200U)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
    }
    for (tick = 0U; tick < 9U; ++tick)
    {
        update_with_motion(
            &service, 0x80U, &left_count, &right_count);
    }
    assert(service.applied_differential_mm_s == 270);

    previous_differential = service.applied_differential_mm_s;
    for (tick = 0U; tick < 20U; ++tick)
    {
        update_with_motion(
            &service, 0x01U, &left_count, &right_count);
        assert(service.applied_differential_mm_s -
                   previous_differential <= 40);
        assert(service.applied_differential_mm_s -
                   previous_differential >= -40);
        assert(service.target_left_mm_s >= 20);
        assert(service.target_right_mm_s >= 20);
        assert((int32_t)service.target_left_mm_s +
                   service.target_right_mm_s ==
               (int32_t)service.forward_speed_mm_s * 2);
        previous_differential = service.applied_differential_mm_s;
    }
    assert(service.applied_differential_mm_s == -270);
}

static void test_t2_timeout_remains_a_fault(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_T2, left_count, right_count));
    while (service.elapsed_ticks < 1999U)
    {
        update_with_motion(
            &service, CENTER_LINE_MASK, &left_count, &right_count);
        assert(service.state == LINE_DRIVE_RUNNING);
    }
    update_with_motion(
        &service, CENTER_LINE_MASK, &left_count, &right_count);
    assert(service.state == LINE_DRIVE_FAULT);
    assert(service.stop_reason == LINE_DRIVE_STOP_TIMEOUT);
    assert(service.applied_differential_mm_s == 0);
    assert(LineDriveService_GetElapsedMs(&service) == 20000U);
}

static void test_line_loss_remains_a_fault(void)
{
    LineDriveService service;
    int32_t left_count = 0;
    int32_t right_count = 0;
    uint16_t tick;

    LineDriveService_Init(&service);
    assert(LineDriveService_Start(
        &service, LINE_DRIVE_MODE_REQUIREMENT_5,
        left_count, right_count));
    for (tick = 0U; tick < 99U; ++tick)
    {
        update_with_motion(
            &service, 0U, &left_count, &right_count);
        assert(service.state == LINE_DRIVE_RUNNING);
    }
    update_with_motion(&service, 0U, &left_count, &right_count);
    assert(service.state == LINE_DRIVE_FAULT);
    assert(service.stop_reason == LINE_DRIVE_STOP_LINE_LOST);
}

int main(void)
{
    test_requirement_4_holds_208_after_8s();
    test_requirement_5_holds_230_and_keeps_running();
    test_balance_modes_smooth_and_limit_differential();
    test_requirement_5_constant_speed_keeps_lap_margin();
    test_t2_startup_keeps_both_wheels_forward();
    test_t2_still_uses_a_line_finish();
    test_t2_profile_starts_deceleration_at_13_5_seconds();
    test_t2_high_error_uses_stronger_turning_gain();
    test_t2_reversal_slews_without_wheel_target_crossing_zero();
    test_t2_timeout_remains_a_fault();
    test_line_loss_remains_a_fault();
    return 0;
}
