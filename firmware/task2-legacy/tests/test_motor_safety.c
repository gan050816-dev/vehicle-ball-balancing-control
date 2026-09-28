#include <assert.h>

#include "motor_safety.h"

static void test_boot_and_zero_command_use_active_brake(void)
{
    MotorOutput output = MotorSafety_MakeOutput(0, 1000U, 0U, 0U);

    assert(output.mode == MOTOR_MODE_BRAKE);
    assert(output.in1 == 1U);
    assert(output.in2 == 1U);
    assert(output.pwm_compare == 1000U);
}

static void test_direction_can_be_inverted_per_motor(void)
{
    MotorOutput normal = MotorSafety_MakeOutput(400, 1000U, 0U, 0U);
    MotorOutput inverted = MotorSafety_MakeOutput(400, 1000U, 1U, 0U);

    assert(normal.in1 != inverted.in1);
    assert(normal.in2 != inverted.in2);
    assert(normal.pwm_compare == 400U);
    assert(inverted.pwm_compare == 400U);
}

static void test_command_is_saturated(void)
{
    MotorOutput output = MotorSafety_MakeOutput(-1400, 1000U, 0U, 0U);

    assert(output.pwm_compare == 1000U);
}

static void test_emergency_brake_overrides_command(void)
{
    MotorOutput output = MotorSafety_MakeOutput(700, 1000U, 0U, 1U);

    assert(output.mode == MOTOR_MODE_BRAKE);
    assert(output.in1 == 1U && output.in2 == 1U);
}

static void test_stall_requires_consecutive_bad_ticks(void)
{
    StallMonitor monitor;
    StallMonitor_Init(&monitor, 300, 30, 4U);

    assert(StallMonitor_Update(&monitor, 500, 10) == 0U);
    assert(StallMonitor_Update(&monitor, 500, 10) == 0U);
    assert(StallMonitor_Update(&monitor, 500, 10) == 0U);
    assert(StallMonitor_Update(&monitor, 500, 10) == 1U);
    assert(StallMonitor_Update(&monitor, 0, 0) == 0U);
}

static void test_direction_change_holds_brake_before_reversal(void)
{
    MotorDirectionGuard guard;
    MotorDirectionGuard_Init(&guard);

    assert(MotorDirectionGuard_Apply(&guard, 500, 3U) == 500);
    assert(MotorDirectionGuard_IsBraking(&guard) == 0U);
    assert(MotorDirectionGuard_Apply(&guard, -500, 3U) == 0);
    assert(MotorDirectionGuard_IsBraking(&guard) != 0U);
    assert(MotorDirectionGuard_Apply(&guard, -500, 3U) == 0);
    assert(MotorDirectionGuard_IsBraking(&guard) != 0U);
    assert(MotorDirectionGuard_Apply(&guard, -500, 3U) == 0);
    assert(MotorDirectionGuard_IsBraking(&guard) != 0U);
    assert(MotorDirectionGuard_Apply(&guard, -500, 3U) == -500);
    assert(MotorDirectionGuard_IsBraking(&guard) == 0U);
}

static void test_pwm_ramp_limits_each_update(void)
{
    MotorPwmRamp ramp;

    MotorPwmRamp_Init(&ramp);
    assert(MotorPwmRamp_Apply(&ramp, 200, 40U) == 40);
    assert(MotorPwmRamp_Apply(&ramp, 200, 40U) == 80);
    assert(MotorPwmRamp_Apply(&ramp, -200, 40U) == 40);
    MotorPwmRamp_Reset(&ramp);
    assert(MotorPwmRamp_Apply(&ramp, -200, 40U) == -40);
    assert(MotorPwmRamp_Apply(&ramp, -200, 0U) == -40);
}

int main(void)
{
    test_boot_and_zero_command_use_active_brake();
    test_direction_can_be_inverted_per_motor();
    test_command_is_saturated();
    test_emergency_brake_overrides_command();
    test_stall_requires_consecutive_bad_ticks();
    test_direction_change_holds_brake_before_reversal();
    test_pwm_ramp_limits_each_update();
    return 0;
}
