#include <assert.h>
#include <math.h>

#include "speed_pid.h"

static void test_positive_error_produces_positive_output(void)
{
    SpeedPid pid;
    SpeedPid_Init(&pid, 0.5f, 0.1f, 0.0f, 1000.0f, 500.0f);

    assert(SpeedPid_Update(&pid, 400.0f, 100.0f, 0.01f) > 0.0f);
}

static void test_output_and_integral_are_limited(void)
{
    SpeedPid pid;
    SpeedPid_Init(&pid, 10.0f, 10.0f, 0.0f, 200.0f, 30.0f);

    for (int i = 0; i < 100; ++i)
    {
        assert(SpeedPid_Update(&pid, 1000.0f, 0.0f, 0.01f) <= 200.0f);
    }
    assert(fabsf(pid.integral) <= 30.0f);
}

static void test_reset_clears_history(void)
{
    SpeedPid pid;
    SpeedPid_Init(&pid, 0.0f, 1.0f, 1.0f, 1000.0f, 500.0f);
    (void)SpeedPid_Update(&pid, 100.0f, 0.0f, 0.01f);

    SpeedPid_Reset(&pid);

    assert(pid.integral == 0.0f);
    assert(pid.previous_error == 0.0f);
    assert(pid.has_previous == 0U);
}

int main(void)
{
    test_positive_error_produces_positive_output();
    test_output_and_integral_are_limited();
    test_reset_clears_history();
    return 0;
}
