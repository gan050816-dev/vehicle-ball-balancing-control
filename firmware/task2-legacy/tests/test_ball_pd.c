#include <assert.h>

#include "ball_pd.h"

static void test_gains_sign_slew_and_saturation(void)
{
    BallPd controller;

    BallPd_Init(&controller);
    assert(controller.command_angle_cdeg == 0);

    /* Task 3 Kp: x=10 mm requests +3.44 degrees. */
    assert(BallPd_Update(&controller, 100, 0, 0) == 344);
    assert(controller.unsaturated_angle_cdeg == 344);
    assert(!controller.saturated);
    assert(BallPd_Update(&controller, 100, 0, 0) == 344);

    /* Task 3 Kd: v=10 mm/s contributes +0.57 degrees. */
    assert(BallPd_Update(&controller, 0, 100, 0) == 57);
    assert(controller.unsaturated_angle_cdeg == 57);

    (void)BallPd_Update(&controller, 0, 10000, 0);
    assert(controller.saturated);
    assert(controller.command_angle_cdeg == 487);
}

static void test_acceleration_feedforward(void)
{
    BallPd controller;
    uint8_t index;

    BallPd_Init(&controller);
    for (index = 0U; index < 5U; ++index)
    {
        (void)BallPd_Update(&controller, 0, 0, 300);
    }
    assert(controller.feedforward_angle_cdeg == -2015);
    assert(controller.unsaturated_angle_cdeg == -2015);
    assert(controller.command_angle_cdeg == -2015);
    assert(!controller.saturated);

    BallPd_Init(&controller);
    assert(BallPd_Update(&controller, 0, 0, -300) == 430);
    assert(controller.feedforward_angle_cdeg == 2015);
}

static void test_negative_feedback_and_angle_limit(void)
{
    BallPd controller;
    uint8_t index;

    BallPd_Init(&controller);
    for (index = 0U; index < 20U; ++index)
    {
        (void)BallPd_Update(&controller, -1500, -10000, 0);
    }
    assert(controller.command_angle_cdeg ==
           BALL_PD_MIN_ANGLE_CDEG);
}

int main(void)
{
    test_gains_sign_slew_and_saturation();
    test_acceleration_feedforward();
    test_negative_feedback_and_angle_limit();
    return 0;
}
