#include "ball_pd.h"

static int32_t DivideRounded(int64_t numerator, int32_t denominator)
{
    if (numerator >= 0)
    {
        return (numerator + (denominator / 2)) / denominator;
    }
    return (numerator - (denominator / 2)) / denominator;
}

static int32_t Clamp(int32_t value, int32_t minimum, int32_t maximum)
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

void BallPd_Init(BallPd *controller)
{
    controller->command_angle_cdeg = 0;
    controller->unsaturated_angle_cdeg = 0;
    controller->feedforward_angle_cdeg = 0;
    controller->saturated = 0U;
}

int16_t BallPd_Update(BallPd *controller,
                      int16_t predicted_x_tenths_mm,
                      int16_t velocity_tenths_mm_s,
                      int16_t forward_acceleration_mm_s2)
{
    int32_t feedback_angle = DivideRounded(
        (int64_t)BALL_PD_KP_NUMERATOR *
            predicted_x_tenths_mm +
        (int64_t)BALL_PD_KD_NUMERATOR *
            velocity_tenths_mm_s,
        BALL_PD_GAIN_DENOMINATOR);
    int32_t feedforward_term = DivideRounded(
        -(int64_t)BALL_PD_ACCEL_FF_NUMERATOR *
            forward_acceleration_mm_s2,
        BALL_PD_ACCEL_FF_DENOMINATOR);
    int32_t raw_angle = feedback_angle + feedforward_term;
    int32_t limited_angle = Clamp(
        raw_angle,
        BALL_PD_MIN_ANGLE_CDEG,
        BALL_PD_MAX_ANGLE_CDEG);
    int32_t previous = controller->command_angle_cdeg;
    int32_t delta = Clamp(
        limited_angle - previous,
        -BALL_PD_SLEW_LIMIT_CDEG,
        BALL_PD_SLEW_LIMIT_CDEG);
    int32_t target = previous + delta;

    controller->unsaturated_angle_cdeg =
        (int16_t)Clamp(raw_angle, INT16_MIN, INT16_MAX);
    controller->feedforward_angle_cdeg =
        (int16_t)Clamp(feedforward_term, INT16_MIN, INT16_MAX);
    controller->saturated = raw_angle != limited_angle ? 1U : 0U;
    controller->command_angle_cdeg = (int16_t)target;
    return controller->command_angle_cdeg;
}
