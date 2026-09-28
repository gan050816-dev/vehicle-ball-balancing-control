#include "drive_logic.h"

/* MG513X Hall: 13 PPR * 28:1 reduction * 4x AB decoding. */
#define DRIVE_ENCODER_COUNTS_REV 1456.0f
/* Initial theoretical circumference for the 65 mm wheel. */
#define DRIVE_WHEEL_CIRCLE_MM 204.20352f

static int16_t DriveLogic_Limit(int32_t value, int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

static uint32_t DriveLogic_IntegerSquareRoot(uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = 1UL << 30U;

    while (bit > value)
    {
        bit >>= 2U;
    }
    while (bit != 0U)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return result;
}

DriveWheelTargets DriveLogic_MixDifferential(int16_t forward_mm_s,
                                              int16_t differential_mm_s,
                                              int16_t max_wheel_mm_s)
{
    DriveWheelTargets target;

    target.left_mm_s = DriveLogic_Limit(
        (int32_t)forward_mm_s + differential_mm_s, max_wheel_mm_s);
    target.right_mm_s = DriveLogic_Limit(
        (int32_t)forward_mm_s - differential_mm_s, max_wheel_mm_s);
    return target;
}

void DriveMotionProfile_Reset(DriveMotionProfile *profile)
{
    profile->speed_milli_mm_s = 0;
    profile->acceleration_mm_s2 = 0;
}

int16_t DriveMotionProfile_Update(DriveMotionProfile *profile,
                                  int16_t requested_mm_s,
                                  uint16_t maximum_acceleration_mm_s2,
                                  uint16_t maximum_jerk_mm_s3,
                                  uint16_t interval_ms)
{
    int32_t target_milli_mm_s = (int32_t)requested_mm_s * 1000;
    int32_t difference =
        target_milli_mm_s - profile->speed_milli_mm_s;
    int32_t direction;
    int32_t directed_acceleration;
    int32_t desired_acceleration;
    int32_t jerk_step;
    uint32_t absolute_difference;
    uint32_t acceleration_radicand;
    uint64_t wide_value;

    if (maximum_acceleration_mm_s2 == 0U ||
        maximum_jerk_mm_s3 == 0U || interval_ms == 0U)
    {
        profile->acceleration_mm_s2 = 0;
        return (int16_t)(profile->speed_milli_mm_s / 1000);
    }

    wide_value =
        ((uint64_t)maximum_jerk_mm_s3 * interval_ms) / 1000U;
    if (wide_value > maximum_acceleration_mm_s2)
    {
        wide_value = maximum_acceleration_mm_s2;
    }
    jerk_step = (int32_t)wide_value;
    if (jerk_step == 0)
    {
        jerk_step = 1;
    }
    absolute_difference = (uint32_t)(
        difference >= 0 ? difference : -difference);
    if (absolute_difference <=
            (uint32_t)jerk_step * interval_ms &&
        profile->acceleration_mm_s2 <= jerk_step &&
        profile->acceleration_mm_s2 >= -jerk_step)
    {
        profile->speed_milli_mm_s = target_milli_mm_s;
        profile->acceleration_mm_s2 = 0;
        return requested_mm_s;
    }

    direction = difference > 0 ? 1 : -1;
    directed_acceleration =
        direction * profile->acceleration_mm_s2;
    wide_value =
        ((uint64_t)2U * maximum_jerk_mm_s3 *
         absolute_difference) /
        1000U;
    acceleration_radicand =
        wide_value > UINT32_MAX ? UINT32_MAX : (uint32_t)wide_value;
    desired_acceleration = (int32_t)(
        (DriveLogic_IntegerSquareRoot(acceleration_radicand) * 9U) /
        10U);
    if (desired_acceleration >
        (int32_t)maximum_acceleration_mm_s2)
    {
        desired_acceleration = maximum_acceleration_mm_s2;
    }

    if (directed_acceleration <
        desired_acceleration - jerk_step)
    {
        directed_acceleration += jerk_step;
    }
    else if (directed_acceleration >
             desired_acceleration + jerk_step)
    {
        directed_acceleration -= jerk_step;
    }
    else
    {
        directed_acceleration = desired_acceleration;
    }

    profile->acceleration_mm_s2 =
        direction * directed_acceleration;
    profile->speed_milli_mm_s +=
        profile->acceleration_mm_s2 * interval_ms;

    if (profile->speed_milli_mm_s >= 0)
    {
        return (int16_t)(
            (profile->speed_milli_mm_s + 500) / 1000);
    }
    return (int16_t)(
        (profile->speed_milli_mm_s - 500) / 1000);
}

float DriveLogic_CountsToMillimeters(int32_t counts)
{
    return ((float)counts * DRIVE_WHEEL_CIRCLE_MM) /
           DRIVE_ENCODER_COUNTS_REV;
}

float DriveLogic_CountsToMmPerSecond(int32_t delta_counts,
                                     uint16_t interval_ms)
{
    if (interval_ms == 0U)
    {
        return 0.0f;
    }
    return (DriveLogic_CountsToMillimeters(delta_counts) * 1000.0f) /
           (float)interval_ms;
}
