#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "drive_logic.h"

static uint16_t RunLapProfile(int16_t first_speed,
                              int16_t second_speed,
                              uint16_t switch_tick,
                              uint16_t maximum_acceleration,
                              uint16_t maximum_jerk,
                              int32_t *distance_at_8s_milli_mm,
                              int32_t *peak_acceleration)
{
    const int32_t track_length_milli_mm = 6141593;
    DriveMotionProfile profile;
    int32_t distance_milli_mm = 0;
    uint16_t tick = 0U;

    DriveMotionProfile_Reset(&profile);
    *distance_at_8s_milli_mm = 0;
    *peak_acceleration = 0;
    while (distance_milli_mm < track_length_milli_mm &&
           tick < 4000U)
    {
        int16_t requested =
            tick < switch_tick ? first_speed : second_speed;
        int16_t speed = DriveMotionProfile_Update(
            &profile, requested, maximum_acceleration,
            maximum_jerk, 10U);

        distance_milli_mm += (int32_t)speed * 10;
        if (abs(profile.acceleration_mm_s2) >
            *peak_acceleration)
        {
            *peak_acceleration =
                abs(profile.acceleration_mm_s2);
        }
        ++tick;
        if (tick == 800U)
        {
            *distance_at_8s_milli_mm = distance_milli_mm;
        }
    }
    return tick;
}

int main(void)
{
    DriveMotionProfile profile;
    int32_t previous_acceleration;
    int16_t speed;
    uint16_t tick;
    uint16_t lap_ticks;
    int32_t distance_at_8s_milli_mm;
    int32_t peak_acceleration;
    DriveWheelTargets targets =
        DriveLogic_MixDifferential(230, 120, 1000);
    assert(targets.left_mm_s == 350);
    assert(targets.right_mm_s == 110);

    targets = DriveLogic_MixDifferential(950, 120, 1000);
    assert(targets.left_mm_s == 1000);
    assert(targets.right_mm_s == 830);

    DriveMotionProfile_Reset(&profile);
    previous_acceleration = 0;
    speed = 0;
    for (tick = 0U;
         tick < 200U &&
         (profile.speed_milli_mm_s != 350000 ||
          profile.acceleration_mm_s2 != 0);
         ++tick)
    {
        speed = DriveMotionProfile_Update(
            &profile, 350, 800U, 2000U, 10U);
        assert(profile.acceleration_mm_s2 <= 800);
        assert(profile.acceleration_mm_s2 >= -800);
        assert(abs(profile.acceleration_mm_s2 -
                   previous_acceleration) <= 20);
        previous_acceleration = profile.acceleration_mm_s2;
    }
    assert(speed == 350);
    assert(profile.acceleration_mm_s2 == 0);

    previous_acceleration = 0;
    for (tick = 0U;
         tick < 200U &&
         (profile.speed_milli_mm_s != 0 ||
          profile.acceleration_mm_s2 != 0);
         ++tick)
    {
        speed = DriveMotionProfile_Update(
            &profile, 0, 800U, 2000U, 10U);
        assert(profile.acceleration_mm_s2 <= 800);
        assert(profile.acceleration_mm_s2 >= -800);
        assert(abs(profile.acceleration_mm_s2 -
                   previous_acceleration) <= 20);
        previous_acceleration = profile.acceleration_mm_s2;
    }
    assert(speed == 0);
    assert(profile.acceleration_mm_s2 == 0);

    lap_ticks = RunLapProfile(
        350, 350, 4000U, 800U, 2000U,
        &distance_at_8s_milli_mm, &peak_acceleration);
    assert(lap_ticks >= 1792U && lap_ticks <= 1802U);
    assert(peak_acceleration >= 780);
    assert(peak_acceleration <= 800);
    lap_ticks = RunLapProfile(
        208, 244, 800U, 300U, 800U,
        &distance_at_8s_milli_mm, &peak_acceleration);
    assert(distance_at_8s_milli_mm >= 1553000);
    assert(distance_at_8s_milli_mm <= 1555000);
    assert(lap_ticks >= 2683U && lap_ticks <= 2685U);
    assert(peak_acceleration == 300);

    assert(fabsf(DriveLogic_CountsToMillimeters(1456) - 204.20352f) <
           0.01f);
    assert(fabsf(DriveLogic_CountsToMmPerSecond(10, 0U)) < 0.01f);
    return 0;
}
