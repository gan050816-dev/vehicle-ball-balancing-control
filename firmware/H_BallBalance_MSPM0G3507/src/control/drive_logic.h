#ifndef DRIVE_LOGIC_H
#define DRIVE_LOGIC_H

#include <stdint.h>

typedef struct
{
    int16_t left_mm_s;
    int16_t right_mm_s;
} DriveWheelTargets;

typedef struct
{
    int32_t speed_milli_mm_s;
    int32_t acceleration_mm_s2;
} DriveMotionProfile;

DriveWheelTargets DriveLogic_MixDifferential(int16_t forward_mm_s,
                                              int16_t differential_mm_s,
                                              int16_t max_wheel_mm_s);
void DriveMotionProfile_Reset(DriveMotionProfile *profile);
int16_t DriveMotionProfile_Update(DriveMotionProfile *profile,
                                  int16_t requested_mm_s,
                                  uint16_t maximum_acceleration_mm_s2,
                                  uint16_t maximum_jerk_mm_s3,
                                  uint16_t interval_ms);
float DriveLogic_CountsToMillimeters(int32_t counts);
float DriveLogic_CountsToMmPerSecond(int32_t delta_counts,
                                      uint16_t interval_ms);

#endif
