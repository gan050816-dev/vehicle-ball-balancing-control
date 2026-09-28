#ifndef BALL_PD_H
#define BALL_PD_H

#include <stdint.h>

#define BALL_PD_KP_NUMERATOR                   344
#define BALL_PD_KD_NUMERATOR                    57
#define BALL_PD_GAIN_DENOMINATOR               100
#define BALL_PD_ACCEL_FF_NUMERATOR             6717
#define BALL_PD_ACCEL_FF_DENOMINATOR           1000
#define BALL_PD_MIN_ANGLE_CDEG                -2865
#define BALL_PD_MAX_ANGLE_CDEG                 1432
#define BALL_PD_SLEW_LIMIT_CDEG                 430

typedef struct
{
    int16_t command_angle_cdeg;
    int16_t unsaturated_angle_cdeg;
    int16_t feedforward_angle_cdeg;
    uint8_t saturated;
} BallPd;

void BallPd_Init(BallPd *controller);
int16_t BallPd_Update(BallPd *controller,
                      int16_t predicted_x_tenths_mm,
                      int16_t velocity_tenths_mm_s,
                      int16_t forward_acceleration_mm_s2);

#endif
