#ifndef TASK3_CONTROLLER_H
#define TASK3_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "vision_service.h"

#define TASK3_BALANCE_ANGLE_CDEG                          0
#define TASK3_MIN_ANGLE_CDEG                          -2865
#define TASK3_MAX_ANGLE_CDEG                           1432
#define TASK3_PD_KP_NUMERATOR                           250
#define TASK3_PD_KD_NUMERATOR                           200
#define TASK3_POSITIVE_PD_KP_NUMERATOR                  250
#define TASK3_POSITIVE_PD_KD_NUMERATOR                  500
#define TASK3_PD_GAIN_DENOMINATOR                       100
#define TASK3_STEPPER_SPEED_RPM                          20U
#define TASK3_STEPPER_ACCELERATION                      100U
#define TASK3_POSITIVE_DRIVE_ANGLE_CDEG                 -2050
#define TASK3_POSITIVE_BRAKE_ANGLE_CDEG                  400
#define TASK3_POSITIVE_HIGH_SPEED_BRAKE_ANGLE_CDEG       600
#define TASK3_POSITIVE_HIGH_SPEED_BRAKE_MIN_VELOCITY_TENTHS_MM_PER_S \
                                                            400
#define TASK3_POSITIVE_CONTROL_X_TENTHS_MM               490
#define TASK3_POSITIVE_STOP_LIMIT_X_TENTHS_MM            480
#define TASK3_POSITIVE_ACCEL_PER_DEG_TENTHS_MM_PER_S2     65
#define TASK3_POSITIVE_MIN_BRAKE_TENTHS_MM_PER_S2        500
#define TASK3_POSITIVE_PREDICTION_DELAY_MS               160U

#define TASK3_POSITIVE_X_TENTHS_MM                      500
#define TASK3_NEGATIVE_X_TENTHS_MM                     -500
#define TASK3_POSITIVE_APPROACH_PREDICTED_X_TENTHS_MM   100
#define TASK3_POSITIVE_BOOST_ENTER_VELOCITY_TENTHS_MM_PER_S \
                                                             50
#define TASK3_POSITIVE_BOOST_EXIT_VELOCITY_TENTHS_MM_PER_S \
                                                             100
#define TASK3_POSITIVE_BOOST_SAMPLE_COUNT                  3U
#define TASK3_POSITIVE_BOOST_MAX_X_TENTHS_MM              480
#define TASK3_POSITIVE_ARRIVAL_MIN_X_TENTHS_MM            465
#define TASK3_POSITIVE_ARRIVAL_MAX_X_TENTHS_MM            520
#define TASK3_NEGATIVE_REFERENCE_RATE_TENTHS_MM_PER_S      600
#define TASK3_NEGATIVE_REFERENCE_RAMP_UP_MS                 300U
#define TASK3_NEGATIVE_REFERENCE_DECEL_START_X_TENTHS_MM   -350
#define TASK3_NEGATIVE_REFERENCE_RAMP_DOWN_MS               500U
#define TASK3_NEGATIVE_FRICTION_COMPENSATION_CDEG           150
#define TASK3_NEGATIVE_FRICTION_MIN_LAG_TENTHS_MM            50
#define TASK3_NEGATIVE_FRICTION_RELEASE_LAG_TENTHS_MM        20
#define TASK3_NEGATIVE_FRICTION_STOP_X_TENTHS_MM           -490
#define TASK3_NEGATIVE_FRICTION_ENTER_MAX_VELOCITY_TENTHS_MM_PER_S \
                                                             100
#define TASK3_NEGATIVE_FINAL_FRICTION_ENTER_MAX_VELOCITY_TENTHS_MM_PER_S \
                                                             150
#define TASK3_NEGATIVE_FRICTION_EXIT_VELOCITY_TENTHS_MM_PER_S \
                                                            -250
#define TASK3_NEGATIVE_FRICTION_SAMPLE_COUNT                  3U
#define TASK3_NEGATIVE_FINAL_FRICTION_SAMPLE_COUNT            2U
#define TASK3_TARGET_MIN_X_TENTHS_MM                   -520
#define TASK3_TARGET_MAX_X_TENTHS_MM                   -470
#define TASK3_STABLE_MAX_VELOCITY_TENTHS_MM_PER_S       200
#define TASK3_PREDICTION_DELAY_MS                        120U
#define TASK3_STABLE_SAMPLE_COUNT                          5U
#define TASK3_VISION_ACQUIRE_TIMEOUT_MS                  500U

#define TASK3_TUNING_KP_MIN                                0
#define TASK3_TUNING_KP_MAX                             1000
#define TASK3_TUNING_KD_MIN                                0
#define TASK3_TUNING_KD_MAX                              500
#define TASK3_TUNING_APPROACH_X_MIN_TENTHS_MM             100
#define TASK3_TUNING_APPROACH_X_MAX_TENTHS_MM             450
#define TASK3_TUNING_PREDICTION_DELAY_MAX_MS              300U
#define TASK3_TUNING_BOOST_VELOCITY_MAX_TENTHS_MM_PER_S  1000

typedef struct
{
    int16_t kp_millidegrees_per_mm;
    int16_t kd_millidegrees_seconds_per_mm;
    int16_t positive_drive_angle_cdeg;
    int16_t positive_approach_x_tenths_mm;
    int16_t boost_enter_velocity_tenths_mm_per_s;
    int16_t boost_exit_velocity_tenths_mm_per_s;
    int16_t minimum_angle_cdeg;
    int16_t maximum_angle_cdeg;
    uint16_t prediction_delay_ms;
    uint16_t stepper_speed_rpm;
    uint8_t stepper_acceleration;
} Task3TuningConfig;

typedef struct
{
    int16_t x_tenths_mm;
    int32_t velocity_tenths_mm_per_s;
} Task3NegativeReferenceState;

typedef enum
{
    TASK3_CONTROLLER_UPDATE_NONE = 0,
    TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE,
    TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING,
    TASK3_CONTROLLER_UPDATE_DONE
} Task3ControllerUpdateResult;

typedef enum
{
    TASK3_CONTROLLER_PHASE_NOT_STARTED = 0,
    TASK3_CONTROLLER_PHASE_POSITIVE_DRIVE,
    TASK3_CONTROLLER_PHASE_POSITIVE_APPROACH,
    TASK3_CONTROLLER_PHASE_NEGATIVE_TRACK,
    TASK3_CONTROLLER_PHASE_LEVEL_TIMEOUT
} Task3ControllerPhase;

typedef struct
{
    bool positive_reached;
    bool negative_reached;
    uint32_t positive_from_start_ms;
    uint32_t negative_from_start_ms;
    uint32_t negative_from_reverse_ms;
    int32_t positive_velocity_tenths_mm_per_s;
    int32_t negative_velocity_tenths_mm_per_s;
} Task3ControllerTimingResult;

typedef struct
{
    VisionService vision;
    Task3ControllerTimingResult timing;
    uint32_t start_ms;
    uint32_t reverse_command_ms;
    uint32_t phase_started_ms;
    int16_t negative_reference_start_x_tenths_mm;
    Task3ControllerPhase phase;
    uint8_t stable_sample_count;
    uint8_t positive_low_speed_sample_count;
    uint8_t negative_friction_sample_count;
    int8_t negative_friction_direction;
    int16_t last_command_angle_cdeg;
    bool positive_boost_active;
    bool negative_friction_active;
    bool command_valid;
    bool active;
    Task3TuningConfig config;
} Task3Controller;

void Task3Controller_Init(Task3Controller *controller);
void Task3Controller_Start(
    Task3Controller *controller, uint32_t now_ms);
void Task3Controller_Cancel(Task3Controller *controller);
Task3ControllerUpdateResult Task3Controller_Update(
    Task3Controller *controller,
    uint32_t now_ms,
    const VisionSample *sample,
    int16_t *target_angle_centi_degrees);
bool Task3Controller_IsActive(
    const Task3Controller *controller);
void Task3Controller_GetTimingResult(
    const Task3Controller *controller,
    Task3ControllerTimingResult *result);
void Task3Controller_GetDefaultConfig(Task3TuningConfig *config);
void Task3Controller_GetConfig(
    const Task3Controller *controller, Task3TuningConfig *config);
bool Task3Controller_SetConfig(
    Task3Controller *controller, const Task3TuningConfig *config);
bool Task3Controller_GetVisionState(
    const Task3Controller *controller,
    uint32_t now_ms,
    VisionState *state);
int32_t Task3Controller_PredictX(
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s);
int32_t Task3Controller_PredictXWithConfig(
    const Task3TuningConfig *config,
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s);
int16_t Task3Controller_ComputeAngleCentiDegrees(
    int16_t target_x_tenths_mm,
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s);
int16_t Task3Controller_ComputeAngleWithConfig(
    const Task3TuningConfig *config,
    int16_t target_x_tenths_mm,
    int16_t x_tenths_mm,
    int32_t velocity_tenths_mm_per_s);
void Task3Controller_ComputeNegativeReference(
    int16_t start_x_tenths_mm,
    uint32_t elapsed_ms,
    Task3NegativeReferenceState *state);
int16_t Task3Controller_ComputeNegativeReferenceX(
    int16_t start_x_tenths_mm, uint32_t elapsed_ms);

#endif
