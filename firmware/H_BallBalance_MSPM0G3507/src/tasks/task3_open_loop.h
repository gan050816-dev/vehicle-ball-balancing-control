#ifndef TASK3_OPEN_LOOP_H
#define TASK3_OPEN_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 临时摩擦诊断固件：1表示把T3三个倾斜阶段全部反向，使小球先从O点
 * 向-X运动。该模式不满足正式T3“先+50 mm、再-50 mm”的顺序。
 */
#define TASK3_REVERSE_FRICTION_DIAGNOSTIC          0U

/*
 * 1表示启用当前预制动辨识测试。K230只触发+50 mm换向、返回
 * +60 mm预制动和-50 mm到达等一次性阶段事件，不进行连续PD控制。
 */
#define TASK3_TIMING_CALIBRATION                    1U
#define TASK3_TIMING_DRIVE_TARGET_TENTHS_MM         89U
#define TASK3_TIMING_POSITIVE_X_TENTHS_MM          500
#define TASK3_TIMING_NEGATIVE_X_TENTHS_MM         -500
#define TASK3_TIMING_BRAKE_TRIGGER_X_TENTHS_MM     600
#define TASK3_TIMING_BRAKE_TRIGGER_VELOCITY_TENTHS_MM_PER_S \
                                                    -200
#define TASK3_TIMING_BRAKE_TARGET_TENTHS_MM         49U
#define TASK3_TIMING_BRAKE_RELEASE_SAMPLES           2U

#define TASK3_RACK_TOWARD_HINGE_TENTHS_MM        199U
#define TASK3_RACK_AWAY_FROM_HINGE_TENTHS_MM      99U
#define TASK3_RACK_LEVEL_TENTHS_MM                149U
#define TASK3_STEPPER_SPEED_RPM                    20U
#define TASK3_STEPPER_ACCELERATION                100U

#define TASK3_FIRST_REVERSE_MS                    650U
#define TASK3_SECOND_REVERSE_MS                  2600U
#define TASK3_RETURN_LEVEL_MS                    3100U
#define TASK3_COMPLETE_MS                        4500U
#define TASK3_TIMEOUT_MS                         5000U

typedef enum
{
    TASK3_UPDATE_NONE = 0,
    TASK3_UPDATE_COMMAND_POSITION,
    TASK3_UPDATE_DONE
} Task3UpdateResult;

typedef enum
{
    TASK3_PHASE_NOT_STARTED = 0,
    TASK3_PHASE_FIRST_MOTION,
    TASK3_PHASE_REVERSE_MOTION,
    TASK3_PHASE_BRAKE_MOTION,
    TASK3_PHASE_LEVEL_HOLD
} Task3Phase;

typedef struct
{
    bool valid;
    uint8_t sequence;
    int16_t x_tenths_mm;
    uint32_t received_at_ms;
} Task3BallSample;

typedef struct
{
    bool positive_reached;
    bool negative_reached;
    uint32_t positive_from_start_ms;
    uint32_t negative_from_start_ms;
    uint32_t negative_from_reverse_ms;
    int32_t positive_velocity_tenths_mm_per_s;
    int32_t negative_velocity_tenths_mm_per_s;
} Task3TimingResult;

typedef struct
{
    uint32_t start_ms;
    uint32_t reverse_command_ms;
    uint32_t last_sample_received_at_ms;
    int16_t last_sample_x_tenths_mm;
    int32_t latest_velocity_tenths_mm_per_s;
    uint8_t last_sample_sequence;
    uint8_t brake_nonnegative_sample_count;
    bool sample_seen;
    Task3Phase phase;
    bool active;
    Task3TimingResult timing;
} Task3OpenLoop;

void Task3OpenLoop_Init(Task3OpenLoop *task);
void Task3OpenLoop_Start(Task3OpenLoop *task, uint32_t now_ms);
void Task3OpenLoop_Cancel(Task3OpenLoop *task);
Task3UpdateResult Task3OpenLoop_Update(
    Task3OpenLoop *task,
    uint32_t now_ms,
    const Task3BallSample *sample,
    uint16_t *target_tenths_mm);
bool Task3OpenLoop_IsActive(const Task3OpenLoop *task);
void Task3OpenLoop_GetTimingResult(
    const Task3OpenLoop *task,
    Task3TimingResult *result);

#endif
