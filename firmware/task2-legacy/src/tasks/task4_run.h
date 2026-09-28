#ifndef TASK4_RUN_H
#define TASK4_RUN_H

#include <stdint.h>

#include "ball_pd.h"
#include "stepper_service.h"
#include "vision_service.h"

#define TASK4_CONTROL_PERIOD_MS       40U
#define TASK4_VISION_TIMEOUT_MS      120U
#define TASK4_TRACK_SPEED_RPM         20U
#define TASK4_TRACK_ACCELERATION     100U

typedef enum
{
    TASK4_RUN_IDLE = 0,
    TASK4_RUN_ACTIVE,
    TASK4_RUN_STOPPED,
    TASK4_RUN_FAULT
} Task4RunState;

typedef enum
{
    TASK4_FAULT_NONE = 0,
    TASK4_FAULT_START_NOT_READY,
    TASK4_FAULT_VISION_STALE,
    TASK4_FAULT_STEPPER,
    TASK4_FAULT_COMMAND_REJECTED
} Task4Fault;

typedef struct
{
    Task4RunState state;
    Task4Fault fault;
    BallPd controller;
    uint32_t last_control_ms;
    int16_t angle_command_cdeg;
    uint8_t vision_sequence_seen;
    uint8_t last_vision_sequence;
    uint32_t update_count;
} Task4Run;

void Task4Run_Init(Task4Run *task);
uint8_t Task4Run_Start(Task4Run *task,
                       StepperService *stepper,
                       const VisionState *vision,
                       uint32_t now_ms);
void Task4Run_Update(Task4Run *task,
                     StepperService *stepper,
                     const VisionState *vision,
                     int16_t forward_acceleration_mm_s2,
                     uint32_t now_ms);
uint8_t Task4Run_Stop(Task4Run *task,
                      StepperService *stepper);
void Task4Run_EmergencyStop(Task4Run *task,
                            StepperService *stepper);

#endif
