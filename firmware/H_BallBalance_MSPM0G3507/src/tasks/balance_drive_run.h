#ifndef BALANCE_DRIVE_RUN_H
#define BALANCE_DRIVE_RUN_H

#include <stdint.h>

#include "balance_vision_service.h"
#include "ball_pd.h"
#include "stepper_service.h"

#define BALANCE_DRIVE_CONTROL_PERIOD_MS       40U
#define BALANCE_DRIVE_VISION_TIMEOUT_MS      120U
#define BALANCE_DRIVE_TRACK_SPEED_RPM         20U
#define BALANCE_DRIVE_TRACK_ACCELERATION     100U
#define BALANCE_DRIVE_STARTUP_PREVIEW_MS       80U
#define BALANCE_DRIVE_STARTUP_MAX_ACCEL_MM_S2 300
#define BALANCE_DRIVE_STARTUP_MAX_EXTRA_MM_S2  64

typedef enum
{
    BALANCE_DRIVE_IDLE = 0,
    BALANCE_DRIVE_ACTIVE,
    BALANCE_DRIVE_STOPPED,
    BALANCE_DRIVE_FAULT
} BalanceDriveRunState;

typedef enum
{
    BALANCE_DRIVE_FAULT_NONE = 0,
    BALANCE_DRIVE_FAULT_START_NOT_READY,
    BALANCE_DRIVE_FAULT_VISION_STALE,
    BALANCE_DRIVE_FAULT_STEPPER,
    BALANCE_DRIVE_FAULT_COMMAND_REJECTED
} BalanceDriveFault;

typedef struct
{
    BalanceDriveRunState state;
    BalanceDriveFault fault;
    BallPd controller;
    uint32_t last_control_ms;
    uint32_t previous_acceleration_ms;
    int16_t angle_command_cdeg;
    int16_t previous_forward_acceleration_mm_s2;
    int16_t feedforward_acceleration_mm_s2;
    int16_t maximum_positive_x_tenths_mm;
    int16_t maximum_negative_x_tenths_mm;
    uint8_t vision_sequence_seen;
    uint8_t last_vision_sequence;
    uint8_t startup_preview_active;
    uint8_t startup_acceleration_seen;
    uint32_t update_count;
} BalanceDriveRun;

void BalanceDriveRun_Init(BalanceDriveRun *run);
uint8_t BalanceDriveRun_Start(
    BalanceDriveRun *run,
    StepperService *stepper,
    const BalanceVisionState *vision,
    uint32_t now_ms);
void BalanceDriveRun_Update(
    BalanceDriveRun *run,
    StepperService *stepper,
    const BalanceVisionState *vision,
    int16_t forward_acceleration_mm_s2,
    uint32_t now_ms);
uint8_t BalanceDriveRun_Stop(
    BalanceDriveRun *run,
    StepperService *stepper);
void BalanceDriveRun_EmergencyStop(
    BalanceDriveRun *run,
    StepperService *stepper);
void BalanceDriveRun_ReportVisionFault(BalanceDriveRun *run);

#endif
