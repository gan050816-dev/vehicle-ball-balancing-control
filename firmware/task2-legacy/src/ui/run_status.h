#ifndef RUN_STATUS_H
#define RUN_STATUS_H

#include <stdint.h>

#include "task_menu.h"

#define RUN_STATUS_LINE_LENGTH 22U
#define RUN_STATUS_T4_DIAGNOSTIC 0U

typedef enum
{
    RUN_DISPLAY_SELECT = 0,
    RUN_DISPLAY_RUNNING,
    RUN_DISPLAY_STOPPED,
    RUN_DISPLAY_HOME,
    RUN_DISPLAY_LEVEL,
    RUN_DISPLAY_WAIT,
    RUN_DISPLAY_FAULT
} RunDisplayPhase;

typedef enum
{
    RUN_STOP_NONE = 0,
    RUN_STOP_TIME,
    RUN_STOP_LINE,
    RUN_STOP_STALL,
    RUN_STOP_FINISH
} RunStopReason;

typedef struct
{
    TaskMode mode;
    RunDisplayPhase phase;
    RunStopReason stop_reason;
    uint16_t maximum_acceleration_mm_s2;
    uint16_t peak_acceleration_mm_s2;
    int16_t peak_ball_x_tenths_mm;
    uint16_t current_speed_mm_s;
    uint32_t elapsed_centiseconds;
    uint8_t ball_valid;
    int16_t ball_x_tenths_mm;
    int16_t motor_angle_cdeg;
    uint16_t vision_diagnostic_code;
    uint8_t task4_fault_code;
    uint16_t task4_vision_fault_diagnostic_code;
    uint32_t task4_vision_fault_age_ms;
    uint32_t task4_vision_fault_checksum_errors;
    uint32_t task4_vision_fault_rx_overflows;
} LineDisplayStatus;

void LineDisplay_Format(const LineDisplayStatus *status,
                        char lines[4][RUN_STATUS_LINE_LENGTH]);

#endif
