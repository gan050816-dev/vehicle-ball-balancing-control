#ifndef VISION_SERVICE_H
#define VISION_SERVICE_H

#include <stdint.h>

#include "k230_ball_protocol.h"

#define VISION_MIN_CONFIDENCE_PERMILLE 650U
#define VISION_DEFAULT_TIMEOUT_MS       120U
#define VISION_PREDICTION_DELAY_MS      120U
#define VISION_MAX_SPEED_TENTHS_MM_S   5000

#define VISION_DIAG_NO_FRAME              1U
#define VISION_DIAG_BALL_INVALID          2U
#define VISION_DIAG_NOT_CALIBRATED        4U
#define VISION_DIAG_TARGET_INVALID        8U
#define VISION_DIAG_MULTIPLE             16U
#define VISION_DIAG_LOW_CONFIDENCE       32U
#define VISION_DIAG_DUPLICATE_SEQUENCE   64U
#define VISION_DIAG_TIMESTAMP_FREEZE    128U
#define VISION_DIAG_POSITION_OUTLIER    256U

typedef struct
{
    uint8_t valid;
    uint8_t initialized;
    uint8_t sequence_seen;
    uint8_t timestamp_seen;
    uint8_t last_sequence;
    uint8_t last_accepted_sequence;
    uint16_t last_k230_timestamp_ms16;
    uint16_t previous_k230_timestamp_ms16;
    int16_t raw_x_tenths_mm;
    int16_t previous_raw_x_tenths_mm;
    int16_t filtered_x_tenths_mm;
    int16_t filtered_velocity_tenths_mm_s;
    uint32_t last_frame_ms;
    uint32_t previous_frame_ms;
    uint8_t accepted_sample_count;
    uint32_t accepted_frame_count;
    uint32_t rejected_frame_count;
    uint32_t duplicate_frame_count;
    uint32_t timestamp_freeze_count;
    uint32_t outlier_frame_count;
    uint16_t diagnostic_code;
} VisionService;

typedef struct
{
    uint8_t valid;
    int16_t x_tenths_mm;
    int16_t velocity_tenths_mm_s;
    int16_t predicted_x_tenths_mm;
    uint32_t age_ms;
    uint8_t sequence;
} VisionState;

void VisionService_Init(VisionService *service);
uint8_t VisionService_PushFrame(VisionService *service,
                                const K230BallFrame *frame);
VisionState VisionService_GetState(const VisionService *service,
                                   uint32_t now_ms,
                                   uint32_t timeout_ms);
uint16_t VisionService_GetDiagnosticCode(
    const VisionService *service);

#endif
