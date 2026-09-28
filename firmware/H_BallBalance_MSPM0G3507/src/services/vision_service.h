#ifndef VISION_SERVICE_H
#define VISION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_SAMPLE_PERIOD_MS                         40U
#define VISION_VELOCITY_WINDOW_SAMPLES                   2U
#define VISION_STALE_TIMEOUT_MS                         120U
#define VISION_MAX_ABS_VELOCITY_TENTHS_MM_PER_S        5000

typedef struct
{
    bool valid;
    uint8_t sequence;
    int16_t x_tenths_mm;
    uint32_t received_at_ms;
} VisionSample;

typedef struct
{
    bool valid;
    bool velocity_valid;
    int16_t x_filtered_tenths_mm;
    int32_t velocity_filtered_tenths_mm_per_s;
    uint32_t received_at_ms;
    uint8_t sequence;
} VisionState;

typedef struct
{
    int16_t x_filtered_tenths_mm;
    int16_t previous_raw_x_tenths_mm;
    int16_t two_samples_ago_raw_x_tenths_mm;
    int32_t velocity_filtered_tenths_mm_per_s;
    uint32_t last_valid_received_at_ms;
    uint32_t previous_raw_received_at_ms;
    uint32_t two_samples_ago_received_at_ms;
    uint32_t invalid_sample_count;
    uint32_t rejected_velocity_count;
    uint32_t duplicate_sample_count;
    uint8_t last_sequence;
    uint8_t accepted_sample_count;
    bool sequence_seen;
    bool state_valid;
    bool velocity_valid;
} VisionService;

void VisionService_Init(VisionService *service);
bool VisionService_PushSample(
    VisionService *service, const VisionSample *sample);
bool VisionService_GetState(
    const VisionService *service,
    uint32_t now_ms,
    VisionState *state);

#endif
