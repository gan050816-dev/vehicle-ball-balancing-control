#ifndef GYRO_PARSER_H
#define GYRO_PARSER_H

#include <stdint.h>

typedef enum
{
    GYRO_FRAME_NONE = 0,
    GYRO_FRAME_RATE,
    GYRO_FRAME_ANGLE
} GyroFrameType;

typedef struct
{
    uint8_t frame[5];
    uint8_t position;
    uint8_t angle_ready;
    uint8_t rate_ready;
    uint8_t offset_ready;
    uint8_t zero_on_next_angle;
    float raw_angle_deg;
    float rate_dps;
    float angle_offset_deg;
    float power_on_offset_deg;
    uint32_t last_angle_ms;
    uint32_t last_rate_ms;
    uint32_t angle_sequence;
    uint32_t rate_sequence;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t format_error_count;
} GyroParser;

typedef struct
{
    float angle_deg;
    float rate_dps;
    uint32_t last_angle_ms;
    uint32_t last_rate_ms;
    uint32_t angle_sequence;
    uint32_t rate_sequence;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t format_error_count;
    uint8_t angle_ready;
    uint8_t rate_ready;
    uint8_t zero_pending;
} GyroSnapshot;

void GyroParser_Init(GyroParser *parser);
GyroFrameType GyroParser_PushByte(GyroParser *parser, uint8_t byte,
                                  uint32_t now_ms);
uint8_t GyroParser_IsAngleFresh(const GyroParser *parser,
                                uint32_t max_age_ms, uint32_t now_ms);
uint8_t GyroParser_IsRateFresh(const GyroParser *parser,
                               uint32_t max_age_ms, uint32_t now_ms);
void GyroParser_ZeroAtCurrent(GyroParser *parser);
void GyroParser_ZeroOnNextAngle(GyroParser *parser);
void GyroParser_UsePowerOnZero(GyroParser *parser);
void GyroParser_GetSnapshot(const GyroParser *parser,
                            GyroSnapshot *snapshot);

#endif
