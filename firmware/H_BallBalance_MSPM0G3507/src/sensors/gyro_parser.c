#include "gyro_parser.h"

#define GYRO_FRAME_SIZE 5U
#define GYRO_HEADER     0x5AU
#define GYRO_RATE_ID    0xAAU
#define GYRO_ANGLE_ID   0xBBU
#define GYRO_RAW_SCALE  32768.0f
#define GYRO_RATE_RANGE 2000.0f
#define GYRO_ANGLE_RANGE 180.0f

static float GyroParser_WrapAngle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void GyroParser_RestartFromCandidate(GyroParser *parser,
                                            uint8_t candidate)
{
    parser->position = 0U;
    if (candidate == GYRO_HEADER)
    {
        parser->frame[0] = candidate;
        parser->position = 1U;
    }
}

static int32_t GyroParser_DecodeSigned16(uint8_t low, uint8_t high)
{
    uint32_t raw_bits = (uint32_t)low | ((uint32_t)high << 8U);

    return raw_bits >= 0x8000U ?
        (int32_t)raw_bits - 0x10000 : (int32_t)raw_bits;
}

void GyroParser_Init(GyroParser *parser)
{
    uint8_t index;

    for (index = 0U; index < GYRO_FRAME_SIZE; ++index)
    {
        parser->frame[index] = 0U;
    }
    parser->position = 0U;
    parser->angle_ready = 0U;
    parser->rate_ready = 0U;
    parser->offset_ready = 0U;
    parser->zero_on_next_angle = 0U;
    parser->raw_angle_deg = 0.0f;
    parser->rate_dps = 0.0f;
    parser->angle_offset_deg = 0.0f;
    parser->power_on_offset_deg = 0.0f;
    parser->last_angle_ms = 0U;
    parser->last_rate_ms = 0U;
    parser->angle_sequence = 0U;
    parser->rate_sequence = 0U;
    parser->valid_frame_count = 0U;
    parser->checksum_error_count = 0U;
    parser->format_error_count = 0U;
}

GyroFrameType GyroParser_PushByte(GyroParser *parser, uint8_t byte,
                                  uint32_t now_ms)
{
    uint8_t checksum;
    int32_t raw;

    if (parser->position == 0U)
    {
        if (byte == GYRO_HEADER)
        {
            parser->frame[0] = byte;
            parser->position = 1U;
        }
        return GYRO_FRAME_NONE;
    }

    if (parser->position == 1U &&
        byte != GYRO_RATE_ID && byte != GYRO_ANGLE_ID)
    {
        ++parser->format_error_count;
        GyroParser_RestartFromCandidate(parser, byte);
        return GYRO_FRAME_NONE;
    }

    parser->frame[parser->position] = byte;
    ++parser->position;
    if (parser->position < GYRO_FRAME_SIZE)
    {
        return GYRO_FRAME_NONE;
    }

    parser->position = 0U;
    checksum = (uint8_t)(parser->frame[0] + parser->frame[1] +
                         parser->frame[2] + parser->frame[3]);
    if (checksum != parser->frame[4])
    {
        ++parser->checksum_error_count;
        GyroParser_RestartFromCandidate(parser, parser->frame[4]);
        return GYRO_FRAME_NONE;
    }

    raw = GyroParser_DecodeSigned16(parser->frame[2], parser->frame[3]);
    ++parser->valid_frame_count;
    if (parser->frame[1] == GYRO_RATE_ID)
    {
        parser->rate_dps = ((float)raw / GYRO_RAW_SCALE) *
                           GYRO_RATE_RANGE;
        parser->last_rate_ms = now_ms;
        ++parser->rate_sequence;
        parser->rate_ready = 1U;
        return GYRO_FRAME_RATE;
    }

    parser->raw_angle_deg = ((float)raw / GYRO_RAW_SCALE) *
                            GYRO_ANGLE_RANGE;
    parser->last_angle_ms = now_ms;
    ++parser->angle_sequence;
    parser->angle_ready = 1U;
    if (parser->offset_ready == 0U)
    {
        parser->angle_offset_deg = parser->raw_angle_deg;
        parser->power_on_offset_deg = parser->raw_angle_deg;
        parser->offset_ready = 1U;
    }
    if (parser->zero_on_next_angle != 0U)
    {
        parser->angle_offset_deg = parser->raw_angle_deg;
        parser->zero_on_next_angle = 0U;
    }
    return GYRO_FRAME_ANGLE;
}

uint8_t GyroParser_IsAngleFresh(const GyroParser *parser,
                                uint32_t max_age_ms, uint32_t now_ms)
{
    return (parser->angle_ready != 0U &&
            (uint32_t)(now_ms - parser->last_angle_ms) <= max_age_ms) ?
        1U : 0U;
}

uint8_t GyroParser_IsRateFresh(const GyroParser *parser,
                               uint32_t max_age_ms, uint32_t now_ms)
{
    return (parser->rate_ready != 0U &&
            (uint32_t)(now_ms - parser->last_rate_ms) <= max_age_ms) ?
        1U : 0U;
}

void GyroParser_ZeroAtCurrent(GyroParser *parser)
{
    if (parser->angle_ready != 0U)
    {
        parser->angle_offset_deg = parser->raw_angle_deg;
        parser->offset_ready = 1U;
        parser->zero_on_next_angle = 0U;
    }
    else
    {
        parser->zero_on_next_angle = 1U;
    }
}

void GyroParser_ZeroOnNextAngle(GyroParser *parser)
{
    parser->zero_on_next_angle = 1U;
}

void GyroParser_UsePowerOnZero(GyroParser *parser)
{
    if (parser->offset_ready != 0U)
    {
        parser->angle_offset_deg = parser->power_on_offset_deg;
        parser->zero_on_next_angle = 0U;
    }
}

void GyroParser_GetSnapshot(const GyroParser *parser,
                            GyroSnapshot *snapshot)
{
    snapshot->angle_deg = GyroParser_WrapAngle(
        parser->raw_angle_deg - parser->angle_offset_deg);
    snapshot->rate_dps = parser->rate_dps;
    snapshot->last_angle_ms = parser->last_angle_ms;
    snapshot->last_rate_ms = parser->last_rate_ms;
    snapshot->angle_sequence = parser->angle_sequence;
    snapshot->rate_sequence = parser->rate_sequence;
    snapshot->valid_frame_count = parser->valid_frame_count;
    snapshot->checksum_error_count = parser->checksum_error_count;
    snapshot->format_error_count = parser->format_error_count;
    snapshot->angle_ready = parser->angle_ready;
    snapshot->rate_ready = parser->rate_ready;
    snapshot->zero_pending = parser->zero_on_next_angle;
}
