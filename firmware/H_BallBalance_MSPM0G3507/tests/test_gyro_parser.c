#include <assert.h>
#include <stdint.h>

#include "gyro_parser.h"

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static GyroFrameType push_frame(GyroParser *parser, uint8_t id,
                                int16_t raw, uint32_t now_ms,
                                uint8_t checksum_adjust)
{
    uint8_t frame[5];
    uint8_t index;
    GyroFrameType result = GYRO_FRAME_NONE;

    frame[0] = 0x5AU;
    frame[1] = id;
    frame[2] = (uint8_t)raw;
    frame[3] = (uint8_t)((uint16_t)raw >> 8U);
    frame[4] = (uint8_t)(frame[0] + frame[1] +
                         frame[2] + frame[3] + checksum_adjust);
    for (index = 0U; index < 5U; ++index)
    {
        result = GyroParser_PushByte(parser, frame[index], now_ms);
    }
    return result;
}

static void test_valid_frames_and_freshness(void)
{
    GyroParser parser;
    GyroSnapshot snapshot;

    GyroParser_Init(&parser);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(snapshot.angle_ready == 0U);
    assert(snapshot.rate_ready == 0U);
    assert(snapshot.valid_frame_count == 0U);
    assert(GyroParser_IsAngleFresh(&parser, 500U, 0U) == 0U);
    assert(GyroParser_IsRateFresh(&parser, 500U, 0U) == 0U);

    assert(GyroParser_PushByte(&parser, 0x00U, 99U) ==
           GYRO_FRAME_NONE);
    assert(parser.format_error_count == 0U);

    assert(push_frame(&parser, 0xBBU, 0, 100U, 0U) ==
           GYRO_FRAME_ANGLE);
    assert(push_frame(&parser, 0xAAU, 16384, 110U, 0U) ==
           GYRO_FRAME_RATE);
    assert(push_frame(&parser, 0xBBU, 8192, 120U, 0U) ==
           GYRO_FRAME_ANGLE);

    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(snapshot.angle_ready == 1U);
    assert(snapshot.rate_ready == 1U);
    assert(snapshot.angle_sequence == 2U);
    assert(snapshot.rate_sequence == 1U);
    assert(snapshot.valid_frame_count == 3U);
    assert(snapshot.last_angle_ms == 120U);
    assert(snapshot.last_rate_ms == 110U);
    assert(absolute_float(snapshot.angle_deg - 45.0f) < 0.01f);
    assert(absolute_float(snapshot.rate_dps - 1000.0f) < 0.1f);
    assert(GyroParser_IsAngleFresh(&parser, 500U, 620U) == 1U);
    assert(GyroParser_IsAngleFresh(&parser, 500U, 621U) == 0U);
    assert(GyroParser_IsRateFresh(&parser, 20U, 130U) == 1U);
    assert(GyroParser_IsRateFresh(&parser, 20U, 131U) == 0U);
}

static void test_invalid_frames_and_resynchronization(void)
{
    GyroParser parser;
    GyroSnapshot snapshot;
    const uint8_t valid_tail[] = {0xBBU, 0x00U, 0x00U, 0x15U};
    uint8_t index;

    GyroParser_Init(&parser);
    assert(push_frame(&parser, 0xBBU, 8192, 200U, 1U) ==
           GYRO_FRAME_NONE);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(snapshot.angle_sequence == 0U);
    assert(snapshot.valid_frame_count == 0U);
    assert(snapshot.checksum_error_count == 1U);

    assert(GyroParser_PushByte(&parser, 0x5AU, 210U) ==
           GYRO_FRAME_NONE);
    assert(GyroParser_PushByte(&parser, 0x5AU, 210U) ==
           GYRO_FRAME_NONE);
    for (index = 0U; index < sizeof(valid_tail); ++index)
    {
        (void)GyroParser_PushByte(&parser, valid_tail[index], 210U);
    }
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(snapshot.angle_sequence == 1U);
    assert(snapshot.valid_frame_count == 1U);
    assert(snapshot.format_error_count == 1U);
    assert(snapshot.angle_ready == 1U);
}

static void test_zero_and_angle_wrap(void)
{
    GyroParser parser;
    GyroSnapshot snapshot;

    GyroParser_Init(&parser);
    GyroParser_ZeroAtCurrent(&parser);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(snapshot.zero_pending == 1U);

    (void)push_frame(&parser, 0xBBU, 5461, 300U, 0U);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(snapshot.zero_pending == 0U);
    assert(absolute_float(snapshot.angle_deg) < 0.01f);

    (void)push_frame(&parser, 0xBBU, 7282, 310U, 0U);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(absolute_float(snapshot.angle_deg - 10.0f) < 0.02f);

    GyroParser_ZeroOnNextAngle(&parser);
    (void)push_frame(&parser, 0xBBU, 8192, 320U, 0U);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(absolute_float(snapshot.angle_deg) < 0.01f);

    GyroParser_UsePowerOnZero(&parser);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(absolute_float(snapshot.angle_deg - 15.0f) < 0.02f);

    GyroParser_Init(&parser);
    (void)push_frame(&parser, 0xBBU, 30948, 400U, 0U);
    (void)push_frame(&parser, 0xBBU, -30948, 410U, 0U);
    GyroParser_GetSnapshot(&parser, &snapshot);
    assert(absolute_float(snapshot.angle_deg - 20.0f) < 0.02f);
}

static void test_timestamp_wraparound(void)
{
    GyroParser parser;

    GyroParser_Init(&parser);
    (void)push_frame(&parser, 0xBBU, 0, UINT32_MAX - 5U, 0U);
    assert(GyroParser_IsAngleFresh(&parser, 10U, 4U) == 1U);
    assert(GyroParser_IsAngleFresh(&parser, 10U, 5U) == 0U);
}

int main(void)
{
    test_valid_frames_and_freshness();
    test_invalid_frames_and_resynchronization();
    test_zero_and_angle_wrap();
    test_timestamp_wraparound();
    return 0;
}
