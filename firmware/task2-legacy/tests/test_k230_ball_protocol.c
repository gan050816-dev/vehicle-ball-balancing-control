#include <assert.h>
#include <stdint.h>

#include "k230_ball_protocol.h"

static void make_state(uint8_t flags,
                       uint8_t sequence,
                       int16_t x_tenths_mm,
                       uint16_t confidence,
                       uint16_t timestamp,
                       uint8_t frame[K230_BALL_FRAME_SIZE])
{
    uint16_t raw_x = (uint16_t)x_tenths_mm;
    uint8_t index;

    frame[0] = K230_BALL_FRAME_HEAD1;
    frame[1] = K230_BALL_FRAME_HEAD2;
    frame[2] = K230_BALL_COMMAND_STATE;
    frame[3] = K230_BALL_PAYLOAD_SIZE;
    frame[4] = flags;
    frame[5] = sequence;
    frame[6] = (uint8_t)raw_x;
    frame[7] = (uint8_t)(raw_x >> 8U);
    frame[8] = (uint8_t)confidence;
    frame[9] = (uint8_t)(confidence >> 8U);
    frame[10] = (uint8_t)timestamp;
    frame[11] = (uint8_t)(timestamp >> 8U);
    frame[12] = (uint8_t)(frame[2] + frame[3]);
    for (index = 4U; index < 12U; ++index)
    {
        frame[12] = (uint8_t)(frame[12] + frame[index]);
    }
}

static void push(K230BallProtocol *protocol,
                 const uint8_t *bytes,
                 uint8_t length,
                 uint32_t now_ms)
{
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        K230BallProtocol_PushByte(protocol, bytes[index], now_ms);
    }
}

static void test_valid_signed_state_and_frozen_default_command(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];
    uint8_t command[K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE];
    static const uint8_t expected[] =
        {0xA5U, 0x5AU, 0x91U, 0x00U, 0x91U};
    uint8_t index;

    K230BallProtocol_Init(&protocol);
    make_state(K230_BALL_FLAG_VALID |
                   K230_BALL_FLAG_CALIBRATED |
                   K230_BALL_FLAG_TARGET_CENTER_VALID,
               42U, -500, 870U, 12345U, frame);
    push(&protocol, frame, sizeof(frame), 777U);

    assert(K230BallProtocol_TakeLatest(&protocol, &state));
    assert(state.valid);
    assert(state.calibrated);
    assert(state.target_center_valid);
    assert(!state.multiple_candidates);
    assert(state.sequence == 42U);
    assert(state.x_tenths_mm == -500);
    assert(state.confidence_permille == 870U);
    assert(state.k230_timestamp_ms16 == 12345U);
    assert(state.received_at_ms == 777U);
    assert(!K230BallProtocol_TakeLatest(&protocol, &state));

    assert(K230BallProtocol_MakeUseDefaultOrigin(command) ==
           K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE);
    for (index = 0U; index < sizeof(expected); ++index)
    {
        assert(command[index] == expected[index]);
    }
}

static void test_invalid_detection_and_malformed_frames(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_state(0U, 1U, 0, 0U, 10U, frame);
    push(&protocol, frame, sizeof(frame), 20U);
    assert(K230BallProtocol_TakeLatest(&protocol, &state));
    assert(!state.valid);

    make_state(0U, 2U, 10, 0U, 11U, frame);
    push(&protocol, frame, sizeof(frame), 21U);
    assert(!K230BallProtocol_TakeLatest(&protocol, &state));

    make_state(K230_BALL_FLAG_VALID, 3U, 10, 1001U, 12U, frame);
    push(&protocol, frame, sizeof(frame), 22U);
    assert(!K230BallProtocol_TakeLatest(&protocol, &state));

    make_state(K230_BALL_FLAG_VALID, 4U, 10, 900U, 13U, frame);
    frame[K230_BALL_FRAME_SIZE - 1U]++;
    push(&protocol, frame, sizeof(frame), 23U);
    assert(!K230BallProtocol_TakeLatest(&protocol, &state));
    assert(protocol.format_error_count == 2U);
    assert(protocol.checksum_error_count == 1U);
}

static void test_stream_resynchronizes(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];
    static const uint8_t noise[] = {0x00U, 0xA5U, 0xA5U};

    K230BallProtocol_Init(&protocol);
    make_state(K230_BALL_FLAG_VALID, 9U, 25, 650U, 99U, frame);
    push(&protocol, noise, sizeof(noise), 100U);
    push(&protocol, &frame[1], sizeof(frame) - 1U, 101U);
    assert(K230BallProtocol_TakeLatest(&protocol, &state));
    assert(state.sequence == 9U);
    assert(state.x_tenths_mm == 25);
}

int main(void)
{
    test_valid_signed_state_and_frozen_default_command();
    test_invalid_detection_and_malformed_frames();
    test_stream_resynchronizes();
    return 0;
}
