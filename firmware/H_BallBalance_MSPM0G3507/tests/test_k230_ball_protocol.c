#include <assert.h>
#include <stdint.h>

#include "k230_ball_protocol.h"

static uint8_t make_frame(uint8_t flags,
                          uint8_t sequence,
                          int16_t x_tenths_mm,
                          uint16_t confidence_permille,
                          uint16_t timestamp_ms16,
                          uint8_t frame[K230_BALL_FRAME_SIZE])
{
    uint16_t raw_x = (uint16_t)x_tenths_mm;
    uint8_t checksum;
    uint8_t index;

    frame[0] = K230_BALL_FRAME_HEAD1;
    frame[1] = K230_BALL_FRAME_HEAD2;
    frame[2] = K230_BALL_COMMAND_STATE;
    frame[3] = K230_BALL_PAYLOAD_SIZE;
    frame[4] = flags;
    frame[5] = sequence;
    frame[6] = (uint8_t)raw_x;
    frame[7] = (uint8_t)(raw_x >> 8U);
    frame[8] = (uint8_t)confidence_permille;
    frame[9] = (uint8_t)(confidence_permille >> 8U);
    frame[10] = (uint8_t)timestamp_ms16;
    frame[11] = (uint8_t)(timestamp_ms16 >> 8U);

    checksum = (uint8_t)(frame[2] + frame[3]);
    for (index = 4U; index < 12U; ++index)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    frame[12] = checksum;
    return K230_BALL_FRAME_SIZE;
}

static void push_bytes(K230BallProtocol *protocol,
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

static uint8_t make_capture_result(
    uint8_t request_id,
    uint8_t status,
    uint16_t center_x_px,
    uint8_t frame[K230_BALL_CAPTURE_RESULT_FRAME_SIZE])
{
    frame[0] = K230_BALL_FRAME_HEAD1;
    frame[1] = K230_BALL_FRAME_HEAD2;
    frame[2] = K230_BALL_COMMAND_CAPTURE_RESULT;
    frame[3] = K230_BALL_CAPTURE_RESULT_PAYLOAD_SIZE;
    frame[4] = request_id;
    frame[5] = status;
    frame[6] = (uint8_t)center_x_px;
    frame[7] = (uint8_t)(center_x_px >> 8U);
    frame[8] = (uint8_t)(frame[2] + frame[3] + frame[4] +
                         frame[5] + frame[6] + frame[7]);
    return K230_BALL_CAPTURE_RESULT_FRAME_SIZE;
}

static void test_positive_position_decodes_complete_state(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_frame(K230_BALL_FLAG_VALID |
                   K230_BALL_FLAG_CALIBRATED |
                   K230_BALL_FLAG_TARGET_CENTER_VALID,
               42U, 500, 870U, 12345U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 777U);

    assert(K230BallProtocol_TakeLatest(&protocol, &state) != 0U);
    assert(state.valid != 0U);
    assert(state.calibrated != 0U);
    assert(state.multiple_candidates == 0U);
    assert(state.target_center_valid != 0U);
    assert(state.sequence == 42U);
    assert(state.x_tenths_mm == 500);
    assert(state.confidence_permille == 870U);
    assert(state.k230_timestamp_ms16 == 12345U);
    assert(state.received_at_ms == 777U);
    assert(protocol.valid_frame_count == 1U);
    assert(K230BallProtocol_TakeLatest(&protocol, &state) == 0U);
}

static void test_positive_position_matches_frozen_wire_bytes(void)
{
    uint8_t frame[K230_BALL_FRAME_SIZE];
    const uint8_t expected[K230_BALL_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x10U, 0x08U,
        0x07U, 0x2AU, 0xF4U, 0x01U,
        0x66U, 0x03U, 0x39U, 0x30U,
        0x10U
    };
    uint8_t index;

    make_frame(K230_BALL_FLAG_VALID |
                   K230_BALL_FLAG_CALIBRATED |
                   K230_BALL_FLAG_MULTI,
               42U, 500, 870U, 12345U, frame);
    for (index = 0U; index < K230_BALL_FRAME_SIZE; ++index)
    {
        assert(frame[index] == expected[index]);
    }
}

static void test_negative_position_uses_little_endian_twos_complement(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_frame(K230_BALL_FLAG_VALID | K230_BALL_FLAG_MULTI,
               43U, -500, 901U, 12400U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 800U);

    assert(K230BallProtocol_TakeLatest(&protocol, &state) != 0U);
    assert(state.x_tenths_mm == -500);
    assert(state.multiple_candidates != 0U);
}

static void test_invalid_detection_is_distinct_from_zero_position(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_frame(0U, 44U, 0, 0U, 12425U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 900U);

    assert(K230BallProtocol_TakeLatest(&protocol, &state) != 0U);
    assert(state.valid == 0U);
    assert(state.x_tenths_mm == 0);
    assert(state.confidence_permille == 0U);
}

static void test_bad_checksum_is_rejected(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_frame(K230_BALL_FLAG_VALID, 1U, 10, 700U, 20U, frame);
    frame[K230_BALL_FRAME_SIZE - 1U]++;
    push_bytes(&protocol, frame, sizeof(frame), 100U);

    assert(K230BallProtocol_TakeLatest(&protocol, &state) == 0U);
    assert(protocol.checksum_error_count == 1U);
}

static void test_invalid_payload_values_are_rejected(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_frame(0U, 2U, 100, 0U, 30U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 101U);
    assert(K230BallProtocol_TakeLatest(&protocol, &state) == 0U);

    make_frame(K230_BALL_FLAG_VALID, 3U, 100, 1001U, 31U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 102U);
    assert(K230BallProtocol_TakeLatest(&protocol, &state) == 0U);

    make_frame(K230_BALL_FLAG_VALID, 4U,
               K230_BALL_X_ABS_MAX_TENTHS_MM + 1,
               900U, 32U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 103U);
    assert(K230BallProtocol_TakeLatest(&protocol, &state) == 0U);
    assert(protocol.format_error_count == 3U);
}

static void test_noise_and_partial_header_resynchronize(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t frame[K230_BALL_FRAME_SIZE];
    const uint8_t noise[] = {0x00U, 0xA5U, 0xA5U};

    K230BallProtocol_Init(&protocol);
    make_frame(K230_BALL_FLAG_VALID, 9U, -25, 650U, 99U, frame);
    push_bytes(&protocol, noise, sizeof(noise), 200U);
    push_bytes(&protocol, &frame[1], sizeof(frame) - 1U, 201U);

    assert(K230BallProtocol_TakeLatest(&protocol, &state) != 0U);
    assert(state.sequence == 9U);
    assert(state.x_tenths_mm == -25);
}

static void test_truncated_frame_does_not_consume_next_frame_header(void)
{
    K230BallProtocol protocol;
    K230BallFrame state;
    uint8_t truncated[K230_BALL_FRAME_SIZE];
    uint8_t valid[K230_BALL_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_frame(K230_BALL_FLAG_VALID, 1U, 10, 700U, 10U, truncated);
    make_frame(K230_BALL_FLAG_VALID, 2U, 20, 800U, 20U, valid);

    push_bytes(&protocol, truncated, K230_BALL_FRAME_SIZE - 1U, 300U);
    push_bytes(&protocol, valid, K230_BALL_FRAME_SIZE, 301U);

    assert(protocol.checksum_error_count == 1U);
    assert(K230BallProtocol_TakeLatest(&protocol, &state) != 0U);
    assert(state.sequence == 2U);
    assert(state.x_tenths_mm == 20);
}

static void test_capture_request_matches_frozen_wire_bytes(void)
{
    uint8_t frame[K230_BALL_CAPTURE_REQUEST_FRAME_SIZE];
    const uint8_t expected[K230_BALL_CAPTURE_REQUEST_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x90U, 0x01U, 0x2AU, 0xBBU
    };
    uint8_t index;

    assert(K230BallProtocol_MakeCaptureRequest(0x2AU, frame) ==
           K230_BALL_CAPTURE_REQUEST_FRAME_SIZE);
    for (index = 0U;
         index < K230_BALL_CAPTURE_REQUEST_FRAME_SIZE;
         ++index)
    {
        assert(frame[index] == expected[index]);
    }
}

static void test_default_origin_request_matches_frozen_wire_bytes(void)
{
    uint8_t frame[K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE];
    const uint8_t expected[K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x91U, 0x00U, 0x91U
    };
    uint8_t index;

    assert(K230BallProtocol_MakeUseDefaultOrigin(frame) ==
           K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE);
    for (index = 0U;
         index < K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE;
         ++index)
    {
        assert(frame[index] == expected[index]);
    }
}

static void test_capture_result_decodes(void)
{
    K230BallProtocol protocol;
    K230BallCaptureResult result;
    uint8_t frame[K230_BALL_CAPTURE_RESULT_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_capture_result(7U, K230_BALL_CAPTURE_STATUS_OK, 638U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 555U);

    assert(K230BallProtocol_TakeCaptureResult(&protocol, &result) != 0U);
    assert(result.request_id == 7U);
    assert(result.status == K230_BALL_CAPTURE_STATUS_OK);
    assert(result.center_x_px == 638U);
    assert(result.received_at_ms == 555U);
    assert(protocol.valid_capture_result_count == 1U);
    assert(K230BallProtocol_TakeCaptureResult(&protocol, &result) == 0U);
}

static void test_capture_result_rejects_bad_status(void)
{
    K230BallProtocol protocol;
    K230BallCaptureResult result;
    uint8_t frame[K230_BALL_CAPTURE_RESULT_FRAME_SIZE];

    K230BallProtocol_Init(&protocol);
    make_capture_result(
        8U, K230_BALL_CAPTURE_STATUS_UNSTABLE + 1U, 0U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 600U);

    assert(K230BallProtocol_TakeCaptureResult(&protocol, &result) == 0U);
    assert(protocol.format_error_count == 1U);

    make_capture_result(
        9U, K230_BALL_CAPTURE_STATUS_TIMEOUT, 640U, frame);
    push_bytes(&protocol, frame, sizeof(frame), 601U);
    assert(K230BallProtocol_TakeCaptureResult(&protocol, &result) == 0U);
    assert(protocol.format_error_count == 2U);
}

int main(void)
{
    test_positive_position_decodes_complete_state();
    test_positive_position_matches_frozen_wire_bytes();
    test_negative_position_uses_little_endian_twos_complement();
    test_invalid_detection_is_distinct_from_zero_position();
    test_bad_checksum_is_rejected();
    test_invalid_payload_values_are_rejected();
    test_noise_and_partial_header_resynchronize();
    test_truncated_frame_does_not_consume_next_frame_header();
    test_capture_request_matches_frozen_wire_bytes();
    test_default_origin_request_matches_frozen_wire_bytes();
    test_capture_result_decodes();
    test_capture_result_rejects_bad_status();
    return 0;
}
