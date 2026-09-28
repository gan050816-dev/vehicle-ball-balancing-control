#include <assert.h>
#include <stdint.h>

#include "x42_protocol.h"

static void push_frame(
    X42Protocol *protocol,
    const uint8_t *bytes,
    uint8_t length,
    uint32_t now_ms)
{
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        X42Protocol_PushByte(protocol, bytes[index], now_ms);
    }
}

static void test_control_response_and_stream_resynchronization(void)
{
    static const uint8_t response[] =
        {0x02U, 0xFDU, 0x02U, 0x6BU};
    static const uint8_t malformed[] =
        {0x02U, 0xFDU, 0x02U, 0x00U};
    X42Protocol protocol;
    X42Frame frame;

    X42Protocol_Init(&protocol);
    X42Protocol_PushByte(&protocol, 0x99U, 1U);
    push_frame(&protocol, malformed, sizeof(malformed), 2U);
    assert(protocol.format_error_count == 1U);
    assert(!X42Protocol_TakeFrame(&protocol, &frame));

    push_frame(&protocol, response, 2U, 3U);
    assert(!X42Protocol_TakeFrame(&protocol, &frame));
    push_frame(
        &protocol, &response[2], sizeof(response) - 2U, 4U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_CONTROL_RESPONSE);
    assert(frame.function == 0xFDU);
    assert(frame.response == X42_RESPONSE_RECEIVED);
    assert(frame.received_at_ms == 4U);
    assert(protocol.valid_frame_count == 1U);
}

static void test_version_status_position_and_error(void)
{
    static const uint8_t version[] =
        {0x02U, 0x1FU, 0x00U, 0xC8U, 0x03U, 0x14U, 0x6BU};
    static const uint8_t status[] =
        {0x02U, 0x3AU, 0x0FU, 0x6BU};
    static const uint8_t position[] = {
        0x02U, 0x36U, 0x00U, 0x00U,
        0x00U, 0x1EU, 0x5AU, 0x6BU
    };
    static const uint8_t error[] = {
        0x02U, 0x37U, 0x01U, 0x00U,
        0x00U, 0x00U, 0x08U, 0x6BU
    };
    X42Protocol protocol;
    X42Frame frame;

    X42Protocol_Init(&protocol);

    push_frame(&protocol, version, sizeof(version), 10U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_VERSION);
    assert(frame.firmware_version == 200U);
    assert(frame.hardware_series == 0U);
    assert(frame.hardware_type == 3U);
    assert(frame.hardware_version == 0x14U);

    push_frame(&protocol, status, sizeof(status), 20U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_STATUS);
    assert((frame.status_flags & X42_STATUS_ENABLED) != 0U);
    assert((frame.status_flags &
            X42_STATUS_POSITION_REACHED) != 0U);
    assert((frame.status_flags & X42_STATUS_STALL) != 0U);
    assert((frame.status_flags &
            X42_STATUS_STALL_PROTECTION) != 0U);

    push_frame(&protocol, position, sizeof(position), 30U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_REALTIME_POSITION);
    assert(frame.negative == 0U);
    assert(frame.magnitude == 7770U);

    push_frame(&protocol, error, sizeof(error), 40U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_POSITION_ERROR);
    assert(frame.negative == 1U);
    assert(frame.magnitude == 8U);
    assert(protocol.valid_frame_count == 4U);
}

static void test_rejects_unknown_function_and_invalid_sign(void)
{
    static const uint8_t invalid_sign[] = {
        0x02U, 0x36U, 0x02U, 0x00U,
        0x00U, 0x00U, 0x01U, 0x6BU
    };
    static const uint8_t valid_status[] =
        {0x02U, 0x3AU, 0x03U, 0x6BU};
    X42Protocol protocol;
    X42Frame frame;

    X42Protocol_Init(&protocol);
    X42Protocol_PushByte(&protocol, 0x02U, 0U);
    X42Protocol_PushByte(&protocol, 0x99U, 0U);
    assert(protocol.format_error_count == 1U);

    push_frame(
        &protocol, invalid_sign, sizeof(invalid_sign), 1U);
    assert(protocol.format_error_count == 2U);
    assert(!X42Protocol_TakeFrame(&protocol, &frame));

    push_frame(
        &protocol, valid_status, sizeof(valid_status), 2U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_STATUS);
}

int main(void)
{
    test_control_response_and_stream_resynchronization();
    test_version_status_position_and_error();
    test_rejects_unknown_function_and_invalid_sign();
    return 0;
}
