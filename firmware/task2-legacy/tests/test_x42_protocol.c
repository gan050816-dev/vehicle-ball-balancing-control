#include <assert.h>
#include <stdint.h>

#include "x42_protocol.h"

static void push(X42Protocol *protocol,
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

static void test_all_required_frame_types(void)
{
    static const uint8_t version[] =
        {0x02U, 0x1FU, 0x00U, 0xC8U, 0x03U, 0x14U, 0x6BU};
    static const uint8_t response[] =
        {0x02U, 0xFDU, 0x02U, 0x6BU};
    static const uint8_t status[] =
        {0x02U, 0x3AU, 0x03U, 0x6BU};
    static const uint8_t position[] = {
        0x02U, 0x36U, 0x00U, 0x00U,
        0x00U, 0x01U, 0x7BU, 0x6BU
    };
    X42Protocol protocol;
    X42Frame frame;

    X42Protocol_Init(&protocol);
    push(&protocol, version, sizeof(version), 10U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_VERSION);
    assert(frame.firmware_version == 200U);

    push(&protocol, response, sizeof(response), 20U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_CONTROL_RESPONSE);
    assert(frame.function == 0xFDU);
    assert(frame.response == X42_RESPONSE_RECEIVED);

    push(&protocol, status, sizeof(status), 30U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_STATUS);
    assert((frame.status_flags & X42_STATUS_ENABLED) != 0U);
    assert((frame.status_flags &
            X42_STATUS_POSITION_REACHED) != 0U);

    push(&protocol, position, sizeof(position), 40U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.type == X42_FRAME_REALTIME_POSITION);
    assert(frame.magnitude == 379U);
    assert(frame.received_at_ms == 40U);
}

static void test_malformed_stream_resynchronizes(void)
{
    static const uint8_t malformed[] =
        {0x02U, 0xFDU, 0x02U, 0x00U};
    static const uint8_t response[] =
        {0x02U, 0xFDU, 0x02U, 0x6BU};
    X42Protocol protocol;
    X42Frame frame;

    X42Protocol_Init(&protocol);
    push(&protocol, malformed, sizeof(malformed), 1U);
    assert(protocol.format_error_count == 1U);
    assert(!X42Protocol_TakeFrame(&protocol, &frame));

    push(&protocol, response, sizeof(response), 2U);
    assert(X42Protocol_TakeFrame(&protocol, &frame));
    assert(frame.response == X42_RESPONSE_RECEIVED);
}

int main(void)
{
    test_all_required_frame_types();
    test_malformed_stream_resynchronizes();
    return 0;
}
