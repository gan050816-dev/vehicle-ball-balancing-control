#include "x42_protocol.h"

#include <string.h>

static uint8_t X42Protocol_IsControlFunction(uint8_t function)
{
    switch (function)
    {
        case 0x0AU:
        case 0xF3U:
        case 0xFDU:
        case 0xFEU:
            return 1U;

        default:
            return 0U;
    }
}

static uint8_t X42Protocol_IsResponse(uint8_t response)
{
    switch (response)
    {
        case X42_RESPONSE_RECEIVED:
        case X42_RESPONSE_HOME_ALREADY_ZERO:
        case X42_RESPONSE_LIMIT_ACTIVE:
        case X42_RESPONSE_PARAMETER_ERROR:
        case X42_RESPONSE_FORMAT_ERROR:
        case X42_RESPONSE_ACTION_COMPLETE:
            return 1U;

        default:
            return 0U;
    }
}

static uint8_t X42Protocol_GetExpectedLength(uint8_t function)
{
    switch (function)
    {
        case X42_FUNCTION_VERSION:
            return 7U;

        case X42_FUNCTION_REALTIME_POSITION:
        case X42_FUNCTION_POSITION_ERROR:
            return 8U;

        case X42_FUNCTION_STATUS:
            return 4U;

        default:
            return X42Protocol_IsControlFunction(function) != 0U
                       ? 4U
                       : 0U;
    }
}

static uint32_t X42Protocol_ReadU32Be(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 24U) |
           ((uint32_t)bytes[1] << 16U) |
           ((uint32_t)bytes[2] << 8U) |
           (uint32_t)bytes[3];
}

static void X42Protocol_ResetParser(X42Protocol *protocol)
{
    protocol->position = 0U;
    protocol->expected_length = 0U;
}

static void X42Protocol_StartOrReset(
    X42Protocol *protocol, uint8_t byte)
{
    X42Protocol_ResetParser(protocol);
    if (byte == X42_PROTOCOL_MOTOR_ADDRESS)
    {
        protocol->buffer[0] = byte;
        protocol->position = 1U;
    }
}

static uint8_t X42Protocol_Decode(
    X42Protocol *protocol, uint32_t now_ms)
{
    X42Frame frame;
    uint8_t function = protocol->buffer[1];

    memset(&frame, 0, sizeof(frame));
    frame.function = function;
    frame.received_at_ms = now_ms;

    if (X42Protocol_IsControlFunction(function) != 0U)
    {
        if (X42Protocol_IsResponse(protocol->buffer[2]) == 0U)
        {
            return 0U;
        }
        frame.type = X42_FRAME_CONTROL_RESPONSE;
        frame.response = protocol->buffer[2];
    }
    else if (function == X42_FUNCTION_VERSION)
    {
        frame.type = X42_FRAME_VERSION;
        frame.firmware_version =
            (uint16_t)(((uint16_t)protocol->buffer[2] << 8U) |
                       protocol->buffer[3]);
        frame.hardware_series =
            (uint8_t)(protocol->buffer[4] >> 4U);
        frame.hardware_type =
            (uint8_t)(protocol->buffer[4] & 0x0FU);
        frame.hardware_version = protocol->buffer[5];
    }
    else if (function == X42_FUNCTION_STATUS)
    {
        frame.type = X42_FRAME_STATUS;
        frame.status_flags = protocol->buffer[2];
    }
    else if (function == X42_FUNCTION_REALTIME_POSITION ||
             function == X42_FUNCTION_POSITION_ERROR)
    {
        if (protocol->buffer[2] > 1U)
        {
            return 0U;
        }
        frame.type =
            function == X42_FUNCTION_REALTIME_POSITION
                ? X42_FRAME_REALTIME_POSITION
                : X42_FRAME_POSITION_ERROR;
        frame.negative = protocol->buffer[2];
        frame.magnitude =
            X42Protocol_ReadU32Be(&protocol->buffer[3]);
    }
    else
    {
        return 0U;
    }

    protocol->latest = frame;
    protocol->frame_ready = 1U;
    protocol->valid_frame_count++;
    return 1U;
}

void X42Protocol_Init(X42Protocol *protocol)
{
    memset(protocol, 0, sizeof(*protocol));
}

void X42Protocol_PushByte(
    X42Protocol *protocol, uint8_t byte, uint32_t now_ms)
{
    if (protocol->position == 0U)
    {
        if (byte == X42_PROTOCOL_MOTOR_ADDRESS)
        {
            protocol->buffer[0] = byte;
            protocol->position = 1U;
        }
        return;
    }

    if (protocol->position == 1U)
    {
        uint8_t expected_length =
            X42Protocol_GetExpectedLength(byte);

        if (expected_length == 0U)
        {
            protocol->format_error_count++;
            X42Protocol_StartOrReset(protocol, byte);
            return;
        }
        protocol->buffer[1] = byte;
        protocol->position = 2U;
        protocol->expected_length = expected_length;
        return;
    }

    protocol->buffer[protocol->position++] = byte;
    if (protocol->position < protocol->expected_length)
    {
        return;
    }

    if (protocol->buffer[protocol->expected_length - 1U] !=
            X42_PROTOCOL_FRAME_END ||
        X42Protocol_Decode(protocol, now_ms) == 0U)
    {
        protocol->format_error_count++;
    }
    X42Protocol_StartOrReset(protocol, byte);
}

uint8_t X42Protocol_TakeFrame(
    X42Protocol *protocol, X42Frame *frame)
{
    if (protocol->frame_ready == 0U)
    {
        return 0U;
    }

    if (frame != 0)
    {
        *frame = protocol->latest;
    }
    protocol->frame_ready = 0U;
    return 1U;
}
