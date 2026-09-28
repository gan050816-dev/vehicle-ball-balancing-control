#include "k230_ball_protocol.h"

#include <string.h>

typedef enum
{
    K230_BALL_PARSE_HEAD1 = 0,
    K230_BALL_PARSE_HEAD2,
    K230_BALL_PARSE_COMMAND,
    K230_BALL_PARSE_LENGTH,
    K230_BALL_PARSE_PAYLOAD,
    K230_BALL_PARSE_CHECKSUM
} K230BallParseState;

static void K230BallProtocol_ResetParser(K230BallProtocol *protocol)
{
    protocol->parse_state = K230_BALL_PARSE_HEAD1;
    protocol->command = 0U;
    protocol->length = 0U;
    protocol->payload_position = 0U;
}

static uint8_t K230BallProtocol_Checksum(const K230BallProtocol *protocol)
{
    uint8_t checksum =
        (uint8_t)(protocol->command + protocol->length);
    uint8_t index;

    for (index = 0U; index < protocol->length; ++index)
    {
        checksum = (uint8_t)(checksum + protocol->payload[index]);
    }
    return checksum;
}

static uint16_t K230BallProtocol_ReadU16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] |
                      ((uint16_t)bytes[1] << 8U));
}

static uint8_t K230BallProtocol_DecodeState(K230BallProtocol *protocol,
                                           uint32_t now_ms)
{
    uint8_t flags = protocol->payload[0];
    int16_t x_tenths_mm =
        (int16_t)K230BallProtocol_ReadU16(&protocol->payload[2]);
    uint16_t confidence_permille =
        K230BallProtocol_ReadU16(&protocol->payload[4]);
    uint8_t valid = (uint8_t)(flags & K230_BALL_FLAG_VALID);

    if ((flags & (uint8_t)~K230_BALL_FLAG_MASK) != 0U ||
        confidence_permille > 1000U ||
        (valid != 0U &&
         (x_tenths_mm < -K230_BALL_X_ABS_MAX_TENTHS_MM ||
          x_tenths_mm > K230_BALL_X_ABS_MAX_TENTHS_MM)) ||
        (valid == 0U &&
         (x_tenths_mm != 0 || confidence_permille != 0U)))
    {
        protocol->format_error_count++;
        return 0U;
    }

    protocol->latest.valid = valid != 0U ? 1U : 0U;
    protocol->latest.calibrated =
        (flags & K230_BALL_FLAG_CALIBRATED) != 0U ? 1U : 0U;
    protocol->latest.multiple_candidates =
        (flags & K230_BALL_FLAG_MULTI) != 0U ? 1U : 0U;
    protocol->latest.target_center_valid =
        (flags & K230_BALL_FLAG_TARGET_CENTER_VALID) != 0U ? 1U : 0U;
    protocol->latest.sequence = protocol->payload[1];
    protocol->latest.x_tenths_mm = x_tenths_mm;
    protocol->latest.confidence_permille = confidence_permille;
    protocol->latest.k230_timestamp_ms16 =
        K230BallProtocol_ReadU16(&protocol->payload[6]);
    protocol->latest.received_at_ms = now_ms;
    protocol->latest_ready = 1U;
    protocol->valid_frame_count++;
    return 1U;
}

static uint8_t K230BallProtocol_DecodeCaptureResult(
    K230BallProtocol *protocol, uint32_t now_ms)
{
    uint8_t status = protocol->payload[1];
    uint16_t center_x_px =
        K230BallProtocol_ReadU16(&protocol->payload[2]);

    if (status > K230_BALL_CAPTURE_STATUS_UNSTABLE ||
        (status != K230_BALL_CAPTURE_STATUS_OK && center_x_px != 0U))
    {
        protocol->format_error_count++;
        return 0U;
    }

    protocol->capture_result.request_id = protocol->payload[0];
    protocol->capture_result.status = status;
    protocol->capture_result.center_x_px = center_x_px;
    protocol->capture_result.received_at_ms = now_ms;
    protocol->capture_result_ready = 1U;
    protocol->valid_capture_result_count++;
    return 1U;
}

static uint8_t K230BallProtocol_Decode(K230BallProtocol *protocol,
                                      uint32_t now_ms)
{
    if (protocol->command == K230_BALL_COMMAND_STATE &&
        protocol->length == K230_BALL_PAYLOAD_SIZE)
    {
        return K230BallProtocol_DecodeState(protocol, now_ms);
    }
    if (protocol->command == K230_BALL_COMMAND_CAPTURE_RESULT &&
        protocol->length == K230_BALL_CAPTURE_RESULT_PAYLOAD_SIZE)
    {
        return K230BallProtocol_DecodeCaptureResult(protocol, now_ms);
    }

    protocol->format_error_count++;
    return 0U;
}

void K230BallProtocol_Init(K230BallProtocol *protocol)
{
    memset(protocol, 0, sizeof(*protocol));
    K230BallProtocol_ResetParser(protocol);
}

void K230BallProtocol_PushByte(K230BallProtocol *protocol,
                               uint8_t byte,
                               uint32_t now_ms)
{
    switch ((K230BallParseState)protocol->parse_state)
    {
        case K230_BALL_PARSE_HEAD1:
            if (byte == K230_BALL_FRAME_HEAD1)
            {
                protocol->parse_state = K230_BALL_PARSE_HEAD2;
            }
            break;

        case K230_BALL_PARSE_HEAD2:
            if (byte == K230_BALL_FRAME_HEAD2)
            {
                protocol->parse_state = K230_BALL_PARSE_COMMAND;
            }
            else if (byte != K230_BALL_FRAME_HEAD1)
            {
                K230BallProtocol_ResetParser(protocol);
            }
            break;

        case K230_BALL_PARSE_COMMAND:
            protocol->command = byte;
            protocol->parse_state = K230_BALL_PARSE_LENGTH;
            break;

        case K230_BALL_PARSE_LENGTH:
            protocol->length = byte;
            protocol->payload_position = 0U;
            if (byte == 0U || byte > K230_BALL_PAYLOAD_SIZE)
            {
                protocol->format_error_count++;
                K230BallProtocol_ResetParser(protocol);
            }
            else
            {
                protocol->parse_state = K230_BALL_PARSE_PAYLOAD;
            }
            break;

        case K230_BALL_PARSE_PAYLOAD:
            protocol->payload[protocol->payload_position++] = byte;
            if (protocol->payload_position >= protocol->length)
            {
                protocol->parse_state = K230_BALL_PARSE_CHECKSUM;
            }
            break;

        case K230_BALL_PARSE_CHECKSUM:
            if (byte == K230BallProtocol_Checksum(protocol))
            {
                (void)K230BallProtocol_Decode(protocol, now_ms);
            }
            else
            {
                protocol->checksum_error_count++;
                K230BallProtocol_ResetParser(protocol);
                if (byte == K230_BALL_FRAME_HEAD1)
                {
                    protocol->parse_state = K230_BALL_PARSE_HEAD2;
                }
                break;
            }
            K230BallProtocol_ResetParser(protocol);
            break;

        default:
            K230BallProtocol_ResetParser(protocol);
            break;
    }
}

uint8_t K230BallProtocol_TakeLatest(K230BallProtocol *protocol,
                                   K230BallFrame *frame)
{
    if (protocol->latest_ready == 0U)
    {
        return 0U;
    }

    if (frame != 0)
    {
        *frame = protocol->latest;
    }
    protocol->latest_ready = 0U;
    return 1U;
}

uint8_t K230BallProtocol_TakeCaptureResult(
    K230BallProtocol *protocol,
    K230BallCaptureResult *result)
{
    if (protocol->capture_result_ready == 0U)
    {
        return 0U;
    }

    if (result != 0)
    {
        *result = protocol->capture_result;
    }
    protocol->capture_result_ready = 0U;
    return 1U;
}

uint8_t K230BallProtocol_MakeCaptureRequest(
    uint8_t request_id,
    uint8_t frame[K230_BALL_CAPTURE_REQUEST_FRAME_SIZE])
{
    uint8_t checksum;

    if (frame == 0)
    {
        return 0U;
    }

    frame[0] = K230_BALL_FRAME_HEAD1;
    frame[1] = K230_BALL_FRAME_HEAD2;
    frame[2] = K230_BALL_COMMAND_CAPTURE_REQUEST;
    frame[3] = K230_BALL_CAPTURE_REQUEST_PAYLOAD_SIZE;
    frame[4] = request_id;
    checksum = (uint8_t)(frame[2] + frame[3] + frame[4]);
    frame[5] = checksum;
    return K230_BALL_CAPTURE_REQUEST_FRAME_SIZE;
}

uint8_t K230BallProtocol_MakeUseDefaultOrigin(
    uint8_t frame[K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE])
{
    if (frame == 0)
    {
        return 0U;
    }

    frame[0] = K230_BALL_FRAME_HEAD1;
    frame[1] = K230_BALL_FRAME_HEAD2;
    frame[2] = K230_BALL_COMMAND_USE_DEFAULT_ORIGIN;
    frame[3] = 0U;
    frame[4] = K230_BALL_COMMAND_USE_DEFAULT_ORIGIN;
    return K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE;
}
