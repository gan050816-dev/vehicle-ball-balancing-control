#ifndef X42_PROTOCOL_H
#define X42_PROTOCOL_H

#include <stdint.h>

#define X42_PROTOCOL_MOTOR_ADDRESS          2U
#define X42_PROTOCOL_FRAME_END           0x6BU
#define X42_PROTOCOL_MAX_FRAME_SIZE         8U

#define X42_FUNCTION_VERSION              0x1FU
#define X42_FUNCTION_REALTIME_POSITION    0x36U
#define X42_FUNCTION_POSITION_ERROR       0x37U
#define X42_FUNCTION_STATUS               0x3AU

#define X42_RESPONSE_RECEIVED             0x02U
#define X42_RESPONSE_HOME_ALREADY_ZERO    0x12U
#define X42_RESPONSE_LIMIT_ACTIVE         0x22U
#define X42_RESPONSE_PARAMETER_ERROR      0xE2U
#define X42_RESPONSE_FORMAT_ERROR         0xEEU
#define X42_RESPONSE_ACTION_COMPLETE      0x9FU

#define X42_STATUS_ENABLED                0x01U
#define X42_STATUS_POSITION_REACHED       0x02U
#define X42_STATUS_STALL                  0x04U
#define X42_STATUS_STALL_PROTECTION       0x08U

typedef enum
{
    X42_FRAME_NONE = 0,
    X42_FRAME_CONTROL_RESPONSE,
    X42_FRAME_VERSION,
    X42_FRAME_STATUS,
    X42_FRAME_REALTIME_POSITION,
    X42_FRAME_POSITION_ERROR
} X42FrameType;

typedef struct
{
    X42FrameType type;
    uint8_t function;
    uint8_t response;
    uint8_t status_flags;
    uint8_t negative;
    uint32_t magnitude;
    uint16_t firmware_version;
    uint8_t hardware_series;
    uint8_t hardware_type;
    uint8_t hardware_version;
    uint32_t received_at_ms;
} X42Frame;

typedef struct
{
    uint8_t buffer[X42_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t position;
    uint8_t expected_length;
    uint8_t frame_ready;
    X42Frame latest;
    uint32_t valid_frame_count;
    uint32_t format_error_count;
} X42Protocol;

void X42Protocol_Init(X42Protocol *protocol);
void X42Protocol_PushByte(
    X42Protocol *protocol, uint8_t byte, uint32_t now_ms);
uint8_t X42Protocol_TakeFrame(
    X42Protocol *protocol, X42Frame *frame);

#endif
