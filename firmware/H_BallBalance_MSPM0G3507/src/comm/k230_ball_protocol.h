#ifndef K230_BALL_PROTOCOL_H
#define K230_BALL_PROTOCOL_H

#include <stdint.h>

#define K230_BALL_FRAME_HEAD1              0xA5U
#define K230_BALL_FRAME_HEAD2              0x5AU
#define K230_BALL_COMMAND_STATE            0x10U
#define K230_BALL_COMMAND_CAPTURE_RESULT   0x11U
#define K230_BALL_COMMAND_CAPTURE_REQUEST  0x90U
#define K230_BALL_COMMAND_USE_DEFAULT_ORIGIN 0x91U
#define K230_BALL_PAYLOAD_SIZE                8U
#define K230_BALL_FRAME_SIZE                 13U
#define K230_BALL_CAPTURE_REQUEST_PAYLOAD_SIZE 1U
#define K230_BALL_CAPTURE_REQUEST_FRAME_SIZE   6U
#define K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE     5U
#define K230_BALL_CAPTURE_RESULT_PAYLOAD_SIZE  4U
#define K230_BALL_CAPTURE_RESULT_FRAME_SIZE    9U

#define K230_BALL_FLAG_VALID              0x01U
#define K230_BALL_FLAG_CALIBRATED         0x02U
#define K230_BALL_FLAG_MULTI              0x04U
#define K230_BALL_FLAG_TARGET_CENTER_VALID 0x08U
#define K230_BALL_FLAG_MASK               0x0FU
#define K230_BALL_X_ABS_MAX_TENTHS_MM       1500

#define K230_BALL_CAPTURE_STATUS_OK          0U
#define K230_BALL_CAPTURE_STATUS_TIMEOUT     1U
#define K230_BALL_CAPTURE_STATUS_UNSTABLE    2U

typedef struct
{
    uint8_t valid;
    uint8_t calibrated;
    uint8_t multiple_candidates;
    uint8_t target_center_valid;
    uint8_t sequence;
    int16_t x_tenths_mm;
    uint16_t confidence_permille;
    uint16_t k230_timestamp_ms16;
    uint32_t received_at_ms;
} K230BallFrame;

typedef struct
{
    uint8_t request_id;
    uint8_t status;
    uint16_t center_x_px;
    uint32_t received_at_ms;
} K230BallCaptureResult;

typedef struct
{
    uint8_t parse_state;
    uint8_t command;
    uint8_t length;
    uint8_t payload[K230_BALL_PAYLOAD_SIZE];
    uint8_t payload_position;
    uint8_t latest_ready;
    uint8_t capture_result_ready;
    K230BallFrame latest;
    K230BallCaptureResult capture_result;
    uint32_t valid_frame_count;
    uint32_t valid_capture_result_count;
    uint32_t checksum_error_count;
    uint32_t format_error_count;
} K230BallProtocol;

void K230BallProtocol_Init(K230BallProtocol *protocol);
void K230BallProtocol_PushByte(K230BallProtocol *protocol,
                               uint8_t byte,
                               uint32_t now_ms);
uint8_t K230BallProtocol_TakeLatest(K230BallProtocol *protocol,
                                   K230BallFrame *frame);
uint8_t K230BallProtocol_TakeCaptureResult(
    K230BallProtocol *protocol,
    K230BallCaptureResult *result);
uint8_t K230BallProtocol_MakeCaptureRequest(
    uint8_t request_id,
    uint8_t frame[K230_BALL_CAPTURE_REQUEST_FRAME_SIZE]);
uint8_t K230BallProtocol_MakeUseDefaultOrigin(
    uint8_t frame[K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE]);

#endif
