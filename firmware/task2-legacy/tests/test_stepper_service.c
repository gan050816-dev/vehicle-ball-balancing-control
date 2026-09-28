#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stepper_service.h"

typedef struct
{
    uint8_t frames[32][16];
    uint8_t lengths[32];
    uint8_t count;
    bool accept;
} Capture;

static bool capture_write(void *context,
                          const uint8_t *data,
                          uint8_t length)
{
    Capture *capture = (Capture *)context;

    if (!capture->accept || capture->count >= 32U ||
        length > 16U)
    {
        return false;
    }
    memcpy(capture->frames[capture->count], data, length);
    capture->lengths[capture->count] = length;
    ++capture->count;
    return true;
}

static void deliver_ack(StepperService *service,
                        uint8_t function,
                        uint32_t now_ms)
{
    X42Frame frame = {0};

    frame.type = X42_FRAME_CONTROL_RESPONSE;
    frame.function = function;
    frame.response = X42_RESPONSE_RECEIVED;
    StepperService_HandleX42Frame(service, &frame, now_ms);
}

static void deliver_status(StepperService *service,
                           uint8_t flags,
                           uint32_t now_ms)
{
    X42Frame frame = {0};

    frame.type = X42_FRAME_STATUS;
    frame.status_flags = flags;
    StepperService_HandleX42Frame(service, &frame, now_ms);
}

static void deliver_position(StepperService *service,
                             uint32_t encoder_units,
                             uint32_t now_ms)
{
    X42Frame frame = {0};

    frame.type = X42_FRAME_REALTIME_POSITION;
    frame.magnitude = encoder_units;
    StepperService_HandleX42Frame(service, &frame, now_ms);
}

static void test_leveling_requires_feedback(void)
{
    Capture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    assert(StepperService_BeginLeveling(&service, 1000U));
    assert(capture.count == 1U);
    assert(capture.frames[0][1] == 0xFEU);

    deliver_ack(&service, 0xFEU, 1001U);
    StepperService_Update(&service, 1050U);
    assert(capture.frames[1][1] == 0xF3U);

    deliver_ack(&service, 0xF3U, 1051U);
    StepperService_Update(&service, 1150U);
    assert(capture.frames[2][1] == 0x0AU);

    deliver_ack(&service, 0x0AU, 1151U);
    StepperService_Update(&service, 1250U);
    assert(capture.frames[3][1] == 0xFDU);
    assert(capture.frames[3][6] == 0x00U);
    assert(capture.frames[3][7] == 0x00U);
    assert(capture.frames[3][8] == 0x01U);
    assert(capture.frames[3][9] == 0x7BU);
    assert(StepperService_GetCommandedAngleCdeg(&service) == 4264U);

    deliver_ack(&service, 0xFDU, 1251U);
    StepperService_Update(&service, 1251U);
    assert(capture.frames[4][1] == X42_FUNCTION_STATUS);
    deliver_status(&service,
                   X42_STATUS_ENABLED |
                       X42_STATUS_POSITION_REACHED,
                   1252U);
    StepperService_Update(&service, 1271U);
    assert(capture.frames[5][1] ==
           X42_FUNCTION_REALTIME_POSITION);
    deliver_position(&service, 7770U, 1272U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_READY);
    assert(StepperService_GetMeasuredAngleCdeg(&service) == 4268U);
    assert(StepperService_GetRelativeAngleCdeg(&service) == 4);
}

static void test_tracking_shapes_latest_target_from_last_sent_position(void)
{
    Capture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 2000U;
    service.commanded_angle_cdeg = 4264U;
    service.measured_angle_cdeg = 4264U;
    service.position_valid = 1U;
    service.next_query_ms = 3000U;
    assert(StepperService_BeginTracking(&service));

    assert(StepperService_TrackAbsoluteAngleCdeg(
        &service, 5700U, 20U, 100U));
    assert(capture.count == 0U);
    StepperService_Update(&service, 2000U);
    assert(capture.count == 1U);
    assert(service.pending_function == 0xFDU);
    assert(service.arrival_pending == 0U);
    assert(StepperService_GetCommandedAngleCdeg(&service) == 4694U);

    assert(StepperService_TrackAbsoluteAngleCdeg(
        &service, 6000U, 20U, 100U));
    assert(StepperService_TrackAbsoluteAngleCdeg(
        &service, 6500U, 20U, 100U));
    assert(service.tracking_desired_angle_cdeg == 6500U);
    assert(capture.count == 1U);

    StepperService_Update(&service, 2050U);
    assert(capture.count == 1U);
    deliver_ack(&service, 0xFDU, 2090U);
    StepperService_Update(&service, 2090U);
    assert(capture.frames[1][1] == X42_FUNCTION_STATUS);
    deliver_status(&service, X42_STATUS_ENABLED, 2091U);
    assert(service.next_query_function ==
           X42_FUNCTION_REALTIME_POSITION);
    StepperService_Update(&service, 2093U);
    assert(capture.frames[2][1] == 0xFDU);
    assert(StepperService_GetCommandedAngleCdeg(&service) == 5124U);
    assert(service.tracking_desired_angle_cdeg == 6500U);

    deliver_ack(&service, 0xFDU, 2094U);
    service.next_query_ms = 3000U;
    StepperService_Update(&service, 2132U);
    assert(capture.count == 3U);
    StepperService_Update(&service, 2133U);
    assert(capture.count == 4U);
    assert(StepperService_GetCommandedAngleCdeg(&service) == 5554U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_READY);
    assert(service.arrival_timeout_count == 0U);
}

static void test_tracking_deadband_and_sustained_follow_error(void)
{
    Capture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 0U;
    service.commanded_angle_cdeg = 4264U;
    service.measured_angle_cdeg = 4264U;
    service.position_valid = 1U;
    service.next_query_ms = 1000U;
    assert(StepperService_BeginTracking(&service));

    assert(StepperService_TrackAbsoluteAngleCdeg(
        &service, 4300U, 20U, 100U));
    StepperService_Update(&service, 0U);
    assert(capture.count == 0U);
    assert(!service.tracking_desired_valid);

    deliver_position(&service, 5163U, 500U);
    assert(service.tracking_error_active);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_READY);
    deliver_position(&service, 5163U, 899U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_READY);
    deliver_position(&service, 5163U, 900U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
    assert(service.tracking_follow_error_count == 1U);
}

static void test_end_tracking_queues_verified_level(void)
{
    Capture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 100U;
    service.next_query_ms = 1000U;
    assert(StepperService_BeginTracking(&service));
    service.pending_function = 0xFDU;

    assert(StepperService_EndTrackingAtLevel(
        &service, 20U, 100U));
    assert(!service.tracking_enabled);
    assert(service.queued_position_valid);
    assert(service.queued_angle_cdeg ==
           STEPPER_LEVEL_ANGLE_CDEG);
    assert(service.queued_require_arrival);

    deliver_ack(&service, 0xFDU, 101U);
    service.next_query_ms = 1000U;
    StepperService_Update(&service, 102U);
    assert(capture.count == 1U);
    assert(capture.frames[0][1] == 0xFDU);
    assert(service.arrival_pending);
}

static void test_limits_faults_and_conversions(void)
{
    Capture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    assert(StepperService_AngleCdegToPulses(4264U) == 379U);
    assert(StepperService_AngleCdegToPulses(36000U) == 3200U);
    assert(StepperService_EncoderUnitsToAngleCdeg(7770U) == 4268U);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    assert(StepperService_BeginTracking(&service));
    assert(!StepperService_TrackAbsoluteAngleCdeg(
        &service, 0U, 20U, 100U));
    assert(!StepperService_TrackAbsoluteAngleCdeg(
        &service, 8166U, 20U, 100U));
    assert(!StepperService_TrackAbsoluteAngleCdeg(
        &service, STEPPER_LEVEL_ANGLE_CDEG, 0U, 100U));

    deliver_status(&service, X42_STATUS_POSITION_REACHED, 10U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    StepperService_ReportRxOverflow(&service);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
    assert(service.rx_overflow_count == 1U);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    deliver_position(&service, 16000U, 20U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
}

int main(void)
{
    test_leveling_requires_feedback();
    test_tracking_shapes_latest_target_from_last_sent_position();
    test_tracking_deadband_and_sustained_follow_error();
    test_end_tracking_queues_verified_level();
    test_limits_faults_and_conversions();
    return 0;
}
