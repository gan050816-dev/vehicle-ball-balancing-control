#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stepper_service.h"

typedef struct
{
    uint8_t frames[24][16];
    uint8_t lengths[24];
    uint8_t count;
    bool accept_writes;
} FrameCapture;

static bool capture_write(
    void *context, const uint8_t *data, uint8_t length)
{
    FrameCapture *capture = (FrameCapture *)context;

    if (!capture->accept_writes ||
        capture->count >= 24U ||
        length > 16U)
    {
        return false;
    }

    memcpy(capture->frames[capture->count], data, length);
    capture->lengths[capture->count] = length;
    ++capture->count;
    return true;
}

static void assert_frame(
    const FrameCapture *capture,
    uint8_t index,
    const uint8_t *expected,
    uint8_t expected_length)
{
    assert(index < capture->count);
    assert(capture->lengths[index] == expected_length);
    assert(memcmp(
               capture->frames[index],
               expected,
               expected_length) == 0);
}

static void deliver_ack(
    StepperService *service,
    uint8_t function,
    uint8_t response,
    uint32_t now_ms)
{
    X42Frame frame = {0};

    frame.type = X42_FRAME_CONTROL_RESPONSE;
    frame.function = function;
    frame.response = response;
    StepperService_HandleX42Frame(service, &frame, now_ms);
}

static void deliver_status(
    StepperService *service,
    uint8_t flags,
    uint32_t now_ms)
{
    X42Frame frame = {0};

    frame.type = X42_FRAME_STATUS;
    frame.function = X42_FUNCTION_STATUS;
    frame.status_flags = flags;
    StepperService_HandleX42Frame(service, &frame, now_ms);
}

static void deliver_position(
    StepperService *service,
    uint8_t negative,
    uint32_t magnitude,
    uint32_t now_ms)
{
    X42Frame frame = {0};

    frame.type = X42_FRAME_REALTIME_POSITION;
    frame.function = X42_FUNCTION_REALTIME_POSITION;
    frame.negative = negative;
    frame.magnitude = magnitude;
    StepperService_HandleX42Frame(service, &frame, now_ms);
}

static void bring_service_to_ready(
    StepperService *service,
    FrameCapture *capture)
{
    static const uint8_t stop_frame[] =
        {0x02U, 0xFEU, 0x98U, 0x00U, 0x6BU};
    static const uint8_t enable_frame[] =
        {0x02U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};
    static const uint8_t clear_zero_frame[] =
        {0x02U, 0x0AU, 0x6DU, 0x6BU};
    static const uint8_t level_frame[] = {
        0x02U, 0xFDU, 0x00U, 0x00U, 0x05U, 0x1EU,
        0x00U, 0x00U, 0x01U, 0x7BU, 0x01U, 0x00U, 0x6BU
    };
    static const uint8_t status_query[] =
        {0x02U, 0x3AU, 0x6BU};
    static const uint8_t position_query[] =
        {0x02U, 0x36U, 0x6BU};

    assert(StepperService_BeginLeveling(service, 1000U));
    assert_frame(capture, 0U, stop_frame, sizeof(stop_frame));
    deliver_ack(
        service, 0xFEU, X42_RESPONSE_RECEIVED, 1002U);

    StepperService_Update(service, 1049U);
    assert(capture->count == 1U);
    StepperService_Update(service, 1050U);
    assert_frame(capture, 1U, enable_frame, sizeof(enable_frame));
    deliver_ack(
        service, 0xF3U, X42_RESPONSE_RECEIVED, 1051U);

    StepperService_Update(service, 1150U);
    assert_frame(
        capture, 2U, clear_zero_frame, sizeof(clear_zero_frame));
    deliver_ack(
        service, 0x0AU, X42_RESPONSE_RECEIVED, 1151U);

    StepperService_Update(service, 1250U);
    assert_frame(capture, 3U, level_frame, sizeof(level_frame));
    assert(StepperService_GetCommandedTenthsMm(service) == 149U);
    deliver_ack(
        service, 0xFDU, X42_RESPONSE_RECEIVED, 1251U);

    StepperService_Update(service, 1251U);
    assert_frame(
        capture, 4U, status_query, sizeof(status_query));
    deliver_status(
        service,
        X42_STATUS_ENABLED | X42_STATUS_POSITION_REACHED,
        1252U);

    StepperService_Update(service, 1271U);
    assert_frame(
        capture, 5U, position_query, sizeof(position_query));
    deliver_position(service, 0U, 7770U, 1272U);

    assert(StepperService_GetStatus(service) ==
           STEPPER_STATUS_READY);
    assert(StepperService_GetCommandState(service) ==
           STEPPER_COMMAND_POSITION_REACHED);
    assert(StepperService_GetMeasuredTenthsMm(service) == 149U);
}

static void test_level_sequence_requires_protocol_feedback(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_STOPPED);
    bring_service_to_ready(&service, &capture);
}

static void test_position_conversion_limits_and_ack(void)
{
    static const uint8_t target_frame[] = {
        0x02U, 0xFDU, 0x00U, 0x00U, 0x14U, 0x64U,
        0x00U, 0x00U, 0x01U, 0xFBU, 0x01U, 0x00U, 0x6BU
    };
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    assert(StepperService_PositionTenthsMmToPulses(0U) == 0U);
    assert(StepperService_PositionTenthsMmToPulses(99U) == 252U);
    assert(StepperService_PositionTenthsMmToPulses(149U) == 379U);
    assert(StepperService_PositionTenthsMmToPulses(199U) == 507U);
    assert(StepperService_PositionTenthsMmToPulses(285U) == 726U);
    assert(StepperService_PositionTenthsMmToPulses(295U) == 751U);
    assert(StepperService_EncoderUnitsToTenthsMm(5163U) == 99U);
    assert(StepperService_EncoderUnitsToTenthsMm(7770U) == 149U);
    assert(StepperService_EncoderUnitsToTenthsMm(10378U) == 199U);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 2000U;

    assert(!StepperService_MoveAbsoluteTenthsMm(
        &service, 0U, 20U, 100U));
    assert(!StepperService_MoveAbsoluteTenthsMm(
        &service, 286U, 20U, 100U));
    assert(!StepperService_MoveAbsoluteTenthsMm(
        &service, 149U, 0U, 100U));
    assert(!StepperService_MoveAbsoluteTenthsMm(
        &service, 149U, 3001U, 100U));

    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 199U, 20U, 100U));
    assert_frame(&capture, 0U, target_frame, sizeof(target_frame));
    assert(StepperService_GetCommandState(&service) ==
           STEPPER_COMMAND_SENT);
    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 2001U);
    assert(StepperService_GetCommandState(&service) ==
           STEPPER_COMMAND_ACKNOWLEDGED);
}

static void test_regular_position_requires_status_and_position(void)
{
    static const uint8_t status_query[] =
        {0x02U, 0x3AU, 0x6BU};
    static const uint8_t position_query[] =
        {0x02U, 0x36U, 0x6BU};
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 2000U;

    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 199U, 20U, 100U));
    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 2001U);

    StepperService_Update(&service, 2001U);
    assert_frame(&capture, 1U, status_query, sizeof(status_query));
    deliver_status(
        &service,
        X42_STATUS_ENABLED | X42_STATUS_POSITION_REACHED,
        2002U);

    StepperService_Update(&service, 2021U);
    assert_frame(
        &capture, 2U, position_query, sizeof(position_query));
    deliver_position(&service, 0U, 10378U, 2022U);
    assert(StepperService_GetCommandState(&service) ==
           STEPPER_COMMAND_POSITION_REACHED);

    StepperService_Update(&service, 2041U);
    assert_frame(&capture, 3U, status_query, sizeof(status_query));
}

static void test_relative_angle_adapter_uses_balance_reference(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 2000U;
    service.next_query_ms = 3000U;

    assert(!StepperService_MoveRelativeAngleCentiDegrees(
        &service, -4000, 20U, 100U));
    assert(!StepperService_MoveRelativeAngleCentiDegrees(
        &service, 4000, 20U, 100U));
    assert(StepperService_MoveRelativeAngleCentiDegrees(
        &service, 1432, 20U, 100U));
    assert(StepperService_GetCommandedTenthsMm(&service) == 199U);

    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 2001U);
    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 3000U;
    service.next_query_ms = 4000U;
    assert(StepperService_MoveRelativeAngleCentiDegrees(
        &service, -2865, 20U, 100U));
    assert(StepperService_GetCommandedTenthsMm(&service) == 49U);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.commanded_tenths_mm = 149U;
    service.measured_tenths_mm = 149U;
    service.position_valid = 1U;
    service.now_ms = 4000U;
    service.next_query_ms = 5000U;
    assert(StepperService_BeginTracking(&service));
    assert(StepperService_QueueTrackingTargetAngleCentiDegrees(
        &service, 1432, 20U, 100U));
    StepperService_Update(&service, 4000U);
    assert(StepperService_GetCommandedTenthsMm(&service) == 164U);
}

static void test_regular_position_timeout_and_disabled_fault(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 3000U;
    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 199U, 20U, 100U));
    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 3001U);
    StepperService_Update(&service, 5001U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
    assert(service.arrival_timeout_count == 1U);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    deliver_status(
        &service, X42_STATUS_POSITION_REACHED, 6000U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
}

static void test_feedback_failures_enter_fault(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    assert(StepperService_BeginLeveling(&service, 0U));
    StepperService_Update(&service, 100U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
    assert(service.response_timeout_count == 1U);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    deliver_status(
        &service,
        X42_STATUS_ENABLED | X42_STATUS_STALL_PROTECTION,
        200U);
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
    service.pending_function = 0xFDU;
    deliver_ack(
        &service, 0xFDU,
        X42_RESPONSE_PARAMETER_ERROR, 300U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
    assert(service.protocol_error_count == 1U);
}

static void test_idle_query_timeout_retries_without_hiding_faults(void)
{
    static const uint8_t status_query[] =
        {0x02U, 0x3AU, 0x6BU};
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    StepperService_SetIdleQueryRetry(&service, true);

    StepperService_Update(&service, 1000U);
    assert_frame(&capture, 0U, status_query, sizeof(status_query));
    StepperService_Update(&service, 1020U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_READY);
    assert(service.query_timeout_count == 1U);
    assert(service.query_function == 0U);

    StepperService_Update(&service, 1039U);
    assert(capture.count == 1U);
    StepperService_Update(&service, 1040U);
    assert_frame(&capture, 1U, status_query, sizeof(status_query));

    deliver_status(
        &service, X42_STATUS_POSITION_REACHED, 1041U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
}

static void test_query_timeout_remains_strict_outside_idle_wait(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;

    StepperService_Update(&service, 2000U);
    StepperService_Update(&service, 2020U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
    assert(service.query_timeout_count == 1U);
}

static void test_position_waits_for_inflight_query(void)
{
    static const uint8_t status_query[] =
        {0x02U, 0x3AU, 0x6BU};
    static const uint8_t target_frame[] = {
        0x02U, 0xFDU, 0x00U, 0x00U, 0x14U, 0x64U,
        0x00U, 0x00U, 0x01U, 0xFBU, 0x01U, 0x00U, 0x6BU
    };
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    StepperService_Update(&service, 1000U);
    assert_frame(
        &capture, 0U, status_query, sizeof(status_query));

    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 199U, 20U, 100U));
    assert(capture.count == 1U);

    deliver_status(&service, X42_STATUS_ENABLED, 1001U);
    StepperService_Update(&service, 1002U);
    assert(capture.count == 1U);
    StepperService_Update(&service, 1003U);
    assert_frame(
        &capture, 1U, target_frame, sizeof(target_frame));
}

static void test_discrete_position_keeps_latest_while_ack_pending(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 1000U;
    service.next_query_ms = 2000U;

    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 199U, 20U, 100U));
    assert(capture.count == 1U);
    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 89U, 20U, 100U));
    assert(capture.count == 1U);
    assert(service.queued_position_valid == 1U);

    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 1001U);
    StepperService_Update(&service, 1001U);
    assert(capture.count == 2U);
    assert(StepperService_GetCommandedTenthsMm(
               &service) == 89U);
}

static void test_tracking_rate_limit_and_latest_target(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.commanded_tenths_mm = 149U;
    service.measured_tenths_mm = 149U;
    service.position_valid = 1U;
    service.now_ms = 1000U;
    service.next_query_ms = 2000U;

    assert(StepperService_BeginTracking(&service));
    assert(StepperService_IsTracking(&service));
    assert(StepperService_QueueTrackingTargetTenthsMm(
        &service, 199U, 20U, 100U));
    StepperService_Update(&service, 1000U);
    assert(capture.count == 1U);
    assert(StepperService_GetCommandedTenthsMm(
               &service) == 164U);

    assert(StepperService_QueueTrackingTargetTenthsMm(
        &service, 180U, 20U, 100U));
    assert(StepperService_QueueTrackingTargetTenthsMm(
        &service, 190U, 20U, 100U));
    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 1001U);
    service.next_query_ms = 2000U;

    StepperService_Update(&service, 1079U);
    assert(capture.count == 1U);
    StepperService_Update(&service, 1080U);
    assert(capture.count == 2U);
    assert(StepperService_GetCommandedTenthsMm(
               &service) == 179U);

    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 1081U);
    service.next_query_ms = 2000U;
    StepperService_Update(&service, 1160U);
    assert(capture.count == 3U);
    assert(StepperService_GetCommandedTenthsMm(
               &service) == 190U);

    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 1161U);
    service.next_query_ms = 2000U;
    assert(StepperService_QueueTrackingTargetTenthsMm(
        &service, 192U, 20U, 100U));
    StepperService_Update(&service, 1240U);
    assert(capture.count == 3U);

    StepperService_EndTracking(&service);
    assert(!StepperService_IsTracking(&service));
}

static void test_tracking_supersedes_queued_discrete_arrival(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.now_ms = 1000U;
    service.next_query_ms = 2000U;

    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 199U, 20U, 100U));
    assert(StepperService_MoveAbsoluteTenthsMm(
        &service, 89U, 20U, 100U));
    assert(StepperService_BeginTracking(&service));
    assert(StepperService_QueueTrackingTargetTenthsMm(
        &service, 149U, 20U, 100U));

    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 1001U);
    StepperService_Update(&service, 1001U);
    assert(StepperService_GetCommandedTenthsMm(
               &service) == 89U);
    assert(service.arrival_pending == 0U);
}

static void test_tracking_queries_position_without_waiting_arrival(void)
{
    static const uint8_t status_query[] =
        {0x02U, 0x3AU, 0x6BU};
    static const uint8_t position_query[] =
        {0x02U, 0x36U, 0x6BU};
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.commanded_tenths_mm = 149U;
    service.measured_tenths_mm = 149U;
    service.position_valid = 1U;
    service.now_ms = 0U;

    assert(StepperService_BeginTracking(&service));
    StepperService_Update(&service, 0U);
    assert_frame(
        &capture, 0U, status_query, sizeof(status_query));
    deliver_status(&service, X42_STATUS_ENABLED, 1U);
    StepperService_Update(&service, 20U);
    assert_frame(
        &capture, 1U, position_query, sizeof(position_query));
    assert(service.arrival_pending == 0U);
}

static void test_tracking_follow_error_faults_after_grace(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.commanded_tenths_mm = 149U;
    service.measured_tenths_mm = 149U;
    service.position_valid = 1U;
    service.now_ms = 0U;

    assert(StepperService_BeginTracking(&service));
    deliver_position(&service, 0U, 5163U, 600U);
    assert(service.tracking_error_active == 1U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_READY);

    deliver_position(&service, 0U, 5163U, 999U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_READY);
    deliver_position(&service, 0U, 5163U, 1000U);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
}

static void test_reset_stop_disable_and_write_failure(void)
{
    static const uint8_t stop_frame[] =
        {0x02U, 0xFEU, 0x98U, 0x00U, 0x6BU};
    static const uint8_t disable_frame[] =
        {0x02U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    FrameCapture reject = {{{0}}, {0}, 0U, false};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    assert(StepperService_Stop(&service));
    assert_frame(&capture, 0U, stop_frame, sizeof(stop_frame));
    assert(StepperService_Disable(&service));
    assert_frame(&capture, 1U, disable_frame, sizeof(disable_frame));
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_STOPPED);

    StepperService_Init(&service, capture_write, &reject);
    assert(!StepperService_BeginLeveling(&service, 0U));
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
}

static void test_axis_angle_tracking_keeps_task2_limits(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    assert(StepperService_AngleCdegToPulses(4264U) == 379U);
    assert(StepperService_EncoderUnitsToAngleCdeg(7770U) ==
           4268U);

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    service.commanded_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    service.measured_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    service.position_valid = 1U;
    service.now_ms = 0U;
    service.next_query_ms = 2000U;

    assert(StepperService_BeginAxisAngleTracking(&service));
    assert(StepperService_TrackAbsoluteAxisAngleCdeg(
        &service, 6000U, 20U, 100U));
    StepperService_Update(&service, 0U);
    assert(StepperService_GetCommandedAngleCdeg(&service) ==
           4694U);
    assert(capture.count == 1U);

    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 1U);
    service.next_query_ms = 2000U;
    StepperService_Update(&service, 39U);
    assert(capture.count == 1U);
    StepperService_Update(&service, 40U);
    assert(capture.count == 2U);
    assert(StepperService_GetCommandedAngleCdeg(&service) ==
           5124U);

    deliver_ack(
        &service, 0xFDU, X42_RESPONSE_RECEIVED, 41U);
    assert(StepperService_EndAxisAngleTrackingAtLevel(
        &service, 20U, 100U));
    service.next_query_ms = 2000U;
    StepperService_Update(&service, 42U);
    assert(StepperService_GetCommandedAngleCdeg(&service) ==
           STEPPER_LEVEL_ANGLE_CDEG);
}

static void test_axis_angle_hard_boundary_faults(void)
{
    FrameCapture capture = {{{0}}, {0}, 0U, true};
    StepperService service;

    StepperService_Init(&service, capture_write, &capture);
    service.status = STEPPER_STATUS_READY;
    deliver_position(&service, 0U, 15500U, 100U);
    assert(StepperService_GetMeasuredAngleCdeg(&service) >
           STEPPER_HARD_MAX_ANGLE_CDEG);
    assert(StepperService_GetStatus(&service) ==
           STEPPER_STATUS_FAULT);
}

int main(void)
{
    test_level_sequence_requires_protocol_feedback();
    test_position_conversion_limits_and_ack();
    test_regular_position_requires_status_and_position();
    test_relative_angle_adapter_uses_balance_reference();
    test_regular_position_timeout_and_disabled_fault();
    test_feedback_failures_enter_fault();
    test_idle_query_timeout_retries_without_hiding_faults();
    test_query_timeout_remains_strict_outside_idle_wait();
    test_position_waits_for_inflight_query();
    test_discrete_position_keeps_latest_while_ack_pending();
    test_tracking_rate_limit_and_latest_target();
    test_tracking_supersedes_queued_discrete_arrival();
    test_tracking_queries_position_without_waiting_arrival();
    test_tracking_follow_error_faults_after_grace();
    test_reset_stop_disable_and_write_failure();
    test_axis_angle_tracking_keeps_task2_limits();
    test_axis_angle_hard_boundary_faults();
    return 0;
}
