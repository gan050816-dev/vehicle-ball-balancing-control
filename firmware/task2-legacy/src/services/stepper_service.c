#include "stepper_service.h"

#include <string.h>

#define X42_FRAME_END                         0x6BU
#define X42_DIRECTION_POSITIVE                0x00U
#define X42_POSITION_MODE_ABSOLUTE             0x01U
#define X42_EXECUTE_IMMEDIATELY                0x00U

#define STEPPER_ENABLE_DELAY_MS                  50U
#define STEPPER_CLEAR_ZERO_DELAY_MS             100U
#define STEPPER_MOVE_DELAY_MS                   100U
#define STEPPER_RESPONSE_TIMEOUT_MS             100U
#define STEPPER_POSITION_ARRIVAL_TIMEOUT_MS    2000U
#define STEPPER_QUERY_PERIOD_MS                  20U
#define STEPPER_QUERY_TIMEOUT_MS                 20U
#define STEPPER_INTER_FRAME_DELAY_MS              2U
#define STEPPER_POSITION_TOLERANCE_CDEG           86U

#define ENCODER_UNITS_PER_REVOLUTION          65536UL

static bool TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool PositionParametersValid(uint16_t angle_cdeg,
                                    uint16_t speed_rpm)
{
    return angle_cdeg >= STEPPER_NORMAL_MIN_ANGLE_CDEG &&
           angle_cdeg <= STEPPER_NORMAL_MAX_ANGLE_CDEG &&
           speed_rpm != 0U && speed_rpm <= 3000U;
}

static void SetFault(StepperService *service)
{
    service->status = STEPPER_STATUS_FAULT;
    service->level_phase = STEPPER_LEVEL_PHASE_NONE;
    service->pending_function = 0U;
    service->query_function = 0U;
    service->arrival_pending = 0U;
    service->queued_position_valid = 0U;
    service->tracking_enabled = 0U;
    service->tracking_desired_valid = 0U;
    service->tracking_error_active = 0U;
}

static bool Write(StepperService *service,
                  const uint8_t *data,
                  uint8_t length)
{
    if (service->write == 0 ||
        !service->write(service->write_context, data, length))
    {
        SetFault(service);
        return false;
    }
    return true;
}

static bool SendTracked(StepperService *service,
                        const uint8_t *data,
                        uint8_t length,
                        uint8_t function,
                        uint32_t now_ms,
                        uint32_t action_delay_ms)
{
    if (!Write(service, data, length))
    {
        return false;
    }
    service->pending_function = function;
    service->command_state = STEPPER_COMMAND_SENT;
    service->deadline_ms = now_ms + STEPPER_RESPONSE_TIMEOUT_MS;
    service->next_action_ms = now_ms + action_delay_ms;
    return true;
}

static bool SendStopTracked(StepperService *service,
                            uint32_t now_ms)
{
    static const uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xFEU, 0x98U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };
    return SendTracked(service, frame, (uint8_t)sizeof(frame),
                       0xFEU, now_ms, STEPPER_ENABLE_DELAY_MS);
}

static bool SendStopUntracked(StepperService *service)
{
    static const uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xFEU, 0x98U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };
    return Write(service, frame, (uint8_t)sizeof(frame));
}

static bool SendEnableTracked(StepperService *service,
                              bool enabled,
                              uint32_t now_ms,
                              uint32_t delay_ms)
{
    uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xF3U, 0xABU, 0x00U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };
    frame[3] = enabled ? 1U : 0U;
    return SendTracked(service, frame, (uint8_t)sizeof(frame),
                       0xF3U, now_ms, delay_ms);
}

static bool SendEnableUntracked(StepperService *service,
                                bool enabled)
{
    uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xF3U, 0xABU, 0x00U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };
    frame[3] = enabled ? 1U : 0U;
    return Write(service, frame, (uint8_t)sizeof(frame));
}

static bool SendClearZero(StepperService *service,
                          uint32_t now_ms)
{
    static const uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0x0AU, 0x6DU, X42_FRAME_END
    };
    return SendTracked(service, frame, (uint8_t)sizeof(frame),
                       0x0AU, now_ms, STEPPER_MOVE_DELAY_MS);
}

static bool SendPosition(StepperService *service,
                         uint16_t angle_cdeg,
                         uint16_t speed_rpm,
                         uint8_t acceleration,
                         uint8_t require_arrival,
                         uint32_t now_ms)
{
    uint32_t pulses =
        StepperService_AngleCdegToPulses(angle_cdeg);
    uint8_t frame[13];

    frame[0] = STEPPER_MOTOR_ADDRESS;
    frame[1] = 0xFDU;
    frame[2] = X42_DIRECTION_POSITIVE;
    frame[3] = (uint8_t)(speed_rpm >> 8U);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(pulses >> 24U);
    frame[7] = (uint8_t)(pulses >> 16U);
    frame[8] = (uint8_t)(pulses >> 8U);
    frame[9] = (uint8_t)pulses;
    frame[10] = X42_POSITION_MODE_ABSOLUTE;
    frame[11] = X42_EXECUTE_IMMEDIATELY;
    frame[12] = X42_FRAME_END;

    service->query_function = 0U;
    service->next_query_function = X42_FUNCTION_STATUS;
    service->position_valid = 0U;
    service->position_reached = 0U;
    service->arrival_pending = require_arrival;
    if (!SendTracked(service, frame, (uint8_t)sizeof(frame),
                     0xFDU, now_ms, 0U))
    {
        return false;
    }

    service->commanded_angle_cdeg = angle_cdeg;
    service->arrival_deadline_ms =
        now_ms + STEPPER_POSITION_ARRIVAL_TIMEOUT_MS;
    service->next_query_ms = now_ms;
    return true;
}

static bool SendQuery(StepperService *service,
                      uint8_t function,
                      uint32_t now_ms)
{
    uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, function, X42_FRAME_END
    };

    if (!Write(service, frame, (uint8_t)sizeof(frame)))
    {
        return false;
    }
    service->query_function = function;
    service->query_deadline_ms = now_ms + STEPPER_QUERY_TIMEOUT_MS;
    return true;
}

static void CheckArrival(StepperService *service)
{
    uint16_t difference;

    if (service->arrival_pending == 0U ||
        service->position_reached == 0U ||
        service->position_valid == 0U)
    {
        return;
    }

    difference =
        service->measured_angle_cdeg > service->commanded_angle_cdeg
            ? (uint16_t)(service->measured_angle_cdeg -
                         service->commanded_angle_cdeg)
            : (uint16_t)(service->commanded_angle_cdeg -
                         service->measured_angle_cdeg);
    if (difference <= STEPPER_POSITION_TOLERANCE_CDEG)
    {
        service->arrival_pending = 0U;
        service->command_state = STEPPER_COMMAND_POSITION_REACHED;
        if (service->status == STEPPER_STATUS_LEVELING)
        {
            service->status = STEPPER_STATUS_READY;
            service->level_phase = STEPPER_LEVEL_PHASE_NONE;
        }
    }
}

static void QueuePosition(StepperService *service,
                          uint16_t angle_cdeg,
                          uint16_t speed_rpm,
                          uint8_t acceleration,
                          uint8_t require_arrival)
{
    service->queued_angle_cdeg = angle_cdeg;
    service->queued_speed_rpm = speed_rpm;
    service->queued_acceleration = acceleration;
    service->queued_require_arrival = require_arrival;
    service->queued_position_valid = 1U;
}

static void SendNextTrackingSegment(StepperService *service,
                                    uint32_t now_ms)
{
    int32_t delta;
    uint16_t next_angle_cdeg;

    if (service->tracking_enabled == 0U ||
        service->tracking_desired_valid == 0U ||
        !TimeReached(now_ms, service->next_tracking_send_ms))
    {
        return;
    }

    delta = (int32_t)service->tracking_desired_angle_cdeg -
            service->commanded_angle_cdeg;
    if (delta < (int32_t)STEPPER_TRACK_MIN_DELTA_CDEG &&
        delta > -(int32_t)STEPPER_TRACK_MIN_DELTA_CDEG)
    {
        service->tracking_desired_valid = 0U;
        return;
    }

    if (delta > (int32_t)STEPPER_TRACK_MAX_STEP_CDEG)
    {
        next_angle_cdeg = (uint16_t)(
            service->commanded_angle_cdeg +
            STEPPER_TRACK_MAX_STEP_CDEG);
    }
    else if (delta <
             -(int32_t)STEPPER_TRACK_MAX_STEP_CDEG)
    {
        next_angle_cdeg = (uint16_t)(
            service->commanded_angle_cdeg -
            STEPPER_TRACK_MAX_STEP_CDEG);
    }
    else
    {
        next_angle_cdeg = service->tracking_desired_angle_cdeg;
        service->tracking_desired_valid = 0U;
    }

    if (SendPosition(service, next_angle_cdeg,
                     service->tracking_speed_rpm,
                     service->tracking_acceleration,
                     0U, now_ms))
    {
        service->next_tracking_send_ms =
            now_ms + STEPPER_TRACK_SEND_PERIOD_MS;
    }
}

static void CheckTrackingFollowError(StepperService *service,
                                     uint32_t now_ms)
{
    uint16_t error;

    if (service->tracking_enabled == 0U ||
        service->position_valid == 0U ||
        !TimeReached(now_ms, service->tracking_grace_deadline_ms))
    {
        return;
    }

    error = service->measured_angle_cdeg >
                    service->commanded_angle_cdeg
                ? (uint16_t)(service->measured_angle_cdeg -
                             service->commanded_angle_cdeg)
                : (uint16_t)(service->commanded_angle_cdeg -
                             service->measured_angle_cdeg);
    if (error <= STEPPER_TRACK_ERROR_LIMIT_CDEG)
    {
        service->tracking_error_active = 0U;
        return;
    }
    if (service->tracking_error_active == 0U)
    {
        service->tracking_error_active = 1U;
        service->tracking_error_since_ms = now_ms;
        return;
    }
    if (TimeReached(
            now_ms,
            service->tracking_error_since_ms +
                STEPPER_TRACK_ERROR_TIMEOUT_MS))
    {
        service->tracking_follow_error_count++;
        SetFault(service);
    }
}

void StepperService_Init(StepperService *service,
                         StepperWriteFunction write,
                         void *write_context)
{
    memset(service, 0, sizeof(*service));
    service->write = write;
    service->write_context = write_context;
    service->status = STEPPER_STATUS_STOPPED;
    service->next_query_function = X42_FUNCTION_STATUS;
}

bool StepperService_BeginLeveling(StepperService *service,
                                  uint32_t now_ms)
{
    if (service->status == STEPPER_STATUS_LEVELING)
    {
        return false;
    }

    service->status = STEPPER_STATUS_LEVELING;
    service->level_phase = STEPPER_LEVEL_PHASE_WAIT_STOP_ACK;
    service->position_valid = 0U;
    service->position_reached = 0U;
    service->pending_function = 0U;
    service->query_function = 0U;
    service->next_query_function = X42_FUNCTION_STATUS;
    service->queued_position_valid = 0U;
    service->arrival_pending = 0U;
    service->tracking_enabled = 0U;
    service->tracking_desired_valid = 0U;
    service->tracking_error_active = 0U;
    service->command_state = STEPPER_COMMAND_NONE;
    service->commanded_angle_cdeg = 0U;
    service->now_ms = now_ms;
    return SendStopTracked(service, now_ms);
}

void StepperService_Update(StepperService *service,
                           uint32_t now_ms)
{
    service->now_ms = now_ms;

    if (service->status == STEPPER_STATUS_FAULT ||
        service->status == STEPPER_STATUS_STOPPED)
    {
        return;
    }
    if (service->pending_function != 0U)
    {
        if (TimeReached(now_ms, service->deadline_ms))
        {
            service->response_timeout_count++;
            SetFault(service);
        }
        return;
    }

    if (service->level_phase == STEPPER_LEVEL_PHASE_WAIT_STOP_ACK)
    {
        if (TimeReached(now_ms, service->next_action_ms) &&
            SendEnableTracked(service, true, now_ms,
                              STEPPER_CLEAR_ZERO_DELAY_MS))
        {
            service->level_phase =
                STEPPER_LEVEL_PHASE_WAIT_ENABLE_ACK;
        }
        return;
    }
    if (service->level_phase == STEPPER_LEVEL_PHASE_WAIT_ENABLE_ACK)
    {
        if (TimeReached(now_ms, service->next_action_ms) &&
            SendClearZero(service, now_ms))
        {
            service->level_phase =
                STEPPER_LEVEL_PHASE_WAIT_CLEAR_ZERO_ACK;
        }
        return;
    }
    if (service->level_phase ==
        STEPPER_LEVEL_PHASE_WAIT_CLEAR_ZERO_ACK)
    {
        if (TimeReached(now_ms, service->next_action_ms) &&
            SendPosition(service, STEPPER_LEVEL_ANGLE_CDEG,
                         STEPPER_LEVEL_SPEED_RPM,
                         STEPPER_LEVEL_ACCELERATION,
                         1U, now_ms))
        {
            service->level_phase =
                STEPPER_LEVEL_PHASE_WAIT_MOVE_ACK;
        }
        return;
    }
    if (service->level_phase == STEPPER_LEVEL_PHASE_WAIT_MOVE_ACK)
    {
        service->level_phase = STEPPER_LEVEL_PHASE_WAIT_ARRIVAL;
        service->next_query_ms = now_ms;
    }
    if (service->level_phase != STEPPER_LEVEL_PHASE_WAIT_ARRIVAL &&
        service->status != STEPPER_STATUS_READY)
    {
        return;
    }

    if (service->arrival_pending != 0U &&
        TimeReached(now_ms, service->arrival_deadline_ms))
    {
        service->arrival_timeout_count++;
        SetFault(service);
        return;
    }
    if (service->query_function != 0U)
    {
        if (TimeReached(now_ms, service->query_deadline_ms))
        {
            service->query_timeout_count++;
            SetFault(service);
        }
        return;
    }

    /* Health polling has priority over a queued tracking target. */
    if (TimeReached(now_ms, service->next_query_ms))
    {
        uint8_t function = service->next_query_function;

        if (SendQuery(service, function, now_ms))
        {
            service->next_query_ms =
                now_ms + STEPPER_QUERY_PERIOD_MS;
        }
        return;
    }

    if (service->tracking_enabled != 0U)
    {
        SendNextTrackingSegment(service, now_ms);
        return;
    }

    if (service->status == STEPPER_STATUS_READY &&
        service->queued_position_valid != 0U &&
        TimeReached(now_ms, service->next_action_ms))
    {
        uint16_t angle_cdeg = service->queued_angle_cdeg;
        uint16_t speed = service->queued_speed_rpm;
        uint8_t acceleration = service->queued_acceleration;
        uint8_t require_arrival =
            service->queued_require_arrival;

        service->queued_position_valid = 0U;
        (void)SendPosition(service, angle_cdeg, speed, acceleration,
                           require_arrival, now_ms);
    }
}

void StepperService_HandleX42Frame(StepperService *service,
                                   const X42Frame *frame,
                                   uint32_t now_ms)
{
    if (frame == 0 || service->status == STEPPER_STATUS_FAULT)
    {
        return;
    }

    service->now_ms = now_ms;
    if (frame->type == X42_FRAME_CONTROL_RESPONSE)
    {
        if (frame->response == X42_RESPONSE_PARAMETER_ERROR ||
            frame->response == X42_RESPONSE_FORMAT_ERROR)
        {
            service->protocol_error_count++;
            SetFault(service);
            return;
        }
        if (frame->response == X42_RESPONSE_ACTION_COMPLETE &&
            frame->function == 0xFDU)
        {
            service->position_reached = 1U;
            service->next_query_function =
                X42_FUNCTION_REALTIME_POSITION;
            CheckArrival(service);
            return;
        }
        if (frame->function == service->pending_function &&
            frame->response == X42_RESPONSE_RECEIVED)
        {
            service->pending_function = 0U;
            service->command_state = STEPPER_COMMAND_ACKNOWLEDGED;
            if (frame->function == 0xFDU)
            {
                if (service->arrival_pending != 0U)
                {
                    service->arrival_deadline_ms =
                        now_ms +
                        STEPPER_POSITION_ARRIVAL_TIMEOUT_MS;
                }
                service->next_query_ms = now_ms;
            }
        }
        return;
    }

    if (frame->type == X42_FRAME_STATUS)
    {
        if (service->query_function == X42_FUNCTION_STATUS)
        {
            service->query_function = 0U;
            service->next_action_ms =
                now_ms + STEPPER_INTER_FRAME_DELAY_MS;
        }
        service->status_flags = frame->status_flags;
        if ((frame->status_flags &
             X42_STATUS_STALL_PROTECTION) != 0U ||
            (service->status != STEPPER_STATUS_STOPPED &&
             (frame->status_flags & X42_STATUS_ENABLED) == 0U))
        {
            SetFault(service);
            return;
        }
        service->position_reached =
            (frame->status_flags &
             X42_STATUS_POSITION_REACHED) != 0U ? 1U : 0U;
        service->next_query_function =
            service->tracking_enabled != 0U
                ? X42_FUNCTION_REALTIME_POSITION
                : service->position_reached != 0U
                ? X42_FUNCTION_REALTIME_POSITION
                : X42_FUNCTION_STATUS;
        CheckArrival(service);
        return;
    }

    if (frame->type == X42_FRAME_REALTIME_POSITION)
    {
        uint16_t measured_angle_cdeg;

        if (service->query_function ==
            X42_FUNCTION_REALTIME_POSITION)
        {
            service->query_function = 0U;
            service->next_action_ms =
                now_ms + STEPPER_INTER_FRAME_DELAY_MS;
        }
        if (frame->negative != 0U)
        {
            SetFault(service);
            return;
        }
        measured_angle_cdeg = StepperService_EncoderUnitsToAngleCdeg(
            frame->magnitude);
        if (measured_angle_cdeg > STEPPER_HARD_MAX_ANGLE_CDEG)
        {
            SetFault(service);
            return;
        }
        service->measured_angle_cdeg = measured_angle_cdeg;
        service->position_valid = 1U;
        service->next_query_function = X42_FUNCTION_STATUS;
        CheckArrival(service);
        CheckTrackingFollowError(service, now_ms);
    }
}

void StepperService_ReportRxOverflow(StepperService *service)
{
    service->rx_overflow_count++;
    SetFault(service);
}

bool StepperService_MoveAbsoluteAngleCdeg(
    StepperService *service,
    uint16_t angle_cdeg,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->tracking_enabled != 0U ||
        !PositionParametersValid(angle_cdeg, speed_rpm))
    {
        return false;
    }
    if (service->pending_function != 0U ||
        service->query_function != 0U ||
        service->queued_position_valid != 0U ||
        TimeReached(service->now_ms, service->next_query_ms))
    {
        if (service->pending_function != 0U)
        {
            return false;
        }
        QueuePosition(service, angle_cdeg, speed_rpm,
                      acceleration, 1U);
        service->arrival_pending = 0U;
        return true;
    }
    return SendPosition(service, angle_cdeg, speed_rpm,
                        acceleration, 1U, service->now_ms);
}

bool StepperService_BeginTracking(StepperService *service)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->arrival_pending != 0U)
    {
        return false;
    }
    service->tracking_enabled = 1U;
    service->queued_position_valid = 0U;
    service->tracking_desired_valid = 0U;
    service->tracking_error_active = 0U;
    service->next_tracking_send_ms = service->now_ms;
    service->tracking_grace_deadline_ms =
        service->now_ms + STEPPER_TRACK_GRACE_MS;
    return true;
}

bool StepperService_TrackAbsoluteAngleCdeg(
    StepperService *service,
    uint16_t angle_cdeg,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->tracking_enabled == 0U ||
        !PositionParametersValid(angle_cdeg, speed_rpm))
    {
        return false;
    }

    service->tracking_desired_angle_cdeg = angle_cdeg;
    service->tracking_speed_rpm = speed_rpm;
    service->tracking_acceleration = acceleration;
    service->tracking_desired_valid = 1U;
    return true;
}

bool StepperService_EndTrackingAtLevel(
    StepperService *service,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->tracking_enabled == 0U ||
        !PositionParametersValid(STEPPER_LEVEL_ANGLE_CDEG,
                                 speed_rpm))
    {
        return false;
    }

    service->tracking_enabled = 0U;
    service->tracking_desired_valid = 0U;
    service->tracking_error_active = 0U;
    QueuePosition(service, STEPPER_LEVEL_ANGLE_CDEG,
                  speed_rpm, acceleration, 1U);
    service->arrival_pending = 0U;
    return true;
}

bool StepperService_Stop(StepperService *service)
{
    bool sent = SendStopUntracked(service);

    if (sent)
    {
        service->status = STEPPER_STATUS_STOPPED;
        service->level_phase = STEPPER_LEVEL_PHASE_NONE;
        service->pending_function = 0U;
        service->query_function = 0U;
        service->queued_position_valid = 0U;
        service->arrival_pending = 0U;
        service->tracking_enabled = 0U;
        service->tracking_desired_valid = 0U;
        service->tracking_error_active = 0U;
        service->command_state = STEPPER_COMMAND_NONE;
    }
    return sent;
}

bool StepperService_Disable(StepperService *service)
{
    bool sent = SendEnableUntracked(service, false);

    if (sent)
    {
        service->status = STEPPER_STATUS_STOPPED;
        service->level_phase = STEPPER_LEVEL_PHASE_NONE;
        service->pending_function = 0U;
        service->query_function = 0U;
        service->queued_position_valid = 0U;
        service->arrival_pending = 0U;
        service->tracking_enabled = 0U;
        service->tracking_desired_valid = 0U;
        service->tracking_error_active = 0U;
        service->command_state = STEPPER_COMMAND_NONE;
    }
    return sent;
}

uint32_t StepperService_AngleCdegToPulses(
    uint16_t angle_cdeg)
{
    uint32_t numerator =
        (uint32_t)angle_cdeg *
        STEPPER_PULSES_PER_REVOLUTION;
    return (numerator + (STEPPER_CDEG_PER_REVOLUTION / 2U)) /
           STEPPER_CDEG_PER_REVOLUTION;
}

uint16_t StepperService_EncoderUnitsToAngleCdeg(
    uint32_t encoder_units)
{
    uint64_t numerator =
        (uint64_t)encoder_units * STEPPER_CDEG_PER_REVOLUTION;
    uint64_t denominator = ENCODER_UNITS_PER_REVOLUTION;
    uint64_t result =
        (numerator + (denominator / 2U)) / denominator;
    return result > UINT16_MAX ? UINT16_MAX : (uint16_t)result;
}

StepperStatus StepperService_GetStatus(
    const StepperService *service)
{
    return service->status;
}

uint16_t StepperService_GetCommandedAngleCdeg(
    const StepperService *service)
{
    return service->commanded_angle_cdeg;
}

uint16_t StepperService_GetMeasuredAngleCdeg(
    const StepperService *service)
{
    return service->measured_angle_cdeg;
}

int16_t StepperService_GetRelativeAngleCdeg(
    const StepperService *service)
{
    uint16_t angle = service->position_valid != 0U
        ? service->measured_angle_cdeg
        : service->commanded_angle_cdeg;
    int32_t relative = (int32_t)angle - STEPPER_LEVEL_ANGLE_CDEG;

    if (relative < INT16_MIN)
    {
        return INT16_MIN;
    }
    if (relative > INT16_MAX)
    {
        return INT16_MAX;
    }
    return (int16_t)relative;
}

StepperCommandState StepperService_GetCommandState(
    const StepperService *service)
{
    return service->command_state;
}
