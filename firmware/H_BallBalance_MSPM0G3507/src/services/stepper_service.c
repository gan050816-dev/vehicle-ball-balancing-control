#include "stepper_service.h"

#define X42_FRAME_END                         0x6BU
#define X42_DIRECTION_RACK_UP                 0x00U
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
#define STEPPER_POSITION_TOLERANCE_TENTHS_MM      3U

#define RACK_TRAVEL_PER_REVOLUTION_UM        125664UL
#define TENTH_MM_TO_UM                          100UL
#define ENCODER_UNITS_PER_REVOLUTION          65536UL
#define CENTI_DEGREES_PER_REVOLUTION           36000L

static bool StepperService_TimeReached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static int32_t StepperService_DivideRoundedSigned(
    int64_t numerator, int32_t denominator)
{
    if (numerator < 0)
    {
        return (int32_t)(
            -((-numerator + (denominator / 2)) / denominator));
    }
    return (int32_t)(
        (numerator + (denominator / 2)) / denominator);
}

static bool StepperService_RelativeAngleToPositionTenthsMm(
    int16_t relative_angle_centi_degrees,
    uint16_t *position_tenths_mm)
{
    int32_t delta_tenths_mm;
    int32_t position;

    if (position_tenths_mm == 0)
    {
        return false;
    }

    delta_tenths_mm = StepperService_DivideRoundedSigned(
        (int64_t)relative_angle_centi_degrees *
            RACK_TRAVEL_PER_REVOLUTION_UM,
        CENTI_DEGREES_PER_REVOLUTION * TENTH_MM_TO_UM);
    position = (int32_t)STEPPER_RACK_LEVEL_TENTHS_MM +
               delta_tenths_mm;
    if (position <
            (int32_t)STEPPER_RACK_NORMAL_MIN_TENTHS_MM ||
        position >
            (int32_t)STEPPER_RACK_NORMAL_MAX_TENTHS_MM)
    {
        return false;
    }

    *position_tenths_mm = (uint16_t)position;
    return true;
}

static bool StepperService_AxisAngleParametersValid(
    uint16_t angle_cdeg,
    uint16_t speed_rpm)
{
    return angle_cdeg >= STEPPER_NORMAL_MIN_ANGLE_CDEG &&
           angle_cdeg <= STEPPER_NORMAL_MAX_ANGLE_CDEG &&
           speed_rpm != 0U && speed_rpm <= 3000U;
}

static void StepperService_SetFault(StepperService *service)
{
    service->status = STEPPER_STATUS_FAULT;
    service->level_phase = STEPPER_LEVEL_PHASE_NONE;
    service->pending_function = 0U;
    service->query_function = 0U;
    service->arrival_pending = 0U;
    service->queued_position_valid = 0U;
    service->tracking_target_valid = 0U;
    service->tracking_active = 0U;
    service->tracking_error_active = 0U;
    service->tracking_axis_mode = 0U;
    service->queued_angle_valid = 0U;
}

static bool StepperService_Write(
    StepperService *service,
    const uint8_t *data,
    uint8_t length)
{
    if (service->write == 0 ||
        !service->write(service->write_context, data, length))
    {
        StepperService_SetFault(service);
        return false;
    }

    return true;
}

static bool StepperService_SendTracked(
    StepperService *service,
    const uint8_t *data,
    uint8_t length,
    uint8_t function,
    uint32_t now_ms,
    uint32_t action_delay_ms)
{
    if (!StepperService_Write(service, data, length))
    {
        return false;
    }

    service->pending_function = function;
    service->command_state = STEPPER_COMMAND_SENT;
    service->deadline_ms = now_ms + STEPPER_RESPONSE_TIMEOUT_MS;
    service->next_action_ms = now_ms + action_delay_ms;
    return true;
}

static bool StepperService_SendStopTracked(
    StepperService *service, uint32_t now_ms)
{
    static const uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xFEU, 0x98U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };

    return StepperService_SendTracked(
        service, frame, (uint8_t)sizeof(frame), 0xFEU,
        now_ms, STEPPER_ENABLE_DELAY_MS);
}

static bool StepperService_SendStopUntracked(StepperService *service)
{
    static const uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xFEU, 0x98U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };

    return StepperService_Write(
        service, frame, (uint8_t)sizeof(frame));
}

static bool StepperService_SendEnableStateTracked(
    StepperService *service,
    bool enabled,
    uint32_t now_ms,
    uint32_t action_delay_ms)
{
    uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xF3U, 0xABU, 0x00U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };

    frame[3] = enabled ? 0x01U : 0x00U;
    return StepperService_SendTracked(
        service, frame, (uint8_t)sizeof(frame), 0xF3U,
        now_ms, action_delay_ms);
}

static bool StepperService_SendEnableStateUntracked(
    StepperService *service, bool enabled)
{
    uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0xF3U, 0xABU, 0x00U,
        X42_EXECUTE_IMMEDIATELY, X42_FRAME_END
    };

    frame[3] = enabled ? 0x01U : 0x00U;
    return StepperService_Write(
        service, frame, (uint8_t)sizeof(frame));
}

static bool StepperService_SendClearZeroTracked(
    StepperService *service, uint32_t now_ms)
{
    static const uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, 0x0AU, 0x6DU, X42_FRAME_END
    };

    return StepperService_SendTracked(
        service, frame, (uint8_t)sizeof(frame), 0x0AU,
        now_ms, STEPPER_MOVE_DELAY_MS);
}

static void StepperService_PrepareForPosition(
    StepperService *service, bool require_arrival)
{
    service->query_function = 0U;
    service->next_query_function = X42_FUNCTION_STATUS;
    service->position_valid = 0U;
    service->position_reached = 0U;
    service->arrival_pending = require_arrival ? 1U : 0U;
}

static bool StepperService_SendPositionTracked(
    StepperService *service,
    uint16_t position_tenths_mm,
    uint16_t speed_rpm,
    uint8_t acceleration,
    uint32_t now_ms,
    bool require_arrival)
{
    uint32_t pulses =
        StepperService_PositionTenthsMmToPulses(position_tenths_mm);
    uint8_t frame[13];

    frame[0] = STEPPER_MOTOR_ADDRESS;
    frame[1] = 0xFDU;
    frame[2] = X42_DIRECTION_RACK_UP;
    frame[3] = (uint8_t)(speed_rpm >> 8);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(pulses >> 24);
    frame[7] = (uint8_t)(pulses >> 16);
    frame[8] = (uint8_t)(pulses >> 8);
    frame[9] = (uint8_t)pulses;
    frame[10] = X42_POSITION_MODE_ABSOLUTE;
    frame[11] = X42_EXECUTE_IMMEDIATELY;
    frame[12] = X42_FRAME_END;

    StepperService_PrepareForPosition(
        service, require_arrival);
    if (!StepperService_SendTracked(
            service, frame, (uint8_t)sizeof(frame), 0xFDU,
            now_ms, 0U))
    {
        return false;
    }

    service->commanded_tenths_mm = position_tenths_mm;
    service->commanded_angle_cdeg = (uint16_t)(
        ((uint64_t)pulses * STEPPER_CDEG_PER_REVOLUTION +
         (STEPPER_PULSES_PER_REVOLUTION / 2U)) /
        STEPPER_PULSES_PER_REVOLUTION);
    service->arrival_axis_mode = 0U;
    if (require_arrival)
    {
        service->arrival_deadline_ms =
            now_ms + STEPPER_POSITION_ARRIVAL_TIMEOUT_MS;
    }
    service->next_query_ms = now_ms;
    return true;
}

static bool StepperService_SendAxisAngleTracked(
    StepperService *service,
    uint16_t angle_cdeg,
    uint16_t speed_rpm,
    uint8_t acceleration,
    uint32_t now_ms,
    bool require_arrival)
{
    uint32_t pulses =
        StepperService_AngleCdegToPulses(angle_cdeg);
    uint8_t frame[13];

    frame[0] = STEPPER_MOTOR_ADDRESS;
    frame[1] = 0xFDU;
    frame[2] = X42_DIRECTION_RACK_UP;
    frame[3] = (uint8_t)(speed_rpm >> 8);
    frame[4] = (uint8_t)speed_rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(pulses >> 24);
    frame[7] = (uint8_t)(pulses >> 16);
    frame[8] = (uint8_t)(pulses >> 8);
    frame[9] = (uint8_t)pulses;
    frame[10] = X42_POSITION_MODE_ABSOLUTE;
    frame[11] = X42_EXECUTE_IMMEDIATELY;
    frame[12] = X42_FRAME_END;

    StepperService_PrepareForPosition(service, require_arrival);
    if (!StepperService_SendTracked(
            service, frame, (uint8_t)sizeof(frame), 0xFDU,
            now_ms, 0U))
    {
        return false;
    }

    service->commanded_angle_cdeg = angle_cdeg;
    service->arrival_axis_mode = 1U;
    if (require_arrival)
    {
        service->arrival_deadline_ms =
            now_ms + STEPPER_POSITION_ARRIVAL_TIMEOUT_MS;
    }
    service->next_query_ms = now_ms;
    return true;
}

static uint16_t StepperService_AbsDifference(
    uint16_t first, uint16_t second)
{
    return first > second
               ? (uint16_t)(first - second)
               : (uint16_t)(second - first);
}

static uint16_t StepperService_LimitTrackingTarget(
    uint16_t current, uint16_t requested)
{
    if (requested > current)
    {
        uint16_t difference =
            (uint16_t)(requested - current);

        return difference > STEPPER_TRACKING_MAX_DELTA_TENTHS_MM
                   ? (uint16_t)(
                         current +
                         STEPPER_TRACKING_MAX_DELTA_TENTHS_MM)
                   : requested;
    }
    else
    {
        uint16_t difference =
            (uint16_t)(current - requested);

        return difference > STEPPER_TRACKING_MAX_DELTA_TENTHS_MM
                   ? (uint16_t)(
                         current -
                         STEPPER_TRACKING_MAX_DELTA_TENTHS_MM)
                   : requested;
    }
}

static uint16_t StepperService_LimitAxisTrackingTarget(
    uint16_t current, uint16_t requested)
{
    if (requested > current)
    {
        uint16_t difference = (uint16_t)(requested - current);

        return difference > STEPPER_AXIS_TRACKING_MAX_DELTA_CDEG
                   ? (uint16_t)(
                         current +
                         STEPPER_AXIS_TRACKING_MAX_DELTA_CDEG)
                   : requested;
    }
    else
    {
        uint16_t difference = (uint16_t)(current - requested);

        return difference > STEPPER_AXIS_TRACKING_MAX_DELTA_CDEG
                   ? (uint16_t)(
                         current -
                         STEPPER_AXIS_TRACKING_MAX_DELTA_CDEG)
                   : requested;
    }
}

static void StepperService_CheckTrackingError(
    StepperService *service, uint32_t now_ms)
{
    uint16_t difference;

    if (service->tracking_active == 0U ||
        service->position_valid == 0U ||
        !StepperService_TimeReached(
            now_ms, service->tracking_grace_deadline_ms))
    {
        service->tracking_error_active = 0U;
        return;
    }

    if (service->tracking_axis_mode != 0U)
    {
        difference = StepperService_AbsDifference(
            service->measured_angle_cdeg,
            service->commanded_angle_cdeg);
    }
    else
    {
        difference = StepperService_AbsDifference(
            service->measured_tenths_mm,
            service->commanded_tenths_mm);
    }
    if (difference <=
        (service->tracking_axis_mode != 0U
             ? STEPPER_AXIS_TRACKING_ERROR_CDEG
             : STEPPER_TRACKING_ERROR_TENTHS_MM))
    {
        service->tracking_error_active = 0U;
        return;
    }

    if (service->tracking_error_active == 0U)
    {
        service->tracking_error_active = 1U;
        service->tracking_error_started_ms = now_ms;
    }
    else if (StepperService_TimeReached(
                 now_ms,
                 service->tracking_error_started_ms +
                     STEPPER_TRACKING_ERROR_DURATION_MS))
    {
        StepperService_SetFault(service);
    }
}

static bool StepperService_SendQuery(
    StepperService *service,
    uint8_t function,
    uint32_t now_ms)
{
    uint8_t frame[] = {
        STEPPER_MOTOR_ADDRESS, function, X42_FRAME_END
    };

    if (!StepperService_Write(
            service, frame, (uint8_t)sizeof(frame)))
    {
        return false;
    }
    service->query_function = function;
    service->query_deadline_ms =
        now_ms + STEPPER_QUERY_TIMEOUT_MS;
    return true;
}

static void StepperService_CheckPositionArrival(
    StepperService *service)
{
    uint16_t difference;

    if (service->arrival_pending == 0U ||
        service->position_reached == 0U ||
        service->position_valid == 0U)
    {
        return;
    }

    if (service->arrival_axis_mode != 0U)
    {
        difference = StepperService_AbsDifference(
            service->measured_angle_cdeg,
            service->commanded_angle_cdeg);
    }
    else
    {
        difference = StepperService_AbsDifference(
            service->measured_tenths_mm,
            service->commanded_tenths_mm);
    }
    if (difference <=
        (service->arrival_axis_mode != 0U
             ? STEPPER_AXIS_TRACKING_MIN_DELTA_CDEG
             : STEPPER_POSITION_TOLERANCE_TENTHS_MM))
    {
        service->arrival_pending = 0U;
        service->command_state =
            STEPPER_COMMAND_POSITION_REACHED;
        if (service->status == STEPPER_STATUS_LEVELING &&
            service->level_phase ==
                STEPPER_LEVEL_PHASE_WAIT_ARRIVAL)
        {
            service->status = STEPPER_STATUS_READY;
            service->level_phase = STEPPER_LEVEL_PHASE_NONE;
        }
    }
}

void StepperService_Init(
    StepperService *service,
    StepperWriteFunction write,
    void *write_context)
{
    service->write = write;
    service->write_context = write_context;
    service->status = STEPPER_STATUS_STOPPED;
    service->level_phase = STEPPER_LEVEL_PHASE_NONE;
    service->deadline_ms = 0U;
    service->next_action_ms = 0U;
    service->next_query_ms = 0U;
    service->query_deadline_ms = 0U;
    service->arrival_deadline_ms = 0U;
    service->next_tracking_send_ms = 0U;
    service->tracking_grace_deadline_ms = 0U;
    service->tracking_error_started_ms = 0U;
    service->now_ms = 0U;
    service->commanded_tenths_mm = 0U;
    service->measured_tenths_mm = 0U;
    service->queued_position_tenths_mm = 0U;
    service->queued_speed_rpm = 0U;
    service->tracking_target_tenths_mm = 0U;
    service->tracking_speed_rpm = 0U;
    service->commanded_angle_cdeg = 0U;
    service->measured_angle_cdeg = 0U;
    service->queued_angle_cdeg = 0U;
    service->axis_tracking_target_cdeg = 0U;
    service->pending_function = 0U;
    service->query_function = 0U;
    service->next_query_function = X42_FUNCTION_STATUS;
    service->queued_acceleration = 0U;
    service->tracking_acceleration = 0U;
    service->queued_position_valid = 0U;
    service->tracking_target_valid = 0U;
    service->tracking_active = 0U;
    service->tracking_error_active = 0U;
    service->tracking_axis_mode = 0U;
    service->queued_angle_valid = 0U;
    service->arrival_axis_mode = 0U;
    service->status_flags = 0U;
    service->version_valid = 0U;
    service->position_valid = 0U;
    service->position_reached = 0U;
    service->arrival_pending = 0U;
    service->idle_query_retry_enabled = 0U;
    service->command_state = STEPPER_COMMAND_NONE;
    service->response_timeout_count = 0U;
    service->query_timeout_count = 0U;
    service->arrival_timeout_count = 0U;
    service->protocol_error_count = 0U;
    service->rx_overflow_count = 0U;
}

bool StepperService_BeginLeveling(
    StepperService *service, uint32_t now_ms)
{
    if (service->status == STEPPER_STATUS_LEVELING)
    {
        return false;
    }

    service->status = STEPPER_STATUS_LEVELING;
    service->level_phase = STEPPER_LEVEL_PHASE_WAIT_STOP_ACK;
    service->version_valid = 0U;
    service->position_valid = 0U;
    service->position_reached = 0U;
    service->pending_function = 0U;
    service->query_function = 0U;
    service->next_query_function = X42_FUNCTION_STATUS;
    service->queued_position_valid = 0U;
    service->queued_angle_valid = 0U;
    service->tracking_target_valid = 0U;
    service->tracking_active = 0U;
    service->tracking_error_active = 0U;
    service->tracking_axis_mode = 0U;
    service->arrival_pending = 0U;
    service->command_state = STEPPER_COMMAND_NONE;
    service->commanded_tenths_mm = 0U;
    service->now_ms = now_ms;
    /*
     * 当前实物为旧版X42_V1.3 + Emm5.0，不使用X42S第二代
     * 的版本帧作为运动许可门槛。直接从停止命令开始，但仍要求
     * 后续停止、使能、清零和位置命令逐条收到控制应答。
     */
    return StepperService_SendStopTracked(service, now_ms);
}

void StepperService_SetIdleQueryRetry(
    StepperService *service, bool enabled)
{
    uint8_t retry_enabled = enabled ? 1U : 0U;

    if (service->idle_query_retry_enabled == retry_enabled)
    {
        return;
    }

    service->idle_query_retry_enabled = retry_enabled;
    if (retry_enabled == 0U)
    {
        /*
         * 丢弃空闲阶段尚未完成的健康查询，从当前时刻重新建立
         * 严格监测，避免旧查询的期限跨越任务启动边界。
         */
        service->query_function = 0U;
        service->next_action_ms = service->now_ms;
        service->next_query_ms = service->now_ms;
    }
}

void StepperService_Update(
    StepperService *service, uint32_t now_ms)
{
    service->now_ms = now_ms;

    if (service->status == STEPPER_STATUS_FAULT ||
        service->status == STEPPER_STATUS_STOPPED)
    {
        return;
    }

    if (service->pending_function != 0U)
    {
        if (StepperService_TimeReached(
                now_ms, service->deadline_ms))
        {
            service->response_timeout_count++;
            StepperService_SetFault(service);
        }
        return;
    }

    if (service->level_phase == STEPPER_LEVEL_PHASE_WAIT_STOP_ACK)
    {
        if (StepperService_TimeReached(
                now_ms, service->next_action_ms) &&
            StepperService_SendEnableStateTracked(
                service, true, now_ms,
                STEPPER_CLEAR_ZERO_DELAY_MS))
        {
            service->level_phase =
                STEPPER_LEVEL_PHASE_WAIT_ENABLE_ACK;
        }
        return;
    }

    if (service->level_phase ==
        STEPPER_LEVEL_PHASE_WAIT_ENABLE_ACK)
    {
        if (StepperService_TimeReached(
                now_ms, service->next_action_ms) &&
            StepperService_SendClearZeroTracked(service, now_ms))
        {
            service->level_phase =
                STEPPER_LEVEL_PHASE_WAIT_CLEAR_ZERO_ACK;
        }
        return;
    }

    if (service->level_phase ==
        STEPPER_LEVEL_PHASE_WAIT_CLEAR_ZERO_ACK)
    {
        if (StepperService_TimeReached(
                now_ms, service->next_action_ms) &&
            StepperService_SendPositionTracked(
                service,
                STEPPER_RACK_LEVEL_TENTHS_MM,
                STEPPER_LEVEL_SPEED_RPM,
                STEPPER_LEVEL_ACCELERATION,
                now_ms,
                true))
        {
            service->level_phase =
                STEPPER_LEVEL_PHASE_WAIT_MOVE_ACK;
        }
        return;
    }

    if (service->level_phase ==
        STEPPER_LEVEL_PHASE_WAIT_MOVE_ACK)
    {
        service->level_phase =
            STEPPER_LEVEL_PHASE_WAIT_ARRIVAL;
        service->next_query_ms = now_ms;
    }

    if (service->level_phase !=
            STEPPER_LEVEL_PHASE_WAIT_ARRIVAL &&
        service->status != STEPPER_STATUS_READY)
    {
        return;
    }

    if (service->arrival_pending != 0U &&
        StepperService_TimeReached(
            now_ms, service->arrival_deadline_ms))
    {
        service->arrival_timeout_count++;
        StepperService_SetFault(service);
        return;
    }

    if (service->query_function != 0U)
    {
        if (StepperService_TimeReached(
                now_ms, service->query_deadline_ms))
        {
            service->query_timeout_count++;
            if (service->idle_query_retry_enabled != 0U)
            {
                service->query_function = 0U;
                service->next_action_ms =
                    now_ms + STEPPER_INTER_FRAME_DELAY_MS;
                service->next_query_ms =
                    now_ms + STEPPER_QUERY_PERIOD_MS;
            }
            else
            {
                StepperService_SetFault(service);
            }
        }
        return;
    }

    if (service->status == STEPPER_STATUS_READY &&
        service->queued_angle_valid != 0U)
    {
        if (StepperService_TimeReached(
                now_ms, service->next_action_ms))
        {
            uint16_t angle_cdeg = service->queued_angle_cdeg;
            uint16_t speed_rpm = service->queued_speed_rpm;
            uint8_t acceleration = service->queued_acceleration;

            service->queued_angle_valid = 0U;
            (void)StepperService_SendAxisAngleTracked(
                service, angle_cdeg, speed_rpm,
                acceleration, now_ms, true);
        }
        return;
    }

    if (service->status == STEPPER_STATUS_READY &&
        service->queued_position_valid != 0U)
    {
        if (StepperService_TimeReached(
                now_ms, service->next_action_ms))
        {
            uint16_t position_tenths_mm =
                service->queued_position_tenths_mm;
            uint16_t speed_rpm = service->queued_speed_rpm;
            uint8_t acceleration =
                service->queued_acceleration;

            service->queued_position_valid = 0U;
            (void)StepperService_SendPositionTracked(
                service, position_tenths_mm, speed_rpm,
                acceleration, now_ms,
                service->tracking_active == 0U);
        }
        return;
    }

    if (service->status == STEPPER_STATUS_READY &&
        service->tracking_active != 0U &&
        service->tracking_target_valid != 0U &&
        StepperService_TimeReached(
            now_ms, service->next_tracking_send_ms) &&
        StepperService_TimeReached(
            now_ms, service->next_action_ms))
    {
        uint16_t difference;

        if (service->tracking_axis_mode != 0U)
        {
            difference = StepperService_AbsDifference(
                service->axis_tracking_target_cdeg,
                service->commanded_angle_cdeg);
            service->next_tracking_send_ms =
                now_ms +
                STEPPER_AXIS_TRACKING_COMMAND_PERIOD_MS;
            if (difference >=
                STEPPER_AXIS_TRACKING_MIN_DELTA_CDEG)
            {
                uint16_t limited_target =
                    StepperService_LimitAxisTrackingTarget(
                        service->commanded_angle_cdeg,
                        service->axis_tracking_target_cdeg);

                (void)StepperService_SendAxisAngleTracked(
                    service, limited_target,
                    service->tracking_speed_rpm,
                    service->tracking_acceleration,
                    now_ms, false);
                return;
            }
        }
        else
        {
            difference = StepperService_AbsDifference(
                service->tracking_target_tenths_mm,
                service->commanded_tenths_mm);
            service->next_tracking_send_ms =
                now_ms + STEPPER_TRACKING_COMMAND_PERIOD_MS;
            if (difference >=
                STEPPER_TRACKING_MIN_DELTA_TENTHS_MM)
            {
                uint16_t limited_target =
                    StepperService_LimitTrackingTarget(
                        service->commanded_tenths_mm,
                        service->tracking_target_tenths_mm);

                (void)StepperService_SendPositionTracked(
                    service, limited_target,
                    service->tracking_speed_rpm,
                    service->tracking_acceleration,
                    now_ms, false);
                return;
            }
        }
    }

    if (StepperService_TimeReached(now_ms, service->next_query_ms))
    {
        uint8_t function = service->next_query_function;

        if (StepperService_SendQuery(service, function, now_ms))
        {
            service->next_query_ms =
                now_ms + STEPPER_QUERY_PERIOD_MS;
        }
    }
}

void StepperService_HandleX42Frame(
    StepperService *service,
    const X42Frame *frame,
    uint32_t now_ms)
{
    if (frame == 0 || service->status == STEPPER_STATUS_FAULT)
    {
        return;
    }

    service->now_ms = now_ms;
    if (frame->type == X42_FRAME_VERSION)
    {
        service->version_valid = 1U;
        return;
    }

    if (frame->type == X42_FRAME_CONTROL_RESPONSE)
    {
        if (frame->response == X42_RESPONSE_PARAMETER_ERROR ||
            frame->response == X42_RESPONSE_FORMAT_ERROR)
        {
            service->protocol_error_count++;
            StepperService_SetFault(service);
            return;
        }
        if (frame->response == X42_RESPONSE_ACTION_COMPLETE &&
            frame->function == 0xFDU)
        {
            service->position_reached = 1U;
            service->next_query_function =
                X42_FUNCTION_REALTIME_POSITION;
            StepperService_CheckPositionArrival(service);
            return;
        }
        if (frame->function == service->pending_function &&
            frame->response == X42_RESPONSE_RECEIVED)
        {
            service->pending_function = 0U;
            service->command_state =
                STEPPER_COMMAND_ACKNOWLEDGED;
            if (frame->function == 0xFDU)
            {
                service->arrival_deadline_ms =
                    now_ms + STEPPER_POSITION_ARRIVAL_TIMEOUT_MS;
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
        /*
         * 回平速度为5 RPM，低于手册默认8 RPM堵转检测转速。
         * Cgi_TF可能在低速高负载时短暂置位；只把已经持续满足
         * 保护条件的Cgp_TF作为立即故障，普通不到位由超时兜底。
         */
        if ((frame->status_flags &
             X42_STATUS_STALL_PROTECTION) != 0U)
        {
            StepperService_SetFault(service);
            return;
        }
        if (service->status != STEPPER_STATUS_STOPPED &&
            (frame->status_flags & X42_STATUS_ENABLED) == 0U)
        {
            StepperService_SetFault(service);
            return;
        }
        service->position_reached =
            (frame->status_flags &
             X42_STATUS_POSITION_REACHED) != 0U
                ? 1U
                : 0U;
        service->next_query_function =
            service->tracking_active != 0U
                ? X42_FUNCTION_REALTIME_POSITION
                : service->position_reached != 0U
                ? X42_FUNCTION_REALTIME_POSITION
                : X42_FUNCTION_STATUS;
        StepperService_CheckPositionArrival(service);
        return;
    }

    if (frame->type == X42_FRAME_REALTIME_POSITION)
    {
        if (service->query_function ==
            X42_FUNCTION_REALTIME_POSITION)
        {
            service->query_function = 0U;
            service->next_action_ms =
                now_ms + STEPPER_INTER_FRAME_DELAY_MS;
        }
        if (frame->negative != 0U)
        {
            StepperService_SetFault(service);
            return;
        }
        service->measured_tenths_mm =
            StepperService_EncoderUnitsToTenthsMm(
                frame->magnitude);
        service->measured_angle_cdeg =
            StepperService_EncoderUnitsToAngleCdeg(
                frame->magnitude);
        if (service->measured_angle_cdeg >
            STEPPER_HARD_MAX_ANGLE_CDEG)
        {
            StepperService_SetFault(service);
            return;
        }
        service->position_valid = 1U;
        service->next_query_function = X42_FUNCTION_STATUS;
        StepperService_CheckTrackingError(
            service, now_ms);
        StepperService_CheckPositionArrival(service);
    }
}

void StepperService_ReportRxOverflow(StepperService *service)
{
    service->rx_overflow_count++;
    StepperService_SetFault(service);
}

bool StepperService_MoveAbsoluteTenthsMm(
    StepperService *service,
    uint16_t position_tenths_mm,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->tracking_active != 0U ||
        position_tenths_mm < STEPPER_RACK_NORMAL_MIN_TENTHS_MM ||
        position_tenths_mm > STEPPER_RACK_NORMAL_MAX_TENTHS_MM ||
        speed_rpm == 0U || speed_rpm > 3000U)
    {
        return false;
    }

    if (service->pending_function != 0U ||
        service->query_function != 0U ||
        service->queued_position_valid != 0U)
    {
        service->queued_position_tenths_mm =
            position_tenths_mm;
        service->queued_speed_rpm = speed_rpm;
        service->queued_acceleration = acceleration;
        service->queued_position_valid = 1U;
        service->arrival_pending = 0U;
        return true;
    }

    return StepperService_SendPositionTracked(
        service, position_tenths_mm, speed_rpm,
        acceleration, service->now_ms, true);
}

bool StepperService_MoveRelativeAngleCentiDegrees(
    StepperService *service,
    int16_t relative_angle_centi_degrees,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    uint16_t position_tenths_mm;

    return StepperService_RelativeAngleToPositionTenthsMm(
               relative_angle_centi_degrees,
               &position_tenths_mm) &&
           StepperService_MoveAbsoluteTenthsMm(
               service, position_tenths_mm,
               speed_rpm, acceleration);
}

bool StepperService_BeginTracking(StepperService *service)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->level_phase != STEPPER_LEVEL_PHASE_NONE)
    {
        return false;
    }

    service->tracking_active = 1U;
    service->tracking_axis_mode = 0U;
    service->queued_angle_valid = 0U;
    service->tracking_target_valid = 0U;
    service->tracking_error_active = 0U;
    service->arrival_pending = 0U;
    service->next_tracking_send_ms = service->now_ms;
    service->tracking_grace_deadline_ms =
        service->now_ms + STEPPER_TRACKING_GRACE_MS;
    return true;
}

bool StepperService_BeginAxisAngleTracking(
    StepperService *service)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->level_phase != STEPPER_LEVEL_PHASE_NONE ||
        service->arrival_pending != 0U)
    {
        return false;
    }

    service->tracking_active = 1U;
    service->tracking_axis_mode = 1U;
    service->queued_position_valid = 0U;
    service->queued_angle_valid = 0U;
    service->tracking_target_valid = 0U;
    service->tracking_error_active = 0U;
    service->arrival_pending = 0U;
    service->next_tracking_send_ms = service->now_ms;
    service->tracking_grace_deadline_ms =
        service->now_ms + STEPPER_TRACKING_GRACE_MS;
    return true;
}

bool StepperService_QueueTrackingTargetTenthsMm(
    StepperService *service,
    uint16_t position_tenths_mm,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->tracking_active == 0U ||
        position_tenths_mm < STEPPER_RACK_NORMAL_MIN_TENTHS_MM ||
        position_tenths_mm > STEPPER_RACK_NORMAL_MAX_TENTHS_MM ||
        speed_rpm == 0U || speed_rpm > 3000U)
    {
        return false;
    }

    service->tracking_target_tenths_mm =
        position_tenths_mm;
    service->tracking_speed_rpm = speed_rpm;
    service->tracking_acceleration = acceleration;
    service->tracking_target_valid = 1U;
    return true;
}

bool StepperService_QueueTrackingTargetAngleCentiDegrees(
    StepperService *service,
    int16_t relative_angle_centi_degrees,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    uint16_t position_tenths_mm;

    return StepperService_RelativeAngleToPositionTenthsMm(
               relative_angle_centi_degrees,
               &position_tenths_mm) &&
           StepperService_QueueTrackingTargetTenthsMm(
               service, position_tenths_mm,
               speed_rpm, acceleration);
}

void StepperService_EndTracking(StepperService *service)
{
    service->tracking_active = 0U;
    service->tracking_target_valid = 0U;
    service->tracking_error_active = 0U;
    service->tracking_axis_mode = 0U;
}

bool StepperService_TrackAbsoluteAxisAngleCdeg(
    StepperService *service,
    uint16_t angle_cdeg,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->tracking_active == 0U ||
        service->tracking_axis_mode == 0U ||
        !StepperService_AxisAngleParametersValid(
            angle_cdeg, speed_rpm))
    {
        return false;
    }

    service->axis_tracking_target_cdeg = angle_cdeg;
    service->tracking_speed_rpm = speed_rpm;
    service->tracking_acceleration = acceleration;
    service->tracking_target_valid = 1U;
    return true;
}

bool StepperService_EndAxisAngleTrackingAtLevel(
    StepperService *service,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    if (service->status != STEPPER_STATUS_READY ||
        service->tracking_active == 0U ||
        service->tracking_axis_mode == 0U ||
        !StepperService_AxisAngleParametersValid(
            STEPPER_LEVEL_ANGLE_CDEG, speed_rpm))
    {
        return false;
    }

    service->tracking_active = 0U;
    service->tracking_axis_mode = 0U;
    service->tracking_target_valid = 0U;
    service->tracking_error_active = 0U;
    service->queued_position_valid = 0U;
    service->queued_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    service->queued_speed_rpm = speed_rpm;
    service->queued_acceleration = acceleration;
    service->queued_angle_valid = 1U;
    service->arrival_pending = 0U;
    service->command_state = STEPPER_COMMAND_NONE;
    return true;
}

bool StepperService_Stop(StepperService *service)
{
    bool sent = StepperService_SendStopUntracked(service);

    if (sent)
    {
        service->status = STEPPER_STATUS_STOPPED;
        service->level_phase = STEPPER_LEVEL_PHASE_NONE;
        service->pending_function = 0U;
        service->query_function = 0U;
        service->queued_position_valid = 0U;
        service->queued_angle_valid = 0U;
        service->tracking_target_valid = 0U;
        service->tracking_active = 0U;
        service->tracking_error_active = 0U;
        service->tracking_axis_mode = 0U;
        service->arrival_pending = 0U;
        service->command_state = STEPPER_COMMAND_NONE;
    }
    return sent;
}

bool StepperService_Disable(StepperService *service)
{
    bool sent =
        StepperService_SendEnableStateUntracked(service, false);

    if (sent)
    {
        service->status = STEPPER_STATUS_STOPPED;
        service->level_phase = STEPPER_LEVEL_PHASE_NONE;
        service->pending_function = 0U;
        service->query_function = 0U;
        service->queued_position_valid = 0U;
        service->queued_angle_valid = 0U;
        service->tracking_target_valid = 0U;
        service->tracking_active = 0U;
        service->tracking_error_active = 0U;
        service->tracking_axis_mode = 0U;
        service->arrival_pending = 0U;
        service->command_state = STEPPER_COMMAND_NONE;
    }
    return sent;
}

uint32_t StepperService_PositionTenthsMmToPulses(
    uint16_t position_tenths_mm)
{
    uint32_t numerator =
        (uint32_t)position_tenths_mm *
        TENTH_MM_TO_UM *
        STEPPER_PULSES_PER_REVOLUTION;

    return (numerator + (RACK_TRAVEL_PER_REVOLUTION_UM / 2U)) /
           RACK_TRAVEL_PER_REVOLUTION_UM;
}

uint16_t StepperService_EncoderUnitsToTenthsMm(
    uint32_t encoder_units)
{
    uint64_t numerator =
        (uint64_t)encoder_units *
        RACK_TRAVEL_PER_REVOLUTION_UM;
    uint64_t denominator =
        (uint64_t)ENCODER_UNITS_PER_REVOLUTION *
        TENTH_MM_TO_UM;
    uint64_t result =
        (numerator + (denominator / 2U)) / denominator;

    return result > UINT16_MAX ? UINT16_MAX : (uint16_t)result;
}

uint32_t StepperService_AngleCdegToPulses(uint16_t angle_cdeg)
{
    uint32_t numerator =
        (uint32_t)angle_cdeg * STEPPER_PULSES_PER_REVOLUTION;

    return (numerator + (STEPPER_CDEG_PER_REVOLUTION / 2U)) /
           STEPPER_CDEG_PER_REVOLUTION;
}

uint16_t StepperService_EncoderUnitsToAngleCdeg(
    uint32_t encoder_units)
{
    uint64_t numerator =
        (uint64_t)encoder_units * STEPPER_CDEG_PER_REVOLUTION;
    uint64_t result =
        (numerator + (ENCODER_UNITS_PER_REVOLUTION / 2U)) /
        ENCODER_UNITS_PER_REVOLUTION;

    return result > UINT16_MAX ? UINT16_MAX : (uint16_t)result;
}

StepperStatus StepperService_GetStatus(
    const StepperService *service)
{
    return service->status;
}

uint16_t StepperService_GetCommandedTenthsMm(
    const StepperService *service)
{
    return service->commanded_tenths_mm;
}

uint16_t StepperService_GetMeasuredTenthsMm(
    const StepperService *service)
{
    return service->measured_tenths_mm;
}

StepperCommandState StepperService_GetCommandState(
    const StepperService *service)
{
    return service->command_state;
}

bool StepperService_IsTracking(
    const StepperService *service)
{
    return service->tracking_active != 0U;
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
    int32_t relative =
        (int32_t)angle - STEPPER_LEVEL_ANGLE_CDEG;

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
