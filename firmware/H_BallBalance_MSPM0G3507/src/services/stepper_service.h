#ifndef STEPPER_SERVICE_H
#define STEPPER_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "x42_protocol.h"

#define STEPPER_MOTOR_ADDRESS                    2U
#define STEPPER_PULSES_PER_REVOLUTION         3200U
#define STEPPER_CDEG_PER_REVOLUTION           36000UL
#define STEPPER_LEVEL_ANGLE_CDEG               4264U
#define STEPPER_NORMAL_MIN_ANGLE_CDEG           286U
#define STEPPER_NORMAL_MAX_ANGLE_CDEG          8165U
#define STEPPER_HARD_MAX_ANGLE_CDEG            8451U
#define STEPPER_RACK_LEVEL_TENTHS_MM            149U
#define STEPPER_RACK_NORMAL_MIN_TENTHS_MM        10U
#define STEPPER_RACK_NORMAL_MAX_TENTHS_MM       285U
#define STEPPER_RACK_HARD_MAX_TENTHS_MM         295U
#define STEPPER_LEVEL_SPEED_RPM                    5U
#define STEPPER_LEVEL_ACCELERATION                30U
#define STEPPER_TRACKING_COMMAND_PERIOD_MS         80U
#define STEPPER_TRACKING_MIN_DELTA_TENTHS_MM        3U
#define STEPPER_TRACKING_MAX_DELTA_TENTHS_MM       15U
#define STEPPER_TRACKING_GRACE_MS                 500U
#define STEPPER_TRACKING_ERROR_TENTHS_MM           30U
#define STEPPER_TRACKING_ERROR_DURATION_MS        400U
#define STEPPER_AXIS_TRACKING_COMMAND_PERIOD_MS    40U
#define STEPPER_AXIS_TRACKING_MIN_DELTA_CDEG       86U
#define STEPPER_AXIS_TRACKING_MAX_DELTA_CDEG      430U
#define STEPPER_AXIS_TRACKING_ERROR_CDEG          859U

typedef bool (*StepperWriteFunction)(
    void *context, const uint8_t *data, uint8_t length);

typedef enum
{
    STEPPER_STATUS_STOPPED = 0,
    STEPPER_STATUS_LEVELING,
    STEPPER_STATUS_READY,
    STEPPER_STATUS_FAULT
} StepperStatus;

typedef enum
{
    STEPPER_LEVEL_PHASE_NONE = 0,
    STEPPER_LEVEL_PHASE_WAIT_VERSION,
    STEPPER_LEVEL_PHASE_WAIT_STOP_ACK,
    STEPPER_LEVEL_PHASE_WAIT_ENABLE_ACK,
    STEPPER_LEVEL_PHASE_WAIT_CLEAR_ZERO_ACK,
    STEPPER_LEVEL_PHASE_WAIT_MOVE_ACK,
    STEPPER_LEVEL_PHASE_WAIT_ARRIVAL
} StepperLevelPhase;

typedef enum
{
    STEPPER_COMMAND_NONE = 0,
    STEPPER_COMMAND_SENT,
    STEPPER_COMMAND_ACKNOWLEDGED,
    STEPPER_COMMAND_POSITION_REACHED
} StepperCommandState;

typedef struct
{
    StepperWriteFunction write;
    void *write_context;
    StepperStatus status;
    StepperLevelPhase level_phase;
    uint32_t deadline_ms;
    uint32_t next_action_ms;
    uint32_t next_query_ms;
    uint32_t query_deadline_ms;
    uint32_t arrival_deadline_ms;
    uint32_t next_tracking_send_ms;
    uint32_t tracking_grace_deadline_ms;
    uint32_t tracking_error_started_ms;
    uint32_t now_ms;
    uint16_t commanded_tenths_mm;
    uint16_t measured_tenths_mm;
    uint16_t queued_position_tenths_mm;
    uint16_t queued_speed_rpm;
    uint16_t tracking_target_tenths_mm;
    uint16_t tracking_speed_rpm;
    uint16_t commanded_angle_cdeg;
    uint16_t measured_angle_cdeg;
    uint16_t queued_angle_cdeg;
    uint16_t axis_tracking_target_cdeg;
    uint8_t pending_function;
    uint8_t query_function;
    uint8_t next_query_function;
    uint8_t queued_acceleration;
    uint8_t tracking_acceleration;
    uint8_t queued_position_valid;
    uint8_t tracking_target_valid;
    uint8_t tracking_active;
    uint8_t tracking_error_active;
    uint8_t tracking_axis_mode;
    uint8_t queued_angle_valid;
    uint8_t arrival_axis_mode;
    uint8_t status_flags;
    uint8_t version_valid;
    uint8_t position_valid;
    uint8_t position_reached;
    uint8_t arrival_pending;
    uint8_t idle_query_retry_enabled;
    StepperCommandState command_state;
    uint32_t response_timeout_count;
    uint32_t query_timeout_count;
    uint32_t arrival_timeout_count;
    uint32_t protocol_error_count;
    uint32_t rx_overflow_count;
} StepperService;

void StepperService_Init(
    StepperService *service,
    StepperWriteFunction write,
    void *write_context);
bool StepperService_BeginLeveling(
    StepperService *service, uint32_t now_ms);
void StepperService_Update(
    StepperService *service, uint32_t now_ms);
void StepperService_SetIdleQueryRetry(
    StepperService *service, bool enabled);
void StepperService_HandleX42Frame(
    StepperService *service,
    const X42Frame *frame,
    uint32_t now_ms);
void StepperService_ReportRxOverflow(StepperService *service);
bool StepperService_MoveAbsoluteTenthsMm(
    StepperService *service,
    uint16_t position_tenths_mm,
    uint16_t speed_rpm,
    uint8_t acceleration);
bool StepperService_MoveRelativeAngleCentiDegrees(
    StepperService *service,
    int16_t relative_angle_centi_degrees,
    uint16_t speed_rpm,
    uint8_t acceleration);
bool StepperService_BeginTracking(StepperService *service);
bool StepperService_BeginAxisAngleTracking(
    StepperService *service);
bool StepperService_QueueTrackingTargetTenthsMm(
    StepperService *service,
    uint16_t position_tenths_mm,
    uint16_t speed_rpm,
    uint8_t acceleration);
bool StepperService_QueueTrackingTargetAngleCentiDegrees(
    StepperService *service,
    int16_t relative_angle_centi_degrees,
    uint16_t speed_rpm,
    uint8_t acceleration);
void StepperService_EndTracking(StepperService *service);
bool StepperService_TrackAbsoluteAxisAngleCdeg(
    StepperService *service,
    uint16_t angle_cdeg,
    uint16_t speed_rpm,
    uint8_t acceleration);
bool StepperService_EndAxisAngleTrackingAtLevel(
    StepperService *service,
    uint16_t speed_rpm,
    uint8_t acceleration);
bool StepperService_Stop(StepperService *service);
bool StepperService_Disable(StepperService *service);

uint32_t StepperService_PositionTenthsMmToPulses(
    uint16_t position_tenths_mm);
uint16_t StepperService_EncoderUnitsToTenthsMm(
    uint32_t encoder_units);
uint32_t StepperService_AngleCdegToPulses(uint16_t angle_cdeg);
uint16_t StepperService_EncoderUnitsToAngleCdeg(
    uint32_t encoder_units);
StepperStatus StepperService_GetStatus(
    const StepperService *service);
uint16_t StepperService_GetCommandedTenthsMm(
    const StepperService *service);
uint16_t StepperService_GetMeasuredTenthsMm(
    const StepperService *service);
StepperCommandState StepperService_GetCommandState(
    const StepperService *service);
bool StepperService_IsTracking(
    const StepperService *service);
uint16_t StepperService_GetCommandedAngleCdeg(
    const StepperService *service);
uint16_t StepperService_GetMeasuredAngleCdeg(
    const StepperService *service);
int16_t StepperService_GetRelativeAngleCdeg(
    const StepperService *service);

#endif
