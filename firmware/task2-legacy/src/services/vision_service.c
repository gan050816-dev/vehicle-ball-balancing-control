#include "vision_service.h"

#include <string.h>

static int32_t DivideRounded(int32_t numerator, int32_t denominator)
{
    if (numerator >= 0)
    {
        return (numerator + (denominator / 2)) / denominator;
    }
    return (numerator - (denominator / 2)) / denominator;
}

static int16_t ClampI16(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

void VisionService_Init(VisionService *service)
{
    memset(service, 0, sizeof(*service));
}

uint8_t VisionService_PushFrame(VisionService *service,
                                const K230BallFrame *frame)
{
    uint32_t dt_ms;
    uint32_t velocity_dt_ms;
    uint16_t k230_dt_ms;
    uint16_t diagnostic_code = 0U;
    int32_t raw_velocity;
    int32_t filtered_position;
    int32_t filtered_velocity;
    int32_t delta_x;

    if (frame == 0)
    {
        return 0U;
    }
    if (service->sequence_seen != 0U &&
        frame->sequence == service->last_sequence)
    {
        service->duplicate_frame_count++;
        service->diagnostic_code =
            VISION_DIAG_DUPLICATE_SEQUENCE;
        return 0U;
    }

    service->last_sequence = frame->sequence;
    service->sequence_seen = 1U;
    if (frame->valid == 0U)
    {
        diagnostic_code |= VISION_DIAG_BALL_INVALID;
    }
    if (frame->calibrated == 0U)
    {
        diagnostic_code |= VISION_DIAG_NOT_CALIBRATED;
    }
    if (frame->target_center_valid == 0U)
    {
        diagnostic_code |= VISION_DIAG_TARGET_INVALID;
    }
    if (frame->multiple_candidates != 0U)
    {
        diagnostic_code |= VISION_DIAG_MULTIPLE;
    }
    if (frame->confidence_permille <
        VISION_MIN_CONFIDENCE_PERMILLE)
    {
        diagnostic_code |= VISION_DIAG_LOW_CONFIDENCE;
    }
    service->diagnostic_code = diagnostic_code;
    /* K230 already reports its highest-confidence box. After a clean track
     * exists, MULTIPLE alone is advisory; all other quality gates and the
     * position outlier check below remain mandatory. */
    if (diagnostic_code != 0U &&
        !(diagnostic_code == VISION_DIAG_MULTIPLE &&
          service->initialized != 0U))
    {
        service->rejected_frame_count++;
        return 0U;
    }
    if (service->timestamp_seen != 0U &&
        frame->k230_timestamp_ms16 ==
            service->last_k230_timestamp_ms16)
    {
        service->timestamp_freeze_count++;
        service->rejected_frame_count++;
        service->diagnostic_code =
            VISION_DIAG_TIMESTAMP_FREEZE;
        return 0U;
    }

    if (service->initialized == 0U)
    {
        service->diagnostic_code = 0U;
        service->initialized = 1U;
        service->valid = 1U;
        service->raw_x_tenths_mm = frame->x_tenths_mm;
        service->previous_raw_x_tenths_mm =
            frame->x_tenths_mm;
        service->filtered_x_tenths_mm = frame->x_tenths_mm;
        service->filtered_velocity_tenths_mm_s = 0;
        service->last_frame_ms = frame->received_at_ms;
        service->previous_frame_ms = frame->received_at_ms;
        service->accepted_sample_count = 1U;
        service->last_accepted_sequence = frame->sequence;
        service->last_k230_timestamp_ms16 =
            frame->k230_timestamp_ms16;
        service->previous_k230_timestamp_ms16 =
            frame->k230_timestamp_ms16;
        service->timestamp_seen = 1U;
        service->accepted_frame_count++;
        return 1U;
    }

    dt_ms = frame->received_at_ms - service->last_frame_ms;
    k230_dt_ms = (uint16_t)(
        frame->k230_timestamp_ms16 -
        service->last_k230_timestamp_ms16);
    if ((dt_ms == 0U || dt_ms > 200U) &&
        k230_dt_ms != 0U && k230_dt_ms <= 200U)
    {
        dt_ms = k230_dt_ms;
    }
    if (dt_ms == 0U || dt_ms > 200U)
    {
        service->diagnostic_code = 0U;
        service->valid = 1U;
        service->raw_x_tenths_mm = frame->x_tenths_mm;
        service->previous_raw_x_tenths_mm =
            frame->x_tenths_mm;
        service->filtered_x_tenths_mm = frame->x_tenths_mm;
        service->filtered_velocity_tenths_mm_s = 0;
        service->last_frame_ms = frame->received_at_ms;
        service->previous_frame_ms = frame->received_at_ms;
        service->accepted_sample_count = 1U;
        service->last_accepted_sequence = frame->sequence;
        service->last_k230_timestamp_ms16 =
            frame->k230_timestamp_ms16;
        service->previous_k230_timestamp_ms16 =
            frame->k230_timestamp_ms16;
        service->timestamp_seen = 1U;
        service->accepted_frame_count++;
        return 1U;
    }

    delta_x = (int32_t)frame->x_tenths_mm -
              service->raw_x_tenths_mm;
    if ((int64_t)(delta_x < 0 ? -delta_x : delta_x) * 1000 >
        (int64_t)VISION_MAX_SPEED_TENTHS_MM_S * dt_ms)
    {
        service->outlier_frame_count++;
        service->rejected_frame_count++;
        service->diagnostic_code =
            VISION_DIAG_POSITION_OUTLIER;
        return 0U;
    }

    service->diagnostic_code = 0U;
    filtered_position =
        service->filtered_x_tenths_mm +
        DivideRounded(
            (int32_t)frame->x_tenths_mm -
                service->filtered_x_tenths_mm,
            2);
    filtered_velocity = service->filtered_velocity_tenths_mm_s;
    if (service->accepted_sample_count >= 2U)
    {
        velocity_dt_ms =
            frame->received_at_ms - service->previous_frame_ms;
        if ((velocity_dt_ms == 0U || velocity_dt_ms > 240U))
        {
            velocity_dt_ms = (uint16_t)(
                frame->k230_timestamp_ms16 -
                service->previous_k230_timestamp_ms16);
        }
        if (velocity_dt_ms != 0U && velocity_dt_ms <= 240U)
        {
            raw_velocity = DivideRounded(
                ((int32_t)frame->x_tenths_mm -
                 service->previous_raw_x_tenths_mm) * 1000,
                (int32_t)velocity_dt_ms);
            filtered_velocity =
                service->filtered_velocity_tenths_mm_s +
                DivideRounded(
                    raw_velocity -
                        service->filtered_velocity_tenths_mm_s,
                    2);
        }
    }

    service->valid = 1U;
    service->previous_raw_x_tenths_mm =
        service->raw_x_tenths_mm;
    service->previous_frame_ms = service->last_frame_ms;
    service->raw_x_tenths_mm = frame->x_tenths_mm;
    service->filtered_x_tenths_mm =
        ClampI16(filtered_position);
    service->filtered_velocity_tenths_mm_s =
        ClampI16(filtered_velocity);
    service->last_frame_ms = frame->received_at_ms;
    if (service->accepted_sample_count < 2U)
    {
        service->accepted_sample_count++;
    }
    service->last_accepted_sequence = frame->sequence;
    service->previous_k230_timestamp_ms16 =
        service->last_k230_timestamp_ms16;
    service->last_k230_timestamp_ms16 =
        frame->k230_timestamp_ms16;
    service->timestamp_seen = 1U;
    service->accepted_frame_count++;
    return 1U;
}

VisionState VisionService_GetState(const VisionService *service,
                                   uint32_t now_ms,
                                   uint32_t timeout_ms)
{
    VisionState state;
    int32_t predicted;

    memset(&state, 0, sizeof(state));
    state.age_ms = now_ms - service->last_frame_ms;
    state.sequence = service->last_accepted_sequence;
    state.x_tenths_mm = service->filtered_x_tenths_mm;
    state.velocity_tenths_mm_s =
        service->filtered_velocity_tenths_mm_s;
    predicted = (int32_t)state.x_tenths_mm +
                DivideRounded(
                    (int32_t)state.velocity_tenths_mm_s *
                        VISION_PREDICTION_DELAY_MS,
                    1000);
    if (predicted > K230_BALL_X_ABS_MAX_TENTHS_MM)
    {
        predicted = K230_BALL_X_ABS_MAX_TENTHS_MM;
    }
    else if (predicted < -K230_BALL_X_ABS_MAX_TENTHS_MM)
    {
        predicted = -K230_BALL_X_ABS_MAX_TENTHS_MM;
    }
    state.predicted_x_tenths_mm = (int16_t)predicted;
    state.valid =
        service->initialized != 0U &&
        service->valid != 0U &&
        state.age_ms <= timeout_ms ? 1U : 0U;
    return state;
}

uint16_t VisionService_GetDiagnosticCode(
    const VisionService *service)
{
    if (service == 0 || service->sequence_seen == 0U)
    {
        return VISION_DIAG_NO_FRAME;
    }
    return service->diagnostic_code;
}
