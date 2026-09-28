#include "vision_service.h"

static int32_t VisionService_AbsInt32(int32_t value)
{
    return value < 0 ? -(value + 1) + 1 : value;
}

void VisionService_Init(VisionService *service)
{
    service->x_filtered_tenths_mm = 0;
    service->previous_raw_x_tenths_mm = 0;
    service->two_samples_ago_raw_x_tenths_mm = 0;
    service->velocity_filtered_tenths_mm_per_s = 0;
    service->last_valid_received_at_ms = 0U;
    service->previous_raw_received_at_ms = 0U;
    service->two_samples_ago_received_at_ms = 0U;
    service->invalid_sample_count = 0U;
    service->rejected_velocity_count = 0U;
    service->duplicate_sample_count = 0U;
    service->last_sequence = 0U;
    service->accepted_sample_count = 0U;
    service->sequence_seen = false;
    service->state_valid = false;
    service->velocity_valid = false;
}

bool VisionService_PushSample(
    VisionService *service, const VisionSample *sample)
{
    int32_t raw_velocity = 0;
    bool raw_velocity_valid = false;

    if (sample == 0)
    {
        return false;
    }

    if (service->sequence_seen &&
        sample->sequence == service->last_sequence)
    {
        service->duplicate_sample_count++;
        return false;
    }
    service->sequence_seen = true;
    service->last_sequence = sample->sequence;

    if (!sample->valid)
    {
        service->invalid_sample_count++;
        return false;
    }

    if (service->accepted_sample_count >=
        VISION_VELOCITY_WINDOW_SAMPLES)
    {
        uint32_t delta_ms =
            sample->received_at_ms -
            service->two_samples_ago_received_at_ms;

        if (delta_ms != 0U &&
            (int32_t)delta_ms > 0)
        {
            int32_t delta_x =
                (int32_t)sample->x_tenths_mm -
                (int32_t)service->two_samples_ago_raw_x_tenths_mm;

            raw_velocity =
                (delta_x * 1000) / (int32_t)delta_ms;
            raw_velocity_valid = true;
        }
    }

    if (raw_velocity_valid &&
        VisionService_AbsInt32(raw_velocity) >
            VISION_MAX_ABS_VELOCITY_TENTHS_MM_PER_S)
    {
        service->rejected_velocity_count++;
        return false;
    }

    if (!service->state_valid)
    {
        service->x_filtered_tenths_mm = sample->x_tenths_mm;
    }
    else
    {
        service->x_filtered_tenths_mm =
            (int16_t)(((int32_t)service->x_filtered_tenths_mm +
                       (int32_t)sample->x_tenths_mm) /
                      2);
    }

    if (raw_velocity_valid)
    {
        if (service->velocity_valid)
        {
            service->velocity_filtered_tenths_mm_per_s =
                (service->velocity_filtered_tenths_mm_per_s +
                 raw_velocity) /
                2;
        }
        else
        {
            service->velocity_filtered_tenths_mm_per_s =
                raw_velocity;
        }
        service->velocity_valid = true;
    }

    service->two_samples_ago_raw_x_tenths_mm =
        service->previous_raw_x_tenths_mm;
    service->two_samples_ago_received_at_ms =
        service->previous_raw_received_at_ms;
    service->previous_raw_x_tenths_mm = sample->x_tenths_mm;
    service->previous_raw_received_at_ms =
        sample->received_at_ms;
    if (service->accepted_sample_count <
        VISION_VELOCITY_WINDOW_SAMPLES)
    {
        service->accepted_sample_count++;
    }
    service->last_valid_received_at_ms =
        sample->received_at_ms;
    service->state_valid = true;
    return true;
}

bool VisionService_GetState(
    const VisionService *service,
    uint32_t now_ms,
    VisionState *state)
{
    if (state == 0 || !service->state_valid ||
        now_ms - service->last_valid_received_at_ms >
            VISION_STALE_TIMEOUT_MS)
    {
        return false;
    }

    state->valid = true;
    state->velocity_valid = service->velocity_valid;
    state->x_filtered_tenths_mm =
        service->x_filtered_tenths_mm;
    state->velocity_filtered_tenths_mm_per_s =
        service->velocity_filtered_tenths_mm_per_s;
    state->received_at_ms =
        service->last_valid_received_at_ms;
    state->sequence = service->last_sequence;
    return true;
}
