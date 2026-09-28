#include <assert.h>
#include <stdint.h>

#include "vision_service.h"

static VisionSample make_sample(
    uint8_t sequence,
    int16_t x_tenths_mm,
    uint32_t received_at_ms)
{
    VisionSample sample;

    sample.valid = true;
    sample.sequence = sequence;
    sample.x_tenths_mm = x_tenths_mm;
    sample.received_at_ms = received_at_ms;
    return sample;
}

static void test_filters_position_and_two_frame_velocity(void)
{
    VisionService service;
    VisionState state;
    VisionSample sample;

    VisionService_Init(&service);

    sample = make_sample(1U, 0, 1000U);
    assert(VisionService_PushSample(&service, &sample));
    assert(VisionService_GetState(&service, 1000U, &state));
    assert(state.x_filtered_tenths_mm == 0);
    assert(!state.velocity_valid);

    sample = make_sample(2U, 100, 1040U);
    assert(VisionService_PushSample(&service, &sample));
    assert(VisionService_GetState(&service, 1040U, &state));
    assert(state.x_filtered_tenths_mm == 50);
    assert(!state.velocity_valid);

    sample = make_sample(3U, 200, 1080U);
    assert(VisionService_PushSample(&service, &sample));
    assert(VisionService_GetState(&service, 1080U, &state));
    assert(state.x_filtered_tenths_mm == 125);
    assert(state.velocity_valid);
    assert(state.velocity_filtered_tenths_mm_per_s == 2500);

    sample = make_sample(4U, 300, 1120U);
    assert(VisionService_PushSample(&service, &sample));
    assert(VisionService_GetState(&service, 1120U, &state));
    assert(state.x_filtered_tenths_mm == 212);
    assert(state.velocity_filtered_tenths_mm_per_s == 2500);
}

static void test_rejects_duplicate_invalid_and_implausible_samples(void)
{
    VisionService service;
    VisionState state;
    VisionSample sample;

    VisionService_Init(&service);
    sample = make_sample(10U, 0, 0U);
    assert(VisionService_PushSample(&service, &sample));

    assert(!VisionService_PushSample(&service, &sample));
    assert(service.duplicate_sample_count == 1U);

    sample = make_sample(11U, 10, 40U);
    sample.valid = false;
    assert(!VisionService_PushSample(&service, &sample));
    assert(service.invalid_sample_count == 1U);

    sample = make_sample(12U, 10, 40U);
    assert(VisionService_PushSample(&service, &sample));
    sample = make_sample(13U, 1000, 80U);
    assert(!VisionService_PushSample(&service, &sample));
    assert(service.rejected_velocity_count == 1U);

    assert(VisionService_GetState(&service, 160U, &state));
    assert(!VisionService_GetState(&service, 161U, &state));
}

static void test_timestamp_wrap_keeps_velocity_valid(void)
{
    VisionService service;
    VisionState state;
    VisionSample sample;

    VisionService_Init(&service);
    sample = make_sample(254U, 0, UINT32_MAX - 39U);
    assert(VisionService_PushSample(&service, &sample));
    sample = make_sample(255U, 40, 0U);
    assert(VisionService_PushSample(&service, &sample));
    sample = make_sample(0U, 80, 40U);
    assert(VisionService_PushSample(&service, &sample));

    assert(VisionService_GetState(&service, 40U, &state));
    assert(state.velocity_valid);
    assert(state.velocity_filtered_tenths_mm_per_s == 1000);
}

int main(void)
{
    test_filters_position_and_two_frame_velocity();
    test_rejects_duplicate_invalid_and_implausible_samples();
    test_timestamp_wrap_keeps_velocity_valid();
    return 0;
}
