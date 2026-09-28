#include <assert.h>
#include <string.h>

#include "balance_vision_service.h"

static K230BallFrame make_frame(uint8_t sequence,
                                int16_t x_tenths_mm,
                                uint32_t now_ms)
{
    K230BallFrame frame;

    memset(&frame, 0, sizeof(frame));
    frame.valid = 1U;
    frame.calibrated = 1U;
    frame.target_center_valid = 1U;
    frame.sequence = sequence;
    frame.x_tenths_mm = x_tenths_mm;
    frame.confidence_permille = 800U;
    frame.received_at_ms = now_ms;
    frame.k230_timestamp_ms16 = (uint16_t)now_ms;
    return frame;
}

static void test_filter_velocity_duplicate_and_timeout(void)
{
    BalanceVisionService service;
    BalanceVisionState state;
    K230BallFrame frame;

    BalanceVisionService_Init(&service);
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           BALANCE_VISION_DIAG_NO_FRAME);
    frame = make_frame(1U, 0, 100U);
    assert(BalanceVisionService_PushFrame(&service, &frame));
    state = BalanceVisionService_GetState(&service, 100U, 120U);
    assert(state.valid);
    assert(state.x_tenths_mm == 0);

    frame = make_frame(2U, 40, 140U);
    assert(BalanceVisionService_PushFrame(&service, &frame));
    state = BalanceVisionService_GetState(&service, 140U, 120U);
    assert(state.x_tenths_mm == 20);

    frame = make_frame(3U, 80, 180U);
    assert(BalanceVisionService_PushFrame(&service, &frame));
    state = BalanceVisionService_GetState(&service, 180U, 120U);
    assert(state.x_tenths_mm == 50);
    assert(state.velocity_tenths_mm_s == 500);
    assert(state.predicted_x_tenths_mm == 110);

    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(service.duplicate_frame_count == 1U);
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           BALANCE_VISION_DIAG_DUPLICATE_SEQUENCE);
    state = BalanceVisionService_GetState(&service, 301U, 120U);
    assert(!state.valid);
    assert(state.age_ms == 121U);
}

static void test_quality_outlier_timestamp_and_long_gap(void)
{
    BalanceVisionService service;
    BalanceVisionState state;
    K230BallFrame frame;

    BalanceVisionService_Init(&service);
    frame = make_frame(1U, 10, 10U);
    assert(BalanceVisionService_PushFrame(&service, &frame));

    frame = make_frame(2U, 20, 50U);
    frame.multiple_candidates = 1U;
    assert(BalanceVisionService_PushFrame(&service, &frame));
    state = BalanceVisionService_GetState(&service, 50U, 120U);
    assert(state.valid);
    assert(state.x_tenths_mm == 15);

    frame = make_frame(3U, 1000, 90U);
    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(service.outlier_frame_count == 1U);
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           BALANCE_VISION_DIAG_POSITION_OUTLIER);

    frame = make_frame(4U, 20, 100U);
    frame.k230_timestamp_ms16 = 50U;
    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(service.timestamp_freeze_count == 1U);
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           BALANCE_VISION_DIAG_TIMESTAMP_FREEZE);

    frame = make_frame(5U, -30, 300U);
    assert(BalanceVisionService_PushFrame(&service, &frame));
    state = BalanceVisionService_GetState(&service, 300U, 120U);
    assert(state.valid);
    assert(state.x_tenths_mm == -30);

    frame = make_frame(6U, 0, 340U);
    frame.confidence_permille =
        BALANCE_VISION_MIN_CONFIDENCE_PERMILLE - 1U;
    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           BALANCE_VISION_DIAG_LOW_CONFIDENCE);
    assert(service.rejected_frame_count == 3U);
}

static void test_multiple_candidate_requires_established_track(void)
{
    BalanceVisionService service;
    BalanceVisionState state;
    K230BallFrame frame;

    BalanceVisionService_Init(&service);
    frame = make_frame(1U, 0, 100U);
    frame.multiple_candidates = 1U;
    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(!service.initialized);
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           BALANCE_VISION_DIAG_MULTIPLE);

    frame = make_frame(2U, 0, 140U);
    assert(BalanceVisionService_PushFrame(&service, &frame));
    frame = make_frame(3U, 20, 180U);
    frame.multiple_candidates = 1U;
    assert(BalanceVisionService_PushFrame(&service, &frame));
    state = BalanceVisionService_GetState(&service, 180U, 120U);
    assert(state.valid);
    assert(state.sequence == 3U);
    assert(state.x_tenths_mm == 10);

    frame = make_frame(4U, 30, 220U);
    frame.multiple_candidates = 1U;
    frame.confidence_permille =
        BALANCE_VISION_MIN_CONFIDENCE_PERMILLE - 1U;
    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           (BALANCE_VISION_DIAG_MULTIPLE |
            BALANCE_VISION_DIAG_LOW_CONFIDENCE));

    frame = make_frame(5U, 1000, 260U);
    frame.multiple_candidates = 1U;
    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           BALANCE_VISION_DIAG_POSITION_OUTLIER);
}

static void test_combined_fixed_origin_flags(void)
{
    BalanceVisionService service;
    K230BallFrame frame;

    BalanceVisionService_Init(&service);
    frame = make_frame(1U, 0, 100U);
    frame.calibrated = 0U;
    frame.target_center_valid = 0U;
    assert(!BalanceVisionService_PushFrame(&service, &frame));
    assert(BalanceVisionService_GetDiagnosticCode(&service) ==
           (BALANCE_VISION_DIAG_NOT_CALIBRATED |
            BALANCE_VISION_DIAG_TARGET_INVALID));
}

int main(void)
{
    test_filter_velocity_duplicate_and_timeout();
    test_quality_outlier_timestamp_and_long_gap();
    test_multiple_candidate_requires_established_track();
    test_combined_fixed_origin_flags();
    return 0;
}
