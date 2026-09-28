#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "balance_drive_run.h"

typedef struct
{
    uint8_t count;
} WriteCapture;

static bool accept_write(void *context,
                         const uint8_t *data,
                         uint8_t length)
{
    WriteCapture *capture = (WriteCapture *)context;

    (void)data;
    (void)length;
    ++capture->count;
    return true;
}

static BalanceVisionState valid_vision(void)
{
    BalanceVisionState vision;

    memset(&vision, 0, sizeof(vision));
    vision.valid = 1U;
    vision.x_tenths_mm = 100;
    vision.predicted_x_tenths_mm = 100;
    return vision;
}

static void test_periodic_pd_and_stale_fault(void)
{
    WriteCapture capture = {0U};
    StepperService stepper;
    BalanceDriveRun run;
    BalanceVisionState vision = valid_vision();

    StepperService_Init(&stepper, accept_write, &capture);
    stepper.status = STEPPER_STATUS_READY;
    stepper.commanded_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    stepper.measured_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    stepper.position_valid = 1U;
    stepper.now_ms = 1000U;
    stepper.next_query_ms = 5000U;
    BalanceDriveRun_Init(&run);

    assert(BalanceDriveRun_Start(
        &run, &stepper, &vision, 1000U));
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1000U);
    assert(run.update_count == 1U);
    assert(run.angle_command_cdeg == 344);
    StepperService_Update(&stepper, 1000U);
    assert(capture.count == 1U);

    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1020U);
    assert(run.update_count == 1U);
    vision.sequence++;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 300, 1040U);
    assert(run.update_count == 2U);
    assert(run.controller.feedforward_angle_cdeg == -2015);
    vision.age_ms = BALANCE_DRIVE_VISION_TIMEOUT_MS + 1U;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1080U);
    assert(run.state == BALANCE_DRIVE_FAULT);
    assert(run.fault == BALANCE_DRIVE_FAULT_VISION_STALE);
}

static void test_stop_queues_level_angle(void)
{
    WriteCapture capture = {0U};
    StepperService stepper;
    BalanceDriveRun run;
    BalanceVisionState vision = valid_vision();

    StepperService_Init(&stepper, accept_write, &capture);
    stepper.status = STEPPER_STATUS_READY;
    stepper.commanded_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    stepper.now_ms = 0U;
    stepper.next_query_ms = 5000U;
    BalanceDriveRun_Init(&run);
    assert(BalanceDriveRun_Start(&run, &stepper, &vision, 0U));
    assert(BalanceDriveRun_Stop(&run, &stepper));
    assert(run.state == BALANCE_DRIVE_STOPPED);
    assert(stepper.queued_angle_valid);
    assert(stepper.queued_angle_cdeg == STEPPER_LEVEL_ANGLE_CDEG);
}

static void test_startup_acceleration_preview_is_bounded_and_one_shot(void)
{
    WriteCapture capture = {0U};
    StepperService stepper;
    BalanceDriveRun run;
    BalanceVisionState vision = valid_vision();

    vision.x_tenths_mm = 0;
    vision.predicted_x_tenths_mm = 0;
    StepperService_Init(&stepper, accept_write, &capture);
    stepper.status = STEPPER_STATUS_READY;
    stepper.commanded_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    stepper.measured_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    stepper.position_valid = 1U;
    stepper.now_ms = 1000U;
    stepper.next_query_ms = 5000U;
    BalanceDriveRun_Init(&run);

    assert(BalanceDriveRun_Start(
        &run, &stepper, &vision, 1000U));
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1000U);
    assert(run.feedforward_acceleration_mm_s2 == 0);
    assert(run.startup_preview_active != 0U);

    vision.sequence++;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 32, 1040U);
    assert(run.feedforward_acceleration_mm_s2 == 96);
    assert(run.controller.feedforward_angle_cdeg == -645);
    assert(run.angle_command_cdeg == -430);

    vision.sequence++;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 64, 1080U);
    assert(run.feedforward_acceleration_mm_s2 == 128);
    assert(run.controller.feedforward_angle_cdeg == -860);
    assert(run.angle_command_cdeg == -860);

    vision.sequence++;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 32, 1120U);
    assert(run.feedforward_acceleration_mm_s2 == 0);
    assert(run.startup_preview_active != 0U);

    vision.sequence++;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1160U);
    assert(run.startup_preview_active == 0U);

    vision.sequence++;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 32, 1200U);
    assert(run.feedforward_acceleration_mm_s2 == 32);
    assert(run.controller.feedforward_angle_cdeg == -215);
}

static void test_tracks_positive_and_negative_extrema_without_faulting(void)
{
    WriteCapture capture = {0U};
    StepperService stepper;
    BalanceDriveRun run;
    BalanceVisionState vision = valid_vision();

    vision.x_tenths_mm = 0;
    vision.predicted_x_tenths_mm = 0;
    StepperService_Init(&stepper, accept_write, &capture);
    stepper.status = STEPPER_STATUS_READY;
    stepper.commanded_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    stepper.measured_angle_cdeg = STEPPER_LEVEL_ANGLE_CDEG;
    stepper.position_valid = 1U;
    stepper.now_ms = 1000U;
    stepper.next_query_ms = 5000U;
    BalanceDriveRun_Init(&run);

    assert(BalanceDriveRun_Start(
        &run, &stepper, &vision, 1000U));
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1000U);
    assert(run.maximum_positive_x_tenths_mm == 0);
    assert(run.maximum_negative_x_tenths_mm == 0);

    vision.sequence++;
    vision.x_tenths_mm = 137;
    vision.predicted_x_tenths_mm = 137;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1040U);
    assert(run.maximum_positive_x_tenths_mm == 137);
    assert(run.maximum_negative_x_tenths_mm == 0);
    assert(run.state == BALANCE_DRIVE_ACTIVE);

    vision.sequence++;
    vision.x_tenths_mm = -164;
    vision.predicted_x_tenths_mm = -164;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1080U);
    assert(run.maximum_positive_x_tenths_mm == 137);
    assert(run.maximum_negative_x_tenths_mm == -164);
    assert(run.state == BALANCE_DRIVE_ACTIVE);

    vision.sequence++;
    vision.x_tenths_mm = 20;
    vision.predicted_x_tenths_mm = 20;
    BalanceDriveRun_Update(
        &run, &stepper, &vision, 0, 1120U);
    assert(run.maximum_positive_x_tenths_mm == 137);
    assert(run.maximum_negative_x_tenths_mm == -164);
    assert(run.fault == BALANCE_DRIVE_FAULT_NONE);
}

int main(void)
{
    test_periodic_pd_and_stale_fault();
    test_stop_queues_level_angle();
    test_startup_acceleration_preview_is_bounded_and_one_shot();
    test_tracks_positive_and_negative_extrema_without_faulting();
    return 0;
}
