#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "task4_run.h"

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

static VisionState valid_vision(void)
{
    VisionState vision;

    memset(&vision, 0, sizeof(vision));
    vision.valid = 1U;
    vision.x_tenths_mm = 100;
    vision.predicted_x_tenths_mm = 100;
    vision.age_ms = 0U;
    return vision;
}

static void test_start_periodic_pd_and_stale_fault(void)
{
    WriteCapture capture = {0U};
    StepperService stepper;
    Task4Run task;
    VisionState vision = valid_vision();

    StepperService_Init(&stepper, accept_write, &capture);
    stepper.status = STEPPER_STATUS_READY;
    stepper.now_ms = 1000U;
    stepper.next_query_ms = 5000U;
    Task4Run_Init(&task);

    assert(Task4Run_Start(
        &task, &stepper, &vision, 1000U));
    Task4Run_Update(
        &task, &stepper, &vision, 0, 1000U);
    assert(task.update_count == 1U);
    assert(task.angle_command_cdeg == 344);
    assert(capture.count == 0U);
    StepperService_Update(&stepper, 1000U);
    assert(capture.count == 1U);

    Task4Run_Update(
        &task, &stepper, &vision, 0, 1020U);
    assert(task.update_count == 1U);
    Task4Run_Update(
        &task, &stepper, &vision, 0, 1040U);
    assert(task.update_count == 1U);
    vision.sequence++;
    vision.age_ms = 0U;
    Task4Run_Update(
        &task, &stepper, &vision, 300, 1040U);
    assert(task.update_count == 2U);
    assert(task.controller.feedforward_angle_cdeg == -2015);
    vision.age_ms = TASK4_VISION_TIMEOUT_MS + 1U;
    Task4Run_Update(
        &task, &stepper, &vision, 0, 1080U);
    assert(task.state == TASK4_RUN_FAULT);
    assert(task.fault == TASK4_FAULT_VISION_STALE);
}

static void test_stop_returns_to_level_and_start_guards(void)
{
    WriteCapture capture = {0U};
    StepperService stepper;
    Task4Run task;
    VisionState vision = valid_vision();

    StepperService_Init(&stepper, accept_write, &capture);
    stepper.status = STEPPER_STATUS_READY;
    stepper.next_query_ms = 5000U;
    Task4Run_Init(&task);
    assert(Task4Run_Start(&task, &stepper, &vision, 0U));
    assert(Task4Run_Stop(&task, &stepper));
    assert(task.state == TASK4_RUN_STOPPED);
    assert(stepper.queued_angle_cdeg ==
           STEPPER_LEVEL_ANGLE_CDEG);
    assert(stepper.queued_require_arrival);

    Task4Run_Init(&task);
    vision.valid = 0U;
    assert(!Task4Run_Start(
        &task, &stepper, &vision, 0U));
    assert(task.fault == TASK4_FAULT_START_NOT_READY);
}

int main(void)
{
    test_start_periodic_pd_and_stale_fault();
    test_stop_returns_to_level_and_start_guards();
    return 0;
}
