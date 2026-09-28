#include <assert.h>
#include <limits.h>

#include "app_controller.h"
#include "safety_manager.h"

static void confirm_home(AppController *app)
{
    assert(AppController_OnKey(app, APP_KEY_CONFIRM) ==
           APP_ACTION_CONFIRM_HOME);
    assert(AppController_GetState(app) ==
           APP_STATE_MOVING_TO_LEVEL);
    assert(AppController_LevelReached(app));
    assert(AppController_GetState(app) == APP_STATE_MENU);
}

static void test_boot_requires_manual_home_confirmation(void)
{
    AppController app;

    AppController_Init(&app);
    assert(AppController_GetState(&app) == APP_STATE_HOME_REQUIRED);
    assert(AppController_GetTask(&app) == TASK_ID_2_LAP_STOP);
    assert(app.safe_stop_requested);
    assert(!app.capture_valid);
    assert(AppController_OnKey(&app, APP_KEY_SELECT) == APP_ACTION_NONE);
    assert(AppController_GetState(&app) == APP_STATE_HOME_REQUIRED);
}

static void test_level_transition_must_complete_before_menu(void)
{
    AppController app;

    AppController_Init(&app);
    assert(!AppController_LevelReached(&app));
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_CONFIRM_HOME);
    assert(AppController_GetState(&app) ==
           APP_STATE_MOVING_TO_LEVEL);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_NONE);
    assert(AppController_LevelReached(&app));
    assert(AppController_GetState(&app) == APP_STATE_MENU);
    assert(!AppController_LevelReached(&app));
}

static void test_task_two_can_enter_ready_and_run_without_capture(void)
{
    AppController app;

    AppController_Init(&app);
    confirm_home(&app);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_PREPARE_TASK);
    assert(AppController_GetState(&app) == APP_STATE_READY);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_START_TASK);
    assert(AppController_GetState(&app) == APP_STATE_RUNNING);
    assert(!app.safe_stop_requested);
}

static void test_task_three_waits_for_vision_before_start(void)
{
    AppController app;

    AppController_Init(&app);
    confirm_home(&app);
    AppController_OnKey(&app, APP_KEY_SELECT);
    assert(AppController_GetTask(&app) == TASK_ID_3_BALL_MOVE);

    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_PREPARE_TASK);
    assert(AppController_GetState(&app) == APP_STATE_PLACE_BALL);
    assert(AppController_OnKey(&app, APP_KEY_CAPTURE) ==
           APP_ACTION_NONE);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_NONE);
    AppController_AdvanceTime(&app, 30000U);
    assert(AppController_GetElapsedMs(&app) == 0U);
    assert(AppController_GetState(&app) == APP_STATE_PLACE_BALL);
    assert(AppController_VisionReady(&app));
    assert(AppController_GetState(&app) == APP_STATE_READY);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_START_TASK);
    assert(AppController_GetState(&app) == APP_STATE_RUNNING);
}

static void test_tasks_four_and_five_wait_for_fixed_vision(void)
{
    AppController app;

    AppController_Init(&app);
    confirm_home(&app);
    AppController_OnKey(&app, APP_KEY_SELECT);
    AppController_OnKey(&app, APP_KEY_SELECT);
    assert(AppController_GetTask(&app) == TASK_ID_4_TO_B_BALANCE);

    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_PREPARE_TASK);
    assert(AppController_GetState(&app) == APP_STATE_PLACE_BALL);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_NONE);
    assert(AppController_VisionReady(&app));
    assert(AppController_GetState(&app) == APP_STATE_READY);

    AppController_Init(&app);
    confirm_home(&app);
    AppController_OnKey(&app, APP_KEY_SELECT);
    AppController_OnKey(&app, APP_KEY_SELECT);
    AppController_OnKey(&app, APP_KEY_SELECT);
    assert(AppController_GetTask(&app) == TASK_ID_5_LAP_CENTER);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_PREPARE_TASK);
    assert(AppController_GetState(&app) == APP_STATE_PLACE_BALL);
    assert(AppController_VisionReady(&app));
    assert(AppController_GetState(&app) == APP_STATE_READY);
}

static void test_only_task_six_requires_vision_capture(void)
{
    AppController app;
    uint8_t selection;

    AppController_Init(&app);
    confirm_home(&app);
    for (selection = 2U; selection < 6U; ++selection)
    {
        AppController_OnKey(&app, APP_KEY_SELECT);
    }
    assert(AppController_GetTask(&app) == TASK_ID_6_LAP_TARGET);

    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_PREPARE_TASK);
    assert(AppController_GetState(&app) == APP_STATE_CAPTURE_REQUIRED);
    assert(AppController_OnKey(&app, APP_KEY_CAPTURE) ==
           APP_ACTION_NONE);
    assert(AppController_VisionReady(&app));
    assert(AppController_GetState(&app) == APP_STATE_CAPTURE_READY);
    assert(AppController_OnKey(&app, APP_KEY_CAPTURE) ==
           APP_ACTION_REQUEST_CAPTURE);
    assert(AppController_GetState(&app) == APP_STATE_CAPTURE_REQUIRED);
    assert(AppController_CaptureSucceeded(&app));
    assert(AppController_GetState(&app) == APP_STATE_READY);
    assert(app.capture_valid);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_START_TASK);
    assert(AppController_GetState(&app) == APP_STATE_RUNNING);
    assert(!app.safe_stop_requested);

    AppController_PreparationLost(&app);
    assert(AppController_GetState(&app) == APP_STATE_PLACE_BALL);
    assert(app.capture_valid);
    assert(AppController_VisionReady(&app));
    assert(AppController_GetState(&app) == APP_STATE_READY);
}

static void test_lost_preparation_returns_to_unlimited_wait(void)
{
    AppController app;

    AppController_Init(&app);
    confirm_home(&app);
    AppController_OnKey(&app, APP_KEY_SELECT);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_PREPARE_TASK);
    assert(AppController_VisionReady(&app));
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) ==
           APP_ACTION_START_TASK);

    AppController_AdvanceTime(&app, 50U);
    AppController_PreparationLost(&app);
    assert(AppController_GetState(&app) == APP_STATE_PLACE_BALL);
    assert(AppController_GetElapsedMs(&app) == 0U);
    assert(!AppController_IsSafeStopRequested(&app));
    AppController_AdvanceTime(&app, 30000U);
    assert(AppController_GetElapsedMs(&app) == 0U);
}

static void test_reset_is_available_from_running_state(void)
{
    AppController app;

    AppController_Init(&app);
    confirm_home(&app);
    (void)AppController_OnKey(&app, APP_KEY_CONFIRM);
    (void)AppController_OnKey(&app, APP_KEY_CONFIRM);
    assert(AppController_GetState(&app) == APP_STATE_RUNNING);

    assert(AppController_OnKey(&app, APP_KEY_RESET) ==
           APP_ACTION_STOP_AND_RESET);
    assert(AppController_GetState(&app) == APP_STATE_HOME_REQUIRED);
    assert(app.safe_stop_requested);
    assert(!app.capture_valid);
}

static void test_elapsed_time_updates_only_while_running(void)
{
    AppController app;

    AppController_Init(&app);
    AppController_AdvanceTime(&app, 100U);
    assert(AppController_GetElapsedMs(&app) == 0U);

    confirm_home(&app);
    (void)AppController_OnKey(&app, APP_KEY_CONFIRM);
    (void)AppController_OnKey(&app, APP_KEY_CONFIRM);
    AppController_AdvanceTime(&app, 1234U);
    assert(AppController_GetElapsedMs(&app) == 1234U);

    app.run_elapsed_ms = UINT32_MAX - 5U;
    AppController_AdvanceTime(&app, 10U);
    assert(AppController_GetElapsedMs(&app) == UINT32_MAX);

    AppController_TaskFinished(&app);
    assert(AppController_GetState(&app) == APP_STATE_DONE);
    assert(app.safe_stop_requested);
    assert(AppController_IsSafeStopRequested(&app));
}

static void test_done_and_fault_require_safe_restart(void)
{
    AppController app;

    AppController_Init(&app);
    confirm_home(&app);
    (void)AppController_OnKey(&app, APP_KEY_CONFIRM);
    (void)AppController_OnKey(&app, APP_KEY_CONFIRM);
    AppController_TaskFinished(&app);
    assert(AppController_GetState(&app) == APP_STATE_DONE);

    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) == APP_ACTION_NONE);
    assert(AppController_GetState(&app) == APP_STATE_HOME_REQUIRED);

    AppController_TaskFailed(&app);
    assert(AppController_GetState(&app) == APP_STATE_FAULT);
    assert(AppController_OnKey(&app, APP_KEY_CONFIRM) == APP_ACTION_NONE);
    assert(AppController_GetState(&app) == APP_STATE_FAULT);
    assert(AppController_OnKey(&app, APP_KEY_RESET) ==
           APP_ACTION_STOP_AND_RESET);
    assert(AppController_GetState(&app) == APP_STATE_HOME_REQUIRED);
}

static void test_safety_requires_home_before_motion(void)
{
    SafetyManager safety;

    SafetyManager_Init(&safety);
    assert(!safety.home_confirmed);
    assert(!SafetyManager_TryEnableMotion(&safety));

    SafetyManager_ConfirmHome(&safety);
    assert(SafetyManager_TryEnableMotion(&safety));
    assert(safety.motion_allowed);
    assert(SafetyManager_IsMotionAllowed(&safety));

    SafetyManager_RequestStop(&safety, SAFETY_STOP_RESET, true);
    assert(!safety.motion_allowed);
    assert(!SafetyManager_IsMotionAllowed(&safety));
    assert(!safety.home_confirmed);
    assert(safety.stop_reason == SAFETY_STOP_RESET);
}

int main(void)
{
    test_boot_requires_manual_home_confirmation();
    test_level_transition_must_complete_before_menu();
    test_task_two_can_enter_ready_and_run_without_capture();
    test_task_three_waits_for_vision_before_start();
    test_tasks_four_and_five_wait_for_fixed_vision();
    test_only_task_six_requires_vision_capture();
    test_lost_preparation_returns_to_unlimited_wait();
    test_reset_is_available_from_running_state();
    test_elapsed_time_updates_only_while_running();
    test_done_and_fault_require_safe_restart();
    test_safety_requires_home_before_motion();
    return 0;
}
