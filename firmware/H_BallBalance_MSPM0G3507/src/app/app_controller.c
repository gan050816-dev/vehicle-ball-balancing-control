#include "app_controller.h"

#include <limits.h>

static void AppController_ResetToHome(AppController *app)
{
    app->state = APP_STATE_HOME_REQUIRED;
    app->capture_valid = false;
    app->safe_stop_requested = true;
    app->run_elapsed_ms = 0U;
}

static bool AppController_TaskUsesFixedVision(TaskId task)
{
    return task == TASK_ID_3_BALL_MOVE ||
           task == TASK_ID_4_TO_B_BALANCE ||
           task == TASK_ID_5_LAP_CENTER;
}

void AppController_Init(AppController *app)
{
    TaskMenu_Init(&app->menu);
    AppController_ResetToHome(app);
}

AppAction AppController_OnKey(AppController *app, AppKey key)
{
    if (key == APP_KEY_RESET)
    {
        AppController_ResetToHome(app);
        return APP_ACTION_STOP_AND_RESET;
    }

    switch (app->state)
    {
        case APP_STATE_HOME_REQUIRED:
            if (key == APP_KEY_CONFIRM)
            {
                app->state = APP_STATE_MOVING_TO_LEVEL;
                app->safe_stop_requested = false;
                return APP_ACTION_CONFIRM_HOME;
            }
            break;

        case APP_STATE_MOVING_TO_LEVEL:
            break;

        case APP_STATE_MENU:
            if (key == APP_KEY_SELECT)
            {
                TaskMenu_SelectNext(&app->menu);
            }
            else if (key == APP_KEY_CONFIRM)
            {
                TaskId task = TaskMenu_GetSelection(&app->menu);

                app->capture_valid = false;
                if (AppController_TaskUsesFixedVision(task))
                {
                    app->state = APP_STATE_PLACE_BALL;
                }
                else
                {
                    app->state = TaskMenu_RequiresCapture(task)
                                     ? APP_STATE_CAPTURE_REQUIRED
                                     : APP_STATE_READY;
                }
                return APP_ACTION_PREPARE_TASK;
            }
            break;

        case APP_STATE_PLACE_BALL:
            break;

        case APP_STATE_CAPTURE_REQUIRED:
            break;

        case APP_STATE_CAPTURE_READY:
            if (key == APP_KEY_CAPTURE)
            {
                app->state = APP_STATE_CAPTURE_REQUIRED;
                return APP_ACTION_REQUEST_CAPTURE;
            }
            break;

        case APP_STATE_READY:
            if (key == APP_KEY_CONFIRM)
            {
                app->state = APP_STATE_RUNNING;
                app->safe_stop_requested = false;
                app->run_elapsed_ms = 0U;
                return APP_ACTION_START_TASK;
            }
            break;

        case APP_STATE_DONE:
            if (key == APP_KEY_CONFIRM)
            {
                AppController_ResetToHome(app);
            }
            break;

        case APP_STATE_RUNNING:
        case APP_STATE_FAULT:
        default:
            break;
    }

    return APP_ACTION_NONE;
}

bool AppController_LevelReached(AppController *app)
{
    if (app->state != APP_STATE_MOVING_TO_LEVEL)
    {
        return false;
    }

    app->state = APP_STATE_MENU;
    return true;
}

bool AppController_VisionReady(AppController *app)
{
    TaskId task = TaskMenu_GetSelection(&app->menu);

    if (app->state == APP_STATE_PLACE_BALL &&
        (AppController_TaskUsesFixedVision(task) ||
         (task == TASK_ID_6_LAP_TARGET && app->capture_valid)))
    {
        app->state = APP_STATE_READY;
        return true;
    }
    if (app->state == APP_STATE_CAPTURE_REQUIRED &&
        task == TASK_ID_6_LAP_TARGET && !app->capture_valid)
    {
        app->state = APP_STATE_CAPTURE_READY;
        return true;
    }
    return false;
}

bool AppController_CaptureSucceeded(AppController *app)
{
    if (app->state != APP_STATE_CAPTURE_REQUIRED ||
        TaskMenu_GetSelection(&app->menu) !=
            TASK_ID_6_LAP_TARGET)
    {
        return false;
    }

    app->capture_valid = true;
    app->state = APP_STATE_READY;
    return true;
}

void AppController_PreparationLost(AppController *app)
{
    TaskId task = TaskMenu_GetSelection(&app->menu);

    if (app->state != APP_STATE_RUNNING &&
        app->state != APP_STATE_READY &&
        app->state != APP_STATE_CAPTURE_READY)
    {
        return;
    }
    if (AppController_TaskUsesFixedVision(task) ||
        (task == TASK_ID_6_LAP_TARGET && app->capture_valid))
    {
        app->state = APP_STATE_PLACE_BALL;
    }
    else if (task == TASK_ID_6_LAP_TARGET)
    {
        app->state = APP_STATE_CAPTURE_REQUIRED;
    }
    else
    {
        return;
    }
    app->safe_stop_requested = false;
    app->run_elapsed_ms = 0U;
}

void AppController_TaskFinished(AppController *app)
{
    if (app->state == APP_STATE_RUNNING)
    {
        app->state = APP_STATE_DONE;
        app->safe_stop_requested = true;
    }
}

void AppController_TaskFailed(AppController *app)
{
    app->state = APP_STATE_FAULT;
    app->safe_stop_requested = true;
}

void AppController_AdvanceTime(AppController *app, uint32_t elapsed_ms)
{
    if (app->state != APP_STATE_RUNNING)
    {
        return;
    }

    if (UINT32_MAX - app->run_elapsed_ms < elapsed_ms)
    {
        app->run_elapsed_ms = UINT32_MAX;
    }
    else
    {
        app->run_elapsed_ms += elapsed_ms;
    }
}

AppState AppController_GetState(const AppController *app)
{
    return app->state;
}

TaskId AppController_GetTask(const AppController *app)
{
    return TaskMenu_GetSelection(&app->menu);
}

uint32_t AppController_GetElapsedMs(const AppController *app)
{
    return app->run_elapsed_ms;
}

bool AppController_IsSafeStopRequested(const AppController *app)
{
    return app->safe_stop_requested;
}
