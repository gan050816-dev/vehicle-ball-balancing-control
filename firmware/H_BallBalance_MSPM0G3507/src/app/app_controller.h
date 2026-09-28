#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "task_menu.h"

typedef enum
{
    APP_STATE_HOME_REQUIRED = 0,
    APP_STATE_MOVING_TO_LEVEL,
    APP_STATE_MENU,
    APP_STATE_PLACE_BALL,
    APP_STATE_CAPTURE_REQUIRED,
    APP_STATE_CAPTURE_READY,
    APP_STATE_READY,
    APP_STATE_RUNNING,
    APP_STATE_DONE,
    APP_STATE_FAULT
} AppState;

typedef enum
{
    APP_KEY_SELECT = 0,
    APP_KEY_CONFIRM,
    APP_KEY_RESET,
    APP_KEY_CAPTURE
} AppKey;

/**
 * 应用层只发出动作请求，硬件服务负责执行并反馈结果。
 */
typedef enum
{
    APP_ACTION_NONE = 0,
    APP_ACTION_CONFIRM_HOME,
    APP_ACTION_PREPARE_TASK,
    APP_ACTION_REQUEST_CAPTURE,
    APP_ACTION_START_TASK,
    APP_ACTION_STOP_AND_RESET
} AppAction;

typedef struct
{
    TaskMenu menu;
    AppState state;
    bool capture_valid;
    bool safe_stop_requested;
    uint32_t run_elapsed_ms;
} AppController;

void AppController_Init(AppController *app);
AppAction AppController_OnKey(AppController *app, AppKey key);
bool AppController_LevelReached(AppController *app);
bool AppController_VisionReady(AppController *app);
bool AppController_CaptureSucceeded(AppController *app);
void AppController_PreparationLost(AppController *app);
void AppController_TaskFinished(AppController *app);
void AppController_TaskFailed(AppController *app);
void AppController_AdvanceTime(AppController *app, uint32_t elapsed_ms);

AppState AppController_GetState(const AppController *app);
TaskId AppController_GetTask(const AppController *app);
uint32_t AppController_GetElapsedMs(const AppController *app);
bool AppController_IsSafeStopRequested(const AppController *app);

#endif
