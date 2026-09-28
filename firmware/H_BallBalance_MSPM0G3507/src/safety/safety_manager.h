#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdbool.h>

typedef enum
{
    SAFETY_STOP_BOOT = 0,
    SAFETY_STOP_RESET,
    SAFETY_STOP_TASK_DONE,
    SAFETY_STOP_FAULT,
    SAFETY_STOP_NONE
} SafetyStopReason;

/**
 * 当前模块只维护安全逻辑状态。底盘和步进电机接入后，
 * 具体制动和停止输出必须由对应驱动执行。
 */
typedef struct
{
    bool home_confirmed;
    bool motion_allowed;
    SafetyStopReason stop_reason;
} SafetyManager;

void SafetyManager_Init(SafetyManager *manager);
void SafetyManager_ConfirmHome(SafetyManager *manager);
bool SafetyManager_TryEnableMotion(SafetyManager *manager);
void SafetyManager_RequestStop(
    SafetyManager *manager, SafetyStopReason reason, bool clear_home);
bool SafetyManager_IsMotionAllowed(const SafetyManager *manager);

#endif
