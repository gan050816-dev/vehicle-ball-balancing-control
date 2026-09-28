#include "safety_manager.h"

void SafetyManager_Init(SafetyManager *manager)
{
    manager->home_confirmed = false;
    manager->motion_allowed = false;
    manager->stop_reason = SAFETY_STOP_BOOT;
}

void SafetyManager_ConfirmHome(SafetyManager *manager)
{
    manager->home_confirmed = true;
}

bool SafetyManager_TryEnableMotion(SafetyManager *manager)
{
    if (!manager->home_confirmed)
    {
        manager->motion_allowed = false;
        return false;
    }

    manager->motion_allowed = true;
    manager->stop_reason = SAFETY_STOP_NONE;
    return true;
}

void SafetyManager_RequestStop(
    SafetyManager *manager, SafetyStopReason reason, bool clear_home)
{
    manager->motion_allowed = false;
    manager->stop_reason = reason;
    if (clear_home)
    {
        manager->home_confirmed = false;
    }
}

bool SafetyManager_IsMotionAllowed(const SafetyManager *manager)
{
    return manager->motion_allowed;
}
