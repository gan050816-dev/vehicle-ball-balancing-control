#include "task_menu.h"

void TaskMenu_Init(TaskMenu *menu)
{
    menu->mode = TASK_MODE_T2_LAP18;
    menu->confirmed = 0U;
    menu->running = 0U;
}

void TaskMenu_SelectNext(TaskMenu *menu)
{
    if (menu->confirmed == 0U && menu->running == 0U)
    {
        menu->mode = (TaskMode)(((uint8_t)menu->mode + 1U) %
                                (uint8_t)TASK_MODE_COUNT);
    }
}

void TaskMenu_Confirm(TaskMenu *menu)
{
    if (menu->running == 0U)
    {
        menu->confirmed = 1U;
    }
}

uint8_t TaskMenu_Start(TaskMenu *menu)
{
    if (menu->confirmed == 0U || menu->running != 0U)
    {
        return 0U;
    }
    menu->running = 1U;
    return 1U;
}

void TaskMenu_Stop(TaskMenu *menu)
{
    menu->running = 0U;
    menu->confirmed = 0U;
}

TaskMode TaskMenu_GetMode(const TaskMenu *menu)
{
    return menu->mode;
}

uint8_t TaskMenu_IsConfirmed(const TaskMenu *menu)
{
    return menu->confirmed;
}

uint8_t TaskMenu_IsRunning(const TaskMenu *menu)
{
    return menu->running;
}

uint8_t TaskMenu_UsesBalanceControl(const TaskMenu *menu)
{
    return menu->mode == TASK_MODE_T4_AB_LAP ? 1U : 0U;
}
