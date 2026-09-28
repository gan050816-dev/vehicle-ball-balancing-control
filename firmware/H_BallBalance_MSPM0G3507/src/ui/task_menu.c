#include "task_menu.h"

void TaskMenu_Init(TaskMenu *menu)
{
    menu->selected = TASK_ID_2_LAP_STOP;
}

void TaskMenu_SelectNext(TaskMenu *menu)
{
    if (menu->selected >= TASK_ID_6_LAP_TARGET)
    {
        menu->selected = TASK_ID_2_LAP_STOP;
    }
    else
    {
        menu->selected = (TaskId)((uint8_t)menu->selected + 1U);
    }
}

TaskId TaskMenu_GetSelection(const TaskMenu *menu)
{
    return menu->selected;
}

bool TaskMenu_RequiresCapture(TaskId task)
{
    return task == TASK_ID_6_LAP_TARGET;
}
