#include <assert.h>

#include "task_menu.h"

int main(void)
{
    TaskMenu menu;
    uint8_t task;

    TaskMenu_Init(&menu);
    assert(TaskMenu_GetSelection(&menu) == TASK_ID_2_LAP_STOP);

    for (task = 3U; task <= 6U; ++task)
    {
        TaskMenu_SelectNext(&menu);
        assert((uint8_t)TaskMenu_GetSelection(&menu) == task);
    }

    TaskMenu_SelectNext(&menu);
    assert(TaskMenu_GetSelection(&menu) == TASK_ID_2_LAP_STOP);

    assert(!TaskMenu_RequiresCapture(TASK_ID_1_VIDEO));
    assert(!TaskMenu_RequiresCapture(TASK_ID_2_LAP_STOP));
    assert(!TaskMenu_RequiresCapture(TASK_ID_3_BALL_MOVE));
    assert(!TaskMenu_RequiresCapture(TASK_ID_4_TO_B_BALANCE));
    assert(!TaskMenu_RequiresCapture(TASK_ID_5_LAP_CENTER));
    assert(TaskMenu_RequiresCapture(TASK_ID_6_LAP_TARGET));
    return 0;
}
