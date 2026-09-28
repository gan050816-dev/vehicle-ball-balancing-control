#include <assert.h>

#include "task_menu.h"

int main(void)
{
    TaskMenu menu;

    TaskMenu_Init(&menu);
    assert(TaskMenu_GetMode(&menu) == TASK_MODE_T2_LAP18);
    assert(TaskMenu_IsConfirmed(&menu) == 0U);
    assert(TaskMenu_IsRunning(&menu) == 0U);
    assert(TaskMenu_UsesBalanceControl(&menu) == 0U);
    assert(TaskMenu_Start(&menu) == 0U);

    TaskMenu_SelectNext(&menu);
    assert(TaskMenu_GetMode(&menu) == TASK_MODE_T4_AB_LAP);
    assert(TaskMenu_UsesBalanceControl(&menu) != 0U);
    TaskMenu_Confirm(&menu);
    assert(TaskMenu_IsConfirmed(&menu) != 0U);
    TaskMenu_SelectNext(&menu);
    assert(TaskMenu_GetMode(&menu) == TASK_MODE_T4_AB_LAP);
    assert(TaskMenu_Start(&menu) != 0U);
    TaskMenu_SelectNext(&menu);
    assert(TaskMenu_GetMode(&menu) == TASK_MODE_T4_AB_LAP);
    assert(TaskMenu_IsRunning(&menu) == 1U);

    TaskMenu_Stop(&menu);
    assert(TaskMenu_IsConfirmed(&menu) == 0U);
    TaskMenu_SelectNext(&menu);
    assert(TaskMenu_GetMode(&menu) == TASK_MODE_T2_LAP18);
    assert(TaskMenu_IsRunning(&menu) == 0U);
    return 0;
}
