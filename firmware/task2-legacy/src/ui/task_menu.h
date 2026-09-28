#ifndef TASK_MENU_H
#define TASK_MENU_H

#include <stdint.h>

typedef enum
{
    TASK_MODE_T2_LAP18 = 0,
    TASK_MODE_T4_AB_LAP,
    TASK_MODE_COUNT
} TaskMode;

typedef struct
{
    TaskMode mode;
    uint8_t confirmed;
    uint8_t running;
} TaskMenu;

void TaskMenu_Init(TaskMenu *menu);
void TaskMenu_SelectNext(TaskMenu *menu);
void TaskMenu_Confirm(TaskMenu *menu);
uint8_t TaskMenu_Start(TaskMenu *menu);
void TaskMenu_Stop(TaskMenu *menu);
TaskMode TaskMenu_GetMode(const TaskMenu *menu);
uint8_t TaskMenu_IsConfirmed(const TaskMenu *menu);
uint8_t TaskMenu_IsRunning(const TaskMenu *menu);
uint8_t TaskMenu_UsesBalanceControl(const TaskMenu *menu);

#endif
