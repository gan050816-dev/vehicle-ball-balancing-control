#ifndef TASK_MENU_H
#define TASK_MENU_H

#include <stdbool.h>
#include <stdint.h>

/**
 * 任务编号与赛题编号保持一一对应，禁止复用为调试页面编号。
 */
typedef enum
{
    TASK_ID_1_VIDEO = 1,
    TASK_ID_2_LAP_STOP,
    TASK_ID_3_BALL_MOVE,
    TASK_ID_4_TO_B_BALANCE,
    TASK_ID_5_LAP_CENTER,
    TASK_ID_6_LAP_TARGET
} TaskId;

typedef struct
{
    TaskId selected;
} TaskMenu;

void TaskMenu_Init(TaskMenu *menu);
void TaskMenu_SelectNext(TaskMenu *menu);
TaskId TaskMenu_GetSelection(const TaskMenu *menu);
bool TaskMenu_RequiresCapture(TaskId task);

#endif
