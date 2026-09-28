#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include <stdint.h>

typedef enum
{
    BUTTON_EVENT_NONE = 0U,
    BUTTON_EVENT_SELECT = 1U << 0,
    BUTTON_EVENT_CONFIRM = 1U << 1,
    BUTTON_EVENT_RESET = 1U << 2,
    BUTTON_EVENT_CAPTURE = 1U << 3
} ButtonEvent;

/**
 * 读取上电时的按键状态。上电时已经按住的按键不会产生按下事件，
 * 必须先释放再重新按下。
 */
void ButtonDriver_Init(void);

/**
 * 按实际经过的毫秒数更新消抖器，返回本次新产生的按下事件位图。
 */
uint8_t ButtonDriver_Update(uint32_t elapsed_ms);

#endif
