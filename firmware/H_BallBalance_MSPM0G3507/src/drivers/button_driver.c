#include "button_driver.h"

#include <stdbool.h>

#include "ti_msp_dl_config.h"

#define BUTTON_COUNT 4U
#define BUTTON_DEBOUNCE_MS 20U

typedef struct
{
    bool stable_pressed;
    bool candidate_pressed;
    uint16_t candidate_time_ms;
} ButtonDebounceState;

static ButtonDebounceState g_buttons[BUTTON_COUNT];

static bool ButtonDriver_ReadPressed(uint8_t index)
{
    switch (index)
    {
        case 0U:
            return (DL_GPIO_readPins(APP_GPIO_KEY_SELECT_PORT,
                                     APP_GPIO_KEY_SELECT_PIN) &
                    APP_GPIO_KEY_SELECT_PIN) == 0U;
        case 1U:
            return (DL_GPIO_readPins(APP_GPIO_KEY_CONFIRM_PORT,
                                     APP_GPIO_KEY_CONFIRM_PIN) &
                    APP_GPIO_KEY_CONFIRM_PIN) == 0U;
        case 2U:
            return (DL_GPIO_readPins(APP_GPIO_KEY_RESET_PORT,
                                     APP_GPIO_KEY_RESET_PIN) &
                    APP_GPIO_KEY_RESET_PIN) == 0U;
        case 3U:
            return (DL_GPIO_readPins(APP_GPIO_KEY_CAPTURE_PORT,
                                     APP_GPIO_KEY_CAPTURE_PIN) &
                    APP_GPIO_KEY_CAPTURE_PIN) == 0U;
        default:
            return false;
    }
}

static uint8_t ButtonDriver_EventMask(uint8_t index)
{
    static const uint8_t masks[BUTTON_COUNT] = {
        BUTTON_EVENT_SELECT,
        BUTTON_EVENT_CONFIRM,
        BUTTON_EVENT_RESET,
        BUTTON_EVENT_CAPTURE
    };
    return masks[index];
}

void ButtonDriver_Init(void)
{
    uint8_t index;

    for (index = 0U; index < BUTTON_COUNT; ++index)
    {
        bool pressed = ButtonDriver_ReadPressed(index);
        g_buttons[index].stable_pressed = pressed;
        g_buttons[index].candidate_pressed = pressed;
        g_buttons[index].candidate_time_ms = 0U;
    }
}

uint8_t ButtonDriver_Update(uint32_t elapsed_ms)
{
    uint8_t events = BUTTON_EVENT_NONE;
    uint8_t index;

    if (elapsed_ms == 0U)
    {
        return events;
    }

    for (index = 0U; index < BUTTON_COUNT; ++index)
    {
        ButtonDebounceState *state = &g_buttons[index];
        bool pressed = ButtonDriver_ReadPressed(index);

        if (pressed != state->candidate_pressed)
        {
            state->candidate_pressed = pressed;
            state->candidate_time_ms = 0U;
            continue;
        }

        if (state->candidate_time_ms < BUTTON_DEBOUNCE_MS)
        {
            uint32_t remaining =
                BUTTON_DEBOUNCE_MS - state->candidate_time_ms;
            state->candidate_time_ms =
                elapsed_ms >= remaining
                    ? BUTTON_DEBOUNCE_MS
                    : (uint16_t)(state->candidate_time_ms + elapsed_ms);
        }

        if (state->candidate_time_ms >= BUTTON_DEBOUNCE_MS &&
            state->stable_pressed != state->candidate_pressed)
        {
            state->stable_pressed = state->candidate_pressed;
            if (state->stable_pressed)
            {
                events |= ButtonDriver_EventMask(index);
            }
        }
    }

    return events;
}
