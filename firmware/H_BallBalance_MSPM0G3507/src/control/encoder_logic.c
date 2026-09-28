#include "encoder_logic.h"

int8_t EncoderLogic_Transition(uint8_t previous,
                               uint8_t current,
                               uint8_t *valid)
{
    static const int8_t transition[16] = {
         0,  1, -1,  2,
        -1,  0,  2,  1,
         1,  2,  0, -1,
         2, -1,  1,  0
    };
    int8_t delta;

    if (previous > 3U || current > 3U || valid == 0)
    {
        return 0;
    }

    delta = transition[(previous << 2U) | current];
    if (delta == 2)
    {
        *valid = 0U;
        return 0;
    }

    *valid = 1U;
    return delta;
}

int16_t EncoderLogic_Delta16(uint16_t current, uint16_t previous)
{
    return (int16_t)(current - previous);
}
