#include "grayscale_logic.h"

uint8_t Grayscale_ActiveMask(uint8_t raw_mask, uint8_t active_high)
{
    return active_high != 0U ? raw_mask : (uint8_t)~raw_mask;
}

int16_t Grayscale_LineError(uint8_t active_mask)
{
    static const int16_t weights[8] = {
        -144, -120, -70, -20, 20, 70, 120, 144
    };
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < 8U; ++index)
    {
        if ((active_mask & (uint8_t)(1U << index)) != 0U)
        {
            sum = (int16_t)(sum + weights[index]);
            ++count;
        }
    }
    return count == 0U ? 0 : (int16_t)(sum / count);
}

int16_t Grayscale_LostLineCommand(int16_t last_nonzero_error,
                                  int16_t magnitude)
{
    if (last_nonzero_error < 0)
    {
        return (int16_t)-magnitude;
    }
    if (last_nonzero_error > 0)
    {
        return magnitude;
    }
    return 0;
}

uint8_t Grayscale_ActiveCount(uint8_t active_mask)
{
    uint8_t count = 0U;

    while (active_mask != 0U)
    {
        count = (uint8_t)(count + (active_mask & 1U));
        active_mask >>= 1U;
    }
    return count;
}
