#include <assert.h>

#include "grayscale_logic.h"

int main(void)
{
    assert(Grayscale_ActiveMask(0x01U, 1U) == 0x01U);
    assert(Grayscale_ActiveMask(0xFEU, 0U) == 0x01U);
    assert(Grayscale_LineError(0x01U) == -144);
    assert(Grayscale_LineError(0x02U) == -120);
    assert(Grayscale_LineError(0x04U) == -70);
    assert(Grayscale_LineError(0x08U) == -20);
    assert(Grayscale_LineError(0x10U) == 20);
    assert(Grayscale_LineError(0x20U) == 70);
    assert(Grayscale_LineError(0x40U) == 120);
    assert(Grayscale_LineError(0x80U) == 144);
    assert(Grayscale_LineError(0x03U) == -132);
    assert(Grayscale_LineError(0xC0U) == 132);
    assert(Grayscale_LineError(0x18U) == 0);
    assert(Grayscale_LineError(0x00U) == 0);
    assert(Grayscale_LostLineCommand(-20, 120) == -120);
    assert(Grayscale_LostLineCommand(20, 120) == 120);
    assert(Grayscale_LostLineCommand(0, 120) == 0);
    assert(Grayscale_ActiveCount(0x00U) == 0U);
    assert(Grayscale_ActiveCount(0x18U) == 2U);
    assert(Grayscale_ActiveCount(0xFFU) == 8U);
    return 0;
}
