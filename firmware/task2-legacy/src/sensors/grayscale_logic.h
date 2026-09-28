#ifndef GRAYSCALE_LOGIC_H
#define GRAYSCALE_LOGIC_H

#include <stdint.h>

uint8_t Grayscale_ActiveMask(uint8_t raw_mask, uint8_t active_high);
int16_t Grayscale_LineError(uint8_t active_mask);
int16_t Grayscale_LostLineCommand(int16_t last_nonzero_error,
                                  int16_t magnitude);
uint8_t Grayscale_ActiveCount(uint8_t active_mask);

#endif
