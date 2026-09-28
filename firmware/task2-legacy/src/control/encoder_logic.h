#ifndef ENCODER_LOGIC_H
#define ENCODER_LOGIC_H

#include <stdint.h>

int8_t EncoderLogic_Transition(uint8_t previous,
                               uint8_t current,
                               uint8_t *valid);
int16_t EncoderLogic_Delta16(uint16_t current, uint16_t previous);

#endif
