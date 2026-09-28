#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdint.h>

#include "run_status.h"

uint8_t OLED_Init(void);
void OLED_ShowLineStatus(const LineDisplayStatus *status);

#endif
