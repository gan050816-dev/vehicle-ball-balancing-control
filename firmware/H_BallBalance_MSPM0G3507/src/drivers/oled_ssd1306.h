#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdbool.h>

bool OLED_Init(void);
bool OLED_ShowLines(const char *line0, const char *line1,
                    const char *line2, const char *line3);

#endif
