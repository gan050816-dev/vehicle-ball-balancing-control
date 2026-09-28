#include "oled_ssd1306.h"

#include <stddef.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define OLED_ADDRESS_7BIT 0x3CU
#define OLED_WIDTH 128U
#define OLED_PAGES 4U
#define OLED_I2C_TIMEOUT 200000U

static uint8_t g_oled_buffer[OLED_WIDTH * OLED_PAGES];

static bool OLED_Write(const uint8_t *data, uint8_t length)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    {
        if (--timeout == 0U)
        {
            return false;
        }
    }

    DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, data, length);
    DL_I2C_startControllerTransfer(OLED_I2C_INST, OLED_ADDRESS_7BIT,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(96U);

    timeout = OLED_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U)
    {
        if (--timeout == 0U)
        {
            return false;
        }
    }

    return (DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
}

static bool OLED_Command(uint8_t command)
{
    uint8_t packet[2] = {0x00U, command};
    return OLED_Write(packet, 2U);
}

static void OLED_ClearBuffer(void)
{
    uint16_t index;

    for (index = 0U; index < sizeof(g_oled_buffer); ++index)
    {
        g_oled_buffer[index] = 0U;
    }
}

static const uint8_t *OLED_Font(char character)
{
    static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
    static const uint8_t digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E}
    };
    static const uint8_t a[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t c[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const uint8_t d[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t e[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t f[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
    static const uint8_t h[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
    static const uint8_t i[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const uint8_t k[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
    static const uint8_t l[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t m[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
    static const uint8_t n[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
    static const uint8_t o[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t p[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t r[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const uint8_t s[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
    static const uint8_t t[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t u[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
    static const uint8_t w[5] = {0x3F, 0x40, 0x38, 0x40, 0x3F};
    static const uint8_t y[5] = {0x07, 0x08, 0x70, 0x08, 0x07};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};

    if (character >= '0' && character <= '9')
    {
        return digits[(uint8_t)(character - '0')];
    }

    switch (character)
    {
        case 'A': return a;
        case 'C': return c;
        case 'D': return d;
        case 'E': return e;
        case 'F': return f;
        case 'H': return h;
        case 'I': return i;
        case 'K': return k;
        case 'L': return l;
        case 'M': return m;
        case 'N': return n;
        case 'O': return o;
        case 'P': return p;
        case 'R': return r;
        case 'S': return s;
        case 'T': return t;
        case 'U': return u;
        case 'W': return w;
        case 'Y': return y;
        case ':': return colon;
        default: return blank;
    }
}

static void OLED_PutCharacter(uint8_t x, uint8_t page, char character)
{
    const uint8_t *glyph = OLED_Font(character);
    uint8_t index;

    if (x > 122U || page >= OLED_PAGES)
    {
        return;
    }

    for (index = 0U; index < 5U; ++index)
    {
        g_oled_buffer[page * OLED_WIDTH + x + index] = glyph[index];
    }
}

static void OLED_PutString(uint8_t x, uint8_t page, const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0' && x <= 122U)
    {
        OLED_PutCharacter(x, page, *text++);
        x = (uint8_t)(x + 6U);
    }
}

static bool OLED_Update(void)
{
    uint8_t page;
    uint8_t column;
    uint8_t packet[6];

    packet[0] = 0x40U;
    for (page = 0U; page < OLED_PAGES; ++page)
    {
        if (!OLED_Command((uint8_t)(0xB0U + page)) ||
            !OLED_Command(0x00U) || !OLED_Command(0x10U))
        {
            return false;
        }

        for (column = 0U; column < OLED_WIDTH; column += 5U)
        {
            uint8_t remaining = (uint8_t)(OLED_WIDTH - column);
            uint8_t count = remaining > 5U ? 5U : remaining;
            uint8_t index;

            for (index = 0U; index < count; ++index)
            {
                packet[index + 1U] =
                    g_oled_buffer[page * OLED_WIDTH + column + index];
            }
            if (!OLED_Write(packet, (uint8_t)(count + 1U)))
            {
                return false;
            }
        }
    }

    return true;
}

bool OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x1F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x02, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };
    uint8_t index;

    delay_cycles(CPUCLK_FREQ / 12U);
    for (index = 0U; index < sizeof(init_commands); ++index)
    {
        if (!OLED_Command(init_commands[index]))
        {
            return false;
        }
    }

    OLED_ClearBuffer();
    return OLED_Update();
}

bool OLED_ShowLines(const char *line0, const char *line1,
                    const char *line2, const char *line3)
{
    OLED_ClearBuffer();
    OLED_PutString(0U, 0U, line0);
    OLED_PutString(0U, 1U, line1);
    OLED_PutString(0U, 2U, line2);
    OLED_PutString(0U, 3U, line3);
    return OLED_Update();
}
