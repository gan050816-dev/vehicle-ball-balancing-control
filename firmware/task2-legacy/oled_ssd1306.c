#include "oled_ssd1306.h"

#include "ti_msp_dl_config.h"

#define OLED_ADDRESS_7BIT 0x3CU
#define OLED_WIDTH 128U
#define OLED_PAGES 4U
#define OLED_I2C_TIMEOUT 200000U

static uint8_t g_buffer[OLED_WIDTH * OLED_PAGES];

static uint8_t OLED_Write(const uint8_t *data, uint8_t length)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    {
        if (--timeout == 0U)
        {
            return 0U;
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
            return 0U;
        }
    }
    return ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_ERROR) == 0U)
               ? 1U
               : 0U;
}

static uint8_t OLED_Command(uint8_t command)
{
    uint8_t packet[2] = {0x00U, command};
    return OLED_Write(packet, 2U);
}

static void OLED_Clear(void)
{
    uint16_t i;
    for (i = 0U; i < sizeof(g_buffer); ++i)
    {
        g_buffer[i] = 0U;
    }
}

static const uint8_t *OLED_Font(char ch)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t glyphs[][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
        {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
        {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
        {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
        {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
        {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
        {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
        {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
        {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
        {0x06, 0x49, 0x49, 0x29, 0x1E}  /* 9 */
    };
    static const uint8_t A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const uint8_t D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
    static const uint8_t I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const uint8_t K[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
    static const uint8_t L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
    static const uint8_t O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const uint8_t S[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
    static const uint8_t T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
    static const uint8_t W[5] = {0x3F, 0x40, 0x38, 0x40, 0x3F};
    static const uint8_t X[5] = {0x63, 0x14, 0x08, 0x14, 0x63};
    static const uint8_t Y[5] = {0x07, 0x08, 0x70, 0x08, 0x07};
    static const uint8_t arrow[5] = {0x00, 0x41, 0x22, 0x14, 0x08};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t plus[5] = {0x08, 0x08, 0x3E, 0x08, 0x08};
    static const uint8_t minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t point[5] = {0x00, 0x60, 0x60, 0x00, 0x00};

    if (ch >= '0' && ch <= '9') return glyphs[(uint8_t)(ch - '0')];
    switch (ch)
    {
        case 'A': return A; case 'B': return B; case 'C': return C;
        case 'D': return D; case 'E': return E; case 'G': return G;
        case 'I': return I; case 'K': return K; case 'L': return L;
        case 'N': return N; case 'O': return O; case 'P': return P;
        case 'R': return R; case 'S': return S;
        case 'T': return T; case 'U': return U; case 'W': return W;
        case 'X': return X; case 'Y': return Y; case '>': return arrow;
        case ':': return colon; case '+': return plus; case '-': return minus;
        case '.': return point; default: return blank;
    }
}

static void OLED_Putc(uint8_t x, uint8_t page, char ch)
{
    const uint8_t *glyph = OLED_Font(ch);
    uint8_t i;
    if (x > 122U || page >= OLED_PAGES) return;
    for (i = 0U; i < 5U; ++i) g_buffer[page * OLED_WIDTH + x + i] = glyph[i];
}

static void OLED_Puts(uint8_t x, uint8_t page, const char *text)
{
    while (*text != '\0' && x <= 122U)
    {
        OLED_Putc(x, page, *text++);
        x = (uint8_t)(x + 6U);
    }
}

static uint8_t OLED_Update(void)
{
    uint8_t page;
    uint8_t col;
    uint8_t packet[6];
    packet[0] = 0x40U;

    for (page = 0U; page < OLED_PAGES; ++page)
    {
        if (!OLED_Command((uint8_t)(0xB0U + page)) ||
            !OLED_Command(0x00U) || !OLED_Command(0x10U)) return 0U;
        for (col = 0U; col < OLED_WIDTH; col += 5U)
        {
            uint8_t count = (uint8_t)((OLED_WIDTH - col) > 5U ? 5U : (OLED_WIDTH - col));
            uint8_t i;
            for (i = 0U; i < count; ++i) packet[i + 1U] = g_buffer[page * OLED_WIDTH + col + i];
            if (!OLED_Write(packet, (uint8_t)(count + 1U))) return 0U;
        }
    }
    return 1U;
}

uint8_t OLED_Init(void)
{
    static const uint8_t init[] = {0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,0x40,
        0x81,0x7F,0xA1,0xA6,0xA8,0x1F,0xA4,0xD3,0x00,0xD5,0x80,0xD9,
        0xF1,0xDA,0x02,0xDB,0x40,0x8D,0x14,0xAF};
    uint8_t i;
    delay_cycles(CPUCLK_FREQ / 12U);
    for (i = 0U; i < sizeof(init); ++i) if (!OLED_Command(init[i])) return 0U;
    OLED_Clear();
    return OLED_Update();
}

void OLED_ShowLineStatus(const LineDisplayStatus *status)
{
    char lines[4][RUN_STATUS_LINE_LENGTH];
    uint8_t page;

    LineDisplay_Format(status, lines);
    OLED_Clear();
    for (page = 0U; page < OLED_PAGES; ++page)
    {
        OLED_Puts(0U, page, lines[page]);
    }
    (void)OLED_Update();
}
