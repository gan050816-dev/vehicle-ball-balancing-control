#include "run_status.h"

static void AppendChar(char *line, uint8_t *position, char value)
{
    if (*position < RUN_STATUS_LINE_LENGTH - 1U)
    {
        line[*position] = value;
        ++(*position);
        line[*position] = '\0';
    }
}

static void AppendText(char *line, uint8_t *position, const char *text)
{
    while (*text != '\0')
    {
        AppendChar(line, position, *text++);
    }
}

static void AppendUnsigned(char *line, uint8_t *position,
                           uint32_t value, uint8_t width)
{
    uint32_t divisor = 1U;
    uint8_t index;

    for (index = 1U; index < width; ++index)
    {
        divisor *= 10U;
    }
    for (index = 0U; index < width; ++index)
    {
        AppendChar(
            line, position, (char)('0' + (value / divisor) % 10U));
        if (divisor > 1U)
        {
            divisor /= 10U;
        }
    }
}

static void AppendSignedTenths(char *line, uint8_t *position,
                               int16_t value)
{
    uint16_t magnitude;

    if (value < 0)
    {
        AppendChar(line, position, '-');
        magnitude = (uint16_t)(-(int32_t)value);
    }
    else
    {
        AppendChar(line, position, '+');
        magnitude = (uint16_t)value;
    }
    if (magnitude > 9999U)
    {
        magnitude = 9999U;
    }
    AppendUnsigned(line, position, magnitude / 10U, 3U);
    AppendChar(line, position, '.');
    AppendUnsigned(line, position, magnitude % 10U, 1U);
}

static void AppendSignedHundredths(char *line, uint8_t *position,
                                   int16_t value)
{
    uint16_t magnitude;

    if (value < 0)
    {
        AppendChar(line, position, '-');
        magnitude = (uint16_t)(-(int32_t)value);
    }
    else
    {
        AppendChar(line, position, '+');
        magnitude = (uint16_t)value;
    }
    if (magnitude > 9999U)
    {
        magnitude = 9999U;
    }
    AppendUnsigned(line, position, magnitude / 100U, 2U);
    AppendChar(line, position, '.');
    AppendUnsigned(line, position, magnitude % 100U, 2U);
}

static void AppendMode(char *line, uint8_t *position, TaskMode mode)
{
    if (mode == TASK_MODE_T2_LAP18)
    {
        AppendText(line, position, "T2");
    }
    else
    {
        AppendText(line, position, "T4");
    }
}

static const char *PhaseText(RunDisplayPhase phase)
{
    switch (phase)
    {
        case RUN_DISPLAY_SELECT:
            return "SELECT";
        case RUN_DISPLAY_RUNNING:
            return "RUN";
        case RUN_DISPLAY_STOPPED:
            return "STOP";
        case RUN_DISPLAY_HOME:
            return "HOME";
        case RUN_DISPLAY_LEVEL:
            return "LEVEL";
        case RUN_DISPLAY_WAIT:
            return "WAIT";
        default:
            return "FAULT";
    }
}

static const char *StopReasonText(RunStopReason reason)
{
    switch (reason)
    {
        case RUN_STOP_TIME:
            return " TIME";
        case RUN_STOP_LINE:
            return " LINE";
        case RUN_STOP_STALL:
            return " STALL";
        case RUN_STOP_FINISH:
            return " FIN";
        default:
            return "";
    }
}

static const char *Task4FaultText(uint8_t fault_code)
{
    switch (fault_code)
    {
        case 1U:
            return " START";
        case 2U:
            return " VISION";
        case 3U:
            return " X42";
        case 4U:
            return " CMD";
        default:
            return "";
    }
}

static uint32_t ClampFourDigits(uint32_t value)
{
    return value > 9999U ? 9999U : value;
}

void LineDisplay_Format(const LineDisplayStatus *status,
                        char lines[4][RUN_STATUS_LINE_LENGTH])
{
    uint8_t position;
    uint32_t centiseconds = status->elapsed_centiseconds;
    uint32_t seconds;

    if (centiseconds > 999999U)
    {
        centiseconds = 999999U;
    }
    seconds = centiseconds / 100U;

    position = 0U;
    lines[0][0] = '\0';
    AppendMode(lines[0], &position, status->mode);
    AppendChar(lines[0], &position, ' ');
    AppendText(lines[0], &position, PhaseText(status->phase));
    if (status->phase == RUN_DISPLAY_STOPPED)
    {
        AppendText(lines[0], &position,
                   StopReasonText(status->stop_reason));
    }
    else if (status->phase == RUN_DISPLAY_FAULT &&
             status->mode == TASK_MODE_T4_AB_LAP)
    {
        AppendText(lines[0], &position,
                   Task4FaultText(status->task4_fault_code));
    }

    position = 0U;
    lines[1][0] = '\0';
    if (status->mode == TASK_MODE_T4_AB_LAP &&
        status->phase == RUN_DISPLAY_FAULT &&
        status->task4_fault_code == 2U)
    {
        AppendText(lines[1], &position, "D:");
        AppendUnsigned(lines[1], &position,
                       status->task4_vision_fault_diagnostic_code,
                       3U);
        AppendText(lines[1], &position, " AGE:");
        AppendUnsigned(lines[1], &position,
                       ClampFourDigits(
                           status->task4_vision_fault_age_ms),
                       4U);
        AppendText(lines[1], &position, "ms");
    }
    else if (status->mode == TASK_MODE_T4_AB_LAP)
    {
        AppendText(lines[1], &position, "AP:");
        AppendUnsigned(lines[1], &position,
                       status->peak_acceleration_mm_s2, 4U);
        AppendText(lines[1], &position, " X:");
        AppendSignedTenths(lines[1], &position,
                           status->peak_ball_x_tenths_mm);
    }
    else
    {
        AppendText(lines[1], &position, "A:");
        AppendUnsigned(lines[1], &position,
                       status->maximum_acceleration_mm_s2, 4U);
        AppendText(lines[1], &position, " mm/s2");
    }

    position = 0U;
    lines[2][0] = '\0';
    if (status->mode == TASK_MODE_T4_AB_LAP &&
        status->phase == RUN_DISPLAY_FAULT &&
        status->task4_fault_code == 2U)
    {
        AppendText(lines[2], &position, "CRC:");
        AppendUnsigned(lines[2], &position,
                       ClampFourDigits(
                           status->task4_vision_fault_checksum_errors),
                       4U);
        AppendText(lines[2], &position, " OVF:");
        AppendUnsigned(lines[2], &position,
                       ClampFourDigits(
                           status->task4_vision_fault_rx_overflows),
                       4U);
    }
    else if (status->mode == TASK_MODE_T4_AB_LAP)
    {
        if (RUN_STATUS_T4_DIAGNOSTIC != 0U)
        {
            AppendText(lines[2], &position, "D:");
            AppendUnsigned(lines[2], &position,
                           status->vision_diagnostic_code, 3U);
        }
        else
        {
            AppendText(lines[2], &position, "X:");
            if (status->ball_valid != 0U)
            {
                AppendSignedTenths(
                    lines[2], &position,
                    status->ball_x_tenths_mm);
            }
            else
            {
                AppendText(lines[2], &position, "----.-");
            }
            AppendText(lines[2], &position, " A:");
            AppendSignedHundredths(
                lines[2], &position,
                status->motor_angle_cdeg);
        }
    }
    else
    {
        AppendText(lines[2], &position, "V:");
        AppendUnsigned(lines[2], &position,
                       status->current_speed_mm_s, 4U);
        AppendText(lines[2], &position, " mm/s");
    }

    position = 0U;
    lines[3][0] = '\0';
    AppendText(lines[3], &position, "RUN:");
    AppendUnsigned(lines[3], &position, seconds, 4U);
    AppendChar(lines[3], &position, '.');
    AppendUnsigned(lines[3], &position, centiseconds % 100U, 2U);
    AppendChar(lines[3], &position, 's');
}
