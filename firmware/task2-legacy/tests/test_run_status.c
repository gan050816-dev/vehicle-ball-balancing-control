#include <assert.h>
#include <string.h>

#include "run_status.h"

int main(void)
{
    LineDisplayStatus status = {
        TASK_MODE_T2_LAP18,
        RUN_DISPLAY_SELECT,
        RUN_STOP_NONE,
        800U,
        0U,
        0,
        0U,
        0U,
        0U,
        0,
        149U,
        0U,
        0U,
        0U,
        0U,
        0U,
        0U
    };
    char lines[4][RUN_STATUS_LINE_LENGTH];

    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T2 SELECT") == 0);
    assert(strcmp(lines[1], "A:0800 mm/s2") == 0);
    assert(strcmp(lines[2], "V:0000 mm/s") == 0);
    assert(strcmp(lines[3], "RUN:0000.00s") == 0);

    status.mode = TASK_MODE_T4_AB_LAP;
    status.phase = RUN_DISPLAY_RUNNING;
    status.maximum_acceleration_mm_s2 = 300U;
    status.peak_acceleration_mm_s2 = 300U;
    status.peak_ball_x_tenths_mm = -400;
    status.current_speed_mm_s = 232U;
    status.elapsed_centiseconds = 2794U;
    status.ball_valid = 1U;
    status.ball_x_tenths_mm = -37;
    status.motor_angle_cdeg = -2015;
    status.vision_diagnostic_code = 12U;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 RUN") == 0);
    assert(strcmp(lines[1], "AP:0300 X:-040.0") == 0);
    assert(strcmp(lines[2], "X:-003.7 A:-20.15") == 0);
    assert(strcmp(lines[3], "RUN:0027.94s") == 0);

    status.elapsed_centiseconds = 3100U;
    status.phase = RUN_DISPLAY_STOPPED;
    status.stop_reason = RUN_STOP_TIME;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 STOP TIME") == 0);
    assert(strcmp(lines[3], "RUN:0031.00s") == 0);

    status.stop_reason = RUN_STOP_LINE;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 STOP LINE") == 0);
    status.stop_reason = RUN_STOP_STALL;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 STOP STALL") == 0);

    status.phase = RUN_DISPLAY_FAULT;
    status.task4_fault_code = 1U;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 FAULT START") == 0);
    status.task4_fault_code = 2U;
    status.task4_vision_fault_diagnostic_code = 32U;
    status.task4_vision_fault_age_ms = 147U;
    status.task4_vision_fault_checksum_errors = 2U;
    status.task4_vision_fault_rx_overflows = 1U;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 FAULT VISION") == 0);
    assert(strcmp(lines[1], "D:032 AGE:0147ms") == 0);
    assert(strcmp(lines[2], "CRC:0002 OVF:0001") == 0);
    status.task4_fault_code = 3U;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 FAULT X42") == 0);
    assert(strcmp(lines[1], "AP:0300 X:-040.0") == 0);
    assert(strcmp(lines[2], "X:-003.7 A:-20.15") == 0);
    status.task4_fault_code = 4U;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 FAULT CMD") == 0);

    status.task4_fault_code = 2U;
    status.task4_vision_fault_age_ms = 12000U;
    status.task4_vision_fault_checksum_errors = 20000U;
    status.task4_vision_fault_rx_overflows = 30000U;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[1], "D:032 AGE:9999ms") == 0);
    assert(strcmp(lines[2], "CRC:9999 OVF:9999") == 0);

    status.phase = RUN_DISPLAY_WAIT;
    status.stop_reason = RUN_STOP_NONE;
    status.elapsed_centiseconds = 0U;
    status.ball_valid = 0U;
    LineDisplay_Format(&status, lines);
    assert(strcmp(lines[0], "T4 WAIT") == 0);
    assert(strcmp(lines[2], "X:----.- A:-20.15") == 0);
    assert(strcmp(lines[3], "RUN:0000.00s") == 0);
    return 0;
}
