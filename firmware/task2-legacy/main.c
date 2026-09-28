#include "ti_msp_dl_config.h"

#include <stdbool.h>

#include "drive_logic.h"
#include "encoder_logic.h"
#include "grayscale_logic.h"
#include "k230_ball_protocol.h"
#include "motor_safety.h"
#include "oled_ssd1306.h"
#include "run_status.h"
#include "speed_pid.h"
#include "stepper_service.h"
#include "task_menu.h"
#include "task4_run.h"
#include "vision_service.h"
#include "x42_protocol.h"

#define CONTROL_TICK_MS 10U
#define PWM_PERIOD_TICKS 1600U
#define MAX_WHEEL_SPEED_MM_S 1000
#define T2_FORWARD_JERK_MM_S3 2000U
#define T4_FORWARD_JERK_MM_S3 800U
#define T2_FORWARD_ACCEL_MM_S2 800U
#define T4_FORWARD_ACCEL_MM_S2 300U
#define T2_SPEED_MM_S 350
#define T4_AB_SPEED_MM_S 208
#define T4_AFTER_B_SPEED_MM_S 244
#define T4_SWITCH_TICKS 800U
#define T2_FINISH_ARM_TICKS 900U
#define T2_TIMEOUT_TICKS 2000U
#define T4_STOP_TICKS 3100U
#define FINISH_ACTIVE_SENSOR_COUNT 4U
#define FINISH_CONFIRM_TICKS 3U
#define LOST_LINE_SPEED_MM_S 120
#define LINE_CENTER_DEADBAND 20
#define T2_HIGH_GAIN_ERROR 60
#define T2_LOW_KP_NUMERATOR 2
#define T2_HIGH_KP_NUMERATOR 4
#define T2_KD_NUMERATOR 1
#define T2_PD_DENOMINATOR 2
#define T2_LOW_DERIVATIVE_LIMIT 20
#define T2_HIGH_DERIVATIVE_LIMIT 40
#define T4_LINE_KP_NUMERATOR 3
#define T4_LINE_KD_NUMERATOR 1
#define T4_PD_DENOMINATOR 4
#define T4_DERIVATIVE_LIMIT 20
#define T2_LOST_LINE_DIFFERENTIAL_MM_S 270
#define T4_LOST_LINE_DIFFERENTIAL_MM_S 120
#define LOST_LINE_STOP_TICKS 100U
#define STEERING_REVERSAL_CONFIRM_TICKS 2U
#define PWM_RAMP_STEP_TICKS 80U
#define DIRECTION_BRAKE_TICKS 3U
#define LEFT_MOTOR_INVERT 0U
#define RIGHT_MOTOR_INVERT 0U
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN 1
#define STALL_MIN_TARGET_MM_S 100
#define KEY_DEBOUNCE_TICKS 3U
#define OLED_REFRESH_TICKS 20U
#define X42_RX_BUFFER_SIZE 64U
#define K230_RX_BUFFER_SIZE 128U
#define UART_TX_SPIN_LIMIT 100000U
#define K230_TX_TIMEOUT_MS 100U

typedef enum
{
    EQUIPMENT_HOME = 0,
    EQUIPMENT_LEVELING,
    EQUIPMENT_READY,
    EQUIPMENT_FAULT
} EquipmentState;

typedef struct
{
    uint8_t frame[K230_BALL_DEFAULT_ORIGIN_FRAME_SIZE];
    uint8_t length;
    uint8_t position;
    uint32_t deadline_ms;
    uint32_t timeout_count;
    uint32_t overflow_count;
} K230Link;

typedef struct
{
    uint8_t stable_pressed;
    uint8_t debounce_ticks;
} KeyState;

static volatile uint8_t g_control_ticks_pending;
static volatile uint8_t g_x42_rx_buffer[X42_RX_BUFFER_SIZE];
static volatile uint8_t g_x42_rx_head;
static volatile uint8_t g_x42_rx_tail;
static volatile uint8_t g_x42_rx_overflow;
static volatile uint8_t g_k230_rx_buffer[K230_RX_BUFFER_SIZE];
static volatile uint8_t g_k230_rx_head;
static volatile uint8_t g_k230_rx_tail;
static volatile uint8_t g_k230_rx_overflow;
static volatile int32_t g_right_count;
static uint32_t g_system_time_ms;
static uint8_t g_right_encoder_state;
static KeyState g_keys[4];
static uint16_t g_lost_line_ticks;
static uint16_t g_previous_left_count;
static int32_t g_previous_right_count;
static int16_t g_previous_line_error;
static int16_t g_last_nonzero_line_error;
static int16_t g_target_left;
static int16_t g_target_right;
static int16_t g_forward_speed;
static int16_t g_requested_forward_speed;
static DriveMotionProfile g_forward_profile;
static int8_t g_applied_steering_sign;
static int8_t g_pending_steering_sign;
static uint8_t g_steering_reversal_ticks;
static int16_t g_left_measured_speed;
static int16_t g_right_measured_speed;
static uint32_t g_elapsed_ticks;
static uint8_t g_finish_confirm_ticks;
static uint8_t g_oled_refresh_ticks;
static uint8_t g_oled_dirty;
static SpeedPid g_left_pid;
static SpeedPid g_right_pid;
static StallMonitor g_left_stall;
static StallMonitor g_right_stall;
static MotorDirectionGuard g_left_direction_guard;
static MotorDirectionGuard g_right_direction_guard;
static MotorPwmRamp g_left_pwm_ramp;
static MotorPwmRamp g_right_pwm_ramp;
static TaskMenu g_menu;
static LineDisplayStatus g_display;
static EquipmentState g_equipment_state;
static X42Protocol g_x42_protocol;
static K230BallProtocol g_k230_protocol;
static StepperService g_stepper;
static VisionService g_vision;
static VisionState g_vision_state;
static Task4Run g_task4;
static K230Link g_k230_link;
static uint32_t g_task4_vision_not_before_ms;
static RunStopReason g_stop_reason;
static uint16_t g_peak_acceleration_mm_s2;
static int16_t g_peak_ball_x_tenths_mm;
static uint32_t g_task4_k230_checksum_start;
static uint32_t g_task4_k230_overflow_start;
static uint16_t g_task4_vision_fault_diagnostic_code;
static uint32_t g_task4_vision_fault_age_ms;
static uint32_t g_task4_vision_fault_checksum_errors;
static uint32_t g_task4_vision_fault_rx_overflows;

static uint8_t ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) & pin) != 0U ? 1U : 0U;
}

static void WritePin(GPIO_Regs *port, uint32_t pin, uint8_t high)
{
    if (high != 0U)
    {
        DL_GPIO_setPins(port, pin);
    }
    else
    {
        DL_GPIO_clearPins(port, pin);
    }
}

static int16_t LimitSymmetric(int32_t value, int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

static uint16_t AbsoluteSpeed(int16_t speed)
{
    return (uint16_t)(speed < 0 ? -speed : speed);
}

static void ApplyMotorOutput(const MotorOutput *left,
                             const MotorOutput *right)
{
    WritePin(CAR_GPIO_MOTOR_AIN1_PORT, CAR_GPIO_MOTOR_AIN1_PIN, left->in1);
    WritePin(CAR_GPIO_MOTOR_AIN2_PORT, CAR_GPIO_MOTOR_AIN2_PIN, left->in2);
    WritePin(CAR_GPIO_MOTOR_BIN2_PORT, CAR_GPIO_MOTOR_BIN2_PIN, right->in1);
    WritePin(CAR_GPIO_MOTOR_BIN1_PORT, CAR_GPIO_MOTOR_BIN1_PIN, right->in2);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST,
        (uint32_t)(PWM_PERIOD_TICKS - left->pwm_compare),
        GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST,
        (uint32_t)(PWM_PERIOD_TICKS - right->pwm_compare),
        GPIO_MOTOR_PWM_C1_IDX);
}

static void BrakeNow(void)
{
    MotorOutput left = MotorSafety_MakeOutput(
        0, PWM_PERIOD_TICKS, LEFT_MOTOR_INVERT, 1U);
    MotorOutput right = MotorSafety_MakeOutput(
        0, PWM_PERIOD_TICKS, RIGHT_MOTOR_INVERT, 1U);

    g_target_left = 0;
    g_target_right = 0;
    g_forward_speed = 0;
    g_requested_forward_speed = 0;
    g_applied_steering_sign = 0;
    g_pending_steering_sign = 0;
    g_steering_reversal_ticks = 0U;
    DriveMotionProfile_Reset(&g_forward_profile);
    ApplyMotorOutput(&left, &right);
    SpeedPid_Reset(&g_left_pid);
    SpeedPid_Reset(&g_right_pid);
    MotorPwmRamp_Reset(&g_left_pwm_ramp);
    MotorPwmRamp_Reset(&g_right_pwm_ramp);
    MotorDirectionGuard_Init(&g_left_direction_guard);
    MotorDirectionGuard_Init(&g_right_direction_guard);
}

static bool WriteX42Frame(void *context,
                          const uint8_t *data,
                          uint8_t length)
{
    uint8_t index;

    (void)context;
    for (index = 0U; index < length; ++index)
    {
        uint32_t spins = 0U;

        while (!DL_UART_Main_transmitDataCheck(
            X42_UART_INST, data[index]))
        {
            if (++spins >= UART_TX_SPIN_LIMIT)
            {
                return false;
            }
        }
    }
    return true;
}

static void EnterEquipmentFault(void)
{
    if (g_equipment_state == EQUIPMENT_FAULT)
    {
        return;
    }
    g_equipment_state = EQUIPMENT_FAULT;
    TaskMenu_Stop(&g_menu);
    BrakeNow();
    if (g_task4.state == TASK4_RUN_ACTIVE)
    {
        Task4Run_EmergencyStop(&g_task4, &g_stepper);
    }
    else
    {
        (void)StepperService_Stop(&g_stepper);
        (void)StepperService_Disable(&g_stepper);
    }
    g_oled_dirty = 1U;
}

static void ResetToHome(void)
{
    BrakeNow();
    if (g_task4.state == TASK4_RUN_ACTIVE)
    {
        Task4Run_EmergencyStop(&g_task4, &g_stepper);
    }
    else
    {
        (void)StepperService_Stop(&g_stepper);
        (void)StepperService_Disable(&g_stepper);
    }
    TaskMenu_Init(&g_menu);
    Task4Run_Init(&g_task4);
    VisionService_Init(&g_vision);
    g_vision_state = VisionService_GetState(
        &g_vision, g_system_time_ms, VISION_DEFAULT_TIMEOUT_MS);
    g_k230_link.position = 0U;
    g_k230_link.length = 0U;
    g_task4_vision_not_before_ms = 0U;
    g_elapsed_ticks = 0U;
    g_stop_reason = RUN_STOP_NONE;
    g_peak_acceleration_mm_s2 = 0U;
    g_peak_ball_x_tenths_mm = 0;
    g_equipment_state = EQUIPMENT_HOME;
    g_oled_dirty = 1U;
}

static void RequestK230DefaultOrigin(void)
{
    VisionService_Init(&g_vision);
    g_vision_state = VisionService_GetState(
        &g_vision, g_system_time_ms, VISION_DEFAULT_TIMEOUT_MS);
    g_task4_vision_not_before_ms =
        g_system_time_ms + CONTROL_TICK_MS;
    g_k230_link.position = 0U;
    g_k230_link.length =
        K230BallProtocol_MakeUseDefaultOrigin(
            g_k230_link.frame);
    g_k230_link.deadline_ms =
        g_system_time_ms + K230_TX_TIMEOUT_MS;
}

static void ServiceK230Tx(void)
{
    while (g_k230_link.position < g_k230_link.length)
    {
        if (!DL_UART_Main_transmitDataCheck(
                K230_A_UART_INST,
                g_k230_link.frame[g_k230_link.position]))
        {
            break;
        }
        ++g_k230_link.position;
    }
    if (g_k230_link.position >= g_k230_link.length)
    {
        g_k230_link.position = 0U;
        g_k230_link.length = 0U;
    }
    else if ((int32_t)(g_system_time_ms -
                       g_k230_link.deadline_ms) >= 0)
    {
        g_k230_link.position = 0U;
        g_k230_link.length = 0U;
        ++g_k230_link.timeout_count;
        if (TaskMenu_UsesBalanceControl(&g_menu) != 0U)
        {
            EnterEquipmentFault();
        }
    }
}

static void ProcessX42Rx(void)
{
    X42Frame frame;
    uint8_t overflow;

    while (g_x42_rx_tail != g_x42_rx_head)
    {
        uint8_t byte = g_x42_rx_buffer[g_x42_rx_tail];

        g_x42_rx_tail =
            (uint8_t)((g_x42_rx_tail + 1U) &
                      (X42_RX_BUFFER_SIZE - 1U));
        X42Protocol_PushByte(
            &g_x42_protocol, byte, g_system_time_ms);
        while (X42Protocol_TakeFrame(
                   &g_x42_protocol, &frame) != 0U)
        {
            StepperService_HandleX42Frame(
                &g_stepper, &frame, g_system_time_ms);
        }
    }

    __disable_irq();
    overflow = g_x42_rx_overflow;
    g_x42_rx_overflow = 0U;
    __enable_irq();
    if (overflow != 0U)
    {
        StepperService_ReportRxOverflow(&g_stepper);
    }
}

static void ProcessK230Rx(void)
{
    K230BallFrame frame;
    uint8_t overflow;

    ServiceK230Tx();
    while (g_k230_rx_tail != g_k230_rx_head)
    {
        uint8_t byte = g_k230_rx_buffer[g_k230_rx_tail];

        g_k230_rx_tail =
            (uint8_t)((g_k230_rx_tail + 1U) &
                      (K230_RX_BUFFER_SIZE - 1U));
        K230BallProtocol_PushByte(
            &g_k230_protocol, byte, g_system_time_ms);
        while (K230BallProtocol_TakeLatest(
                   &g_k230_protocol, &frame) != 0U)
        {
            (void)VisionService_PushFrame(&g_vision, &frame);
        }
    }

    __disable_irq();
    overflow = g_k230_rx_overflow;
    g_k230_rx_overflow = 0U;
    __enable_irq();
    if (overflow != 0U)
    {
        ++g_k230_link.overflow_count;
        if (TaskMenu_UsesBalanceControl(&g_menu) != 0U)
        {
            EnterEquipmentFault();
        }
    }
    g_vision_state = VisionService_GetState(
        &g_vision, g_system_time_ms, VISION_DEFAULT_TIMEOUT_MS);
}

static uint8_t GrayscaleScan(void)
{
    uint8_t raw = 0U;

    raw |= ReadPin(CAR_GPIO_GRAY_X0_PORT, CAR_GPIO_GRAY_X0_PIN) << 0U;
    raw |= ReadPin(CAR_GPIO_GRAY_X1_PORT, CAR_GPIO_GRAY_X1_PIN) << 1U;
    raw |= ReadPin(CAR_GPIO_GRAY_X2_PORT, CAR_GPIO_GRAY_X2_PIN) << 2U;
    raw |= ReadPin(CAR_GPIO_GRAY_X3_PORT, CAR_GPIO_GRAY_X3_PIN) << 3U;
    raw |= ReadPin(CAR_GPIO_GRAY_X4_PORT, CAR_GPIO_GRAY_X4_PIN) << 4U;
    raw |= ReadPin(CAR_GPIO_GRAY_X5_PORT, CAR_GPIO_GRAY_X5_PIN) << 5U;
    raw |= ReadPin(CAR_GPIO_GRAY_X6_PORT, CAR_GPIO_GRAY_X6_PIN) << 6U;
    raw |= ReadPin(CAR_GPIO_GRAY_X7_PORT, CAR_GPIO_GRAY_X7_PIN) << 7U;
    return Grayscale_ActiveMask(raw, 0U);
}

static int16_t LineDifferential(uint8_t line_mask)
{
    int16_t error;
    int16_t derivative;
    int16_t derivative_limit;
    int16_t kp_numerator;
    int16_t kd_numerator;
    int16_t denominator;
    int16_t absolute_error;
    int32_t command;
    uint8_t mode_t2 =
        TaskMenu_GetMode(&g_menu) == TASK_MODE_T2_LAP18 ? 1U : 0U;

    if (line_mask == 0U)
    {
        return Grayscale_LostLineCommand(
            g_last_nonzero_line_error,
            mode_t2 != 0U ?
                T2_LOST_LINE_DIFFERENTIAL_MM_S :
                T4_LOST_LINE_DIFFERENTIAL_MM_S);
    }

    error = Grayscale_LineError(line_mask);
    if (error >= -LINE_CENTER_DEADBAND &&
        error <= LINE_CENTER_DEADBAND)
    {
        error = 0;
    }
    absolute_error = error < 0 ? (int16_t)-error : error;
    if (mode_t2 != 0U)
    {
        if (absolute_error >= T2_HIGH_GAIN_ERROR)
        {
            kp_numerator = T2_HIGH_KP_NUMERATOR;
            derivative_limit = T2_HIGH_DERIVATIVE_LIMIT;
        }
        else
        {
            kp_numerator = T2_LOW_KP_NUMERATOR;
            derivative_limit = T2_LOW_DERIVATIVE_LIMIT;
        }
        kd_numerator = T2_KD_NUMERATOR;
        denominator = T2_PD_DENOMINATOR;
    }
    else
    {
        kp_numerator = T4_LINE_KP_NUMERATOR;
        kd_numerator = T4_LINE_KD_NUMERATOR;
        denominator = T4_PD_DENOMINATOR;
        derivative_limit = T4_DERIVATIVE_LIMIT;
    }
    derivative = LimitSymmetric(
        (int32_t)error - g_previous_line_error, derivative_limit);
    g_previous_line_error = error;
    if (error != 0)
    {
        g_last_nonzero_line_error = error;
    }

    command = ((int32_t)kp_numerator * error) +
              ((int32_t)kd_numerator * derivative);
    return (int16_t)(command / denominator);
}

static int16_t FilterSteeringReversal(int16_t requested,
                                      uint8_t immediate)
{
    int8_t requested_sign =
        requested > 0 ? 1 : (requested < 0 ? -1 : 0);

    if (requested_sign == 0)
    {
        g_pending_steering_sign = 0;
        g_steering_reversal_ticks = 0U;
        return 0;
    }
    if (immediate != 0U || g_applied_steering_sign == 0 ||
        requested_sign == g_applied_steering_sign)
    {
        g_applied_steering_sign = requested_sign;
        g_pending_steering_sign = 0;
        g_steering_reversal_ticks = 0U;
        return requested;
    }

    if (g_pending_steering_sign != requested_sign)
    {
        g_pending_steering_sign = requested_sign;
        g_steering_reversal_ticks = 1U;
        return 0;
    }
    if (g_steering_reversal_ticks < STEERING_REVERSAL_CONFIRM_TICKS)
    {
        ++g_steering_reversal_ticks;
    }
    if (g_steering_reversal_ticks < STEERING_REVERSAL_CONFIRM_TICKS)
    {
        return 0;
    }

    g_applied_steering_sign = requested_sign;
    g_pending_steering_sign = 0;
    g_steering_reversal_ticks = 0U;
    return requested;
}

static int16_t ProfileForwardSpeed(void)
{
    if (TaskMenu_GetMode(&g_menu) == TASK_MODE_T2_LAP18)
    {
        return T2_SPEED_MM_S;
    }
    return g_elapsed_ticks < T4_SWITCH_TICKS ?
        T4_AB_SPEED_MM_S : T4_AFTER_B_SPEED_MM_S;
}

static uint16_t ProfileMaximumAcceleration(void)
{
    return TaskMenu_GetMode(&g_menu) == TASK_MODE_T2_LAP18 ?
        T2_FORWARD_ACCEL_MM_S2 : T4_FORWARD_ACCEL_MM_S2;
}

static uint16_t ProfileMaximumJerk(void)
{
    return TaskMenu_GetMode(&g_menu) == TASK_MODE_T2_LAP18 ?
        T2_FORWARD_JERK_MM_S3 : T4_FORWARD_JERK_MM_S3;
}

static uint16_t RunStopTicks(void)
{
    return TaskMenu_GetMode(&g_menu) == TASK_MODE_T2_LAP18 ?
        T2_TIMEOUT_TICKS : T4_STOP_TICKS;
}

static void UpdateMotionTargets(uint8_t line_mask)
{
    int16_t differential = 0;
    DriveWheelTargets targets;

    if (TaskMenu_IsRunning(&g_menu) != 0U)
    {
        g_requested_forward_speed =
            line_mask == 0U ? LOST_LINE_SPEED_MM_S : ProfileForwardSpeed();
        differential = LineDifferential(line_mask);
        differential = FilterSteeringReversal(
            differential, line_mask == 0U ? 1U : 0U);
    }
    else
    {
        g_requested_forward_speed = 0;
    }

    g_forward_speed = DriveMotionProfile_Update(
        &g_forward_profile, g_requested_forward_speed,
        ProfileMaximumAcceleration(), ProfileMaximumJerk(),
        CONTROL_TICK_MS);
    targets = DriveLogic_MixDifferential(
        g_forward_speed, differential, MAX_WHEEL_SPEED_MM_S);
    g_target_left = targets.left_mm_s;
    g_target_right = targets.right_mm_s;
}

static void StartLineFollow(void)
{
    if (g_equipment_state != EQUIPMENT_READY)
    {
        return;
    }
    BrakeNow();
    if (TaskMenu_Start(&g_menu) == 0U)
    {
        return;
    }
    if (TaskMenu_UsesBalanceControl(&g_menu) != 0U &&
        Task4Run_Start(&g_task4, &g_stepper,
                       &g_vision_state,
                       g_system_time_ms) == 0U)
    {
        TaskMenu_Stop(&g_menu);
        Task4Run_Init(&g_task4);
        g_oled_dirty = 1U;
        return;
    }
    if (TaskMenu_UsesBalanceControl(&g_menu) != 0U)
    {
        g_task4_k230_checksum_start =
            g_k230_protocol.checksum_error_count;
        g_task4_k230_overflow_start =
            g_k230_link.overflow_count;
        g_task4_vision_fault_diagnostic_code = 0U;
        g_task4_vision_fault_age_ms = 0U;
        g_task4_vision_fault_checksum_errors = 0U;
        g_task4_vision_fault_rx_overflows = 0U;
    }
    g_lost_line_ticks = 0U;
    g_finish_confirm_ticks = 0U;
    g_previous_line_error = 0;
    g_last_nonzero_line_error = 0;
    g_elapsed_ticks = 0U;
    g_stop_reason = RUN_STOP_NONE;
    g_peak_acceleration_mm_s2 = 0U;
    g_peak_ball_x_tenths_mm = 0;
    g_oled_refresh_ticks = 0U;
    g_oled_dirty = 1U;
}

static void TryStartConfirmedTask(void)
{
    if (g_equipment_state != EQUIPMENT_READY ||
        TaskMenu_IsConfirmed(&g_menu) == 0U ||
        TaskMenu_IsRunning(&g_menu) != 0U)
    {
        return;
    }
    if (TaskMenu_UsesBalanceControl(&g_menu) != 0U &&
        (g_vision_state.valid == 0U ||
         (int32_t)(g_vision.last_frame_ms -
                   g_task4_vision_not_before_ms) < 0))
    {
        return;
    }
    StartLineFollow();
}

static void StopLineFollow(RunStopReason reason)
{
    uint8_t task4_stop_ok = 1U;

    if (g_task4.state == TASK4_RUN_ACTIVE)
    {
        task4_stop_ok =
            Task4Run_Stop(&g_task4, &g_stepper);
    }
    TaskMenu_Stop(&g_menu);
    g_stop_reason = reason;
    g_requested_forward_speed = 0;
    g_finish_confirm_ticks = 0U;
    BrakeNow();
    g_oled_dirty = 1U;
    if (task4_stop_ok == 0U)
    {
        EnterEquipmentFault();
    }
}

static uint8_t ReadKeyPressed(uint8_t index)
{
    switch (index)
    {
        case 0U:
            return (uint8_t)!ReadPin(
                CAR_GPIO_KEY1_PORT, CAR_GPIO_KEY1_PIN);
        case 1U:
            return (uint8_t)!ReadPin(
                CAR_GPIO_KEY2_PORT, CAR_GPIO_KEY2_PIN);
        case 2U:
            return (uint8_t)!ReadPin(
                CAR_GPIO_KEY3_PORT, CAR_GPIO_KEY3_PIN);
        default:
            return (uint8_t)!ReadPin(
                CAR_GPIO_KEY4_PORT, CAR_GPIO_KEY4_PIN);
    }
}

static uint8_t ScanKeyPress(uint8_t index)
{
    uint8_t pressed = ReadKeyPressed(index);
    KeyState *key = &g_keys[index];

    if (pressed == key->stable_pressed)
    {
        key->debounce_ticks = 0U;
        return 0U;
    }
    if (++key->debounce_ticks < KEY_DEBOUNCE_TICKS)
    {
        return 0U;
    }

    key->stable_pressed = pressed;
    key->debounce_ticks = 0U;
    return pressed;
}

static void HandleKeys(void)
{
    uint8_t key1 = ScanKeyPress(0U);
    uint8_t key2 = ScanKeyPress(1U);
    uint8_t key3 = ScanKeyPress(2U);
    uint8_t key4 = ScanKeyPress(3U);

    if (key4 != 0U)
    {
        ResetToHome();
        return;
    }
    if (g_equipment_state == EQUIPMENT_HOME)
    {
        if (key1 != 0U &&
            StepperService_BeginLeveling(
                &g_stepper, g_system_time_ms))
        {
            g_equipment_state = EQUIPMENT_LEVELING;
            g_oled_dirty = 1U;
        }
        return;
    }
    if (g_equipment_state != EQUIPMENT_READY)
    {
        return;
    }
    if (TaskMenu_IsRunning(&g_menu) != 0U ||
        TaskMenu_IsConfirmed(&g_menu) != 0U)
    {
        return;
    }
    if (key2 != 0U)
    {
        TaskMenu_SelectNext(&g_menu);
        g_stop_reason = RUN_STOP_NONE;
        g_oled_dirty = 1U;
        return;
    }
    if (key3 != 0U)
    {
        TaskMenu_Confirm(&g_menu);
        g_stop_reason = RUN_STOP_NONE;
        g_elapsed_ticks = 0U;
        if (TaskMenu_UsesBalanceControl(&g_menu) != 0U)
        {
            RequestK230DefaultOrigin();
        }
        g_oled_dirty = 1U;
    }
}

static void UpdateSpeedControl(void)
{
    uint16_t left_now = (uint16_t)DL_TimerG_getTimerCount(LEFT_QEI_INST);
    int32_t right_now = g_right_count;
    int16_t left_delta =
        EncoderLogic_Delta16(left_now, g_previous_left_count);
    int32_t right_delta = right_now - g_previous_right_count;
    int16_t left_pwm;
    int16_t right_pwm;
    MotorOutput left_output;
    MotorOutput right_output;

    g_previous_left_count = left_now;
    g_previous_right_count = right_now;
    g_left_measured_speed = (int16_t)DriveLogic_CountsToMmPerSecond(
        LEFT_ENCODER_SIGN * left_delta, CONTROL_TICK_MS);
    g_right_measured_speed = (int16_t)DriveLogic_CountsToMmPerSecond(
        RIGHT_ENCODER_SIGN * right_delta, CONTROL_TICK_MS);

    if (TaskMenu_IsRunning(&g_menu) == 0U &&
        g_target_left == 0 && g_target_right == 0)
    {
        BrakeNow();
        return;
    }

    left_pwm = (int16_t)SpeedPid_Update(
        &g_left_pid, (float)g_target_left,
        (float)g_left_measured_speed, 0.01f);
    right_pwm = (int16_t)SpeedPid_Update(
        &g_right_pid, (float)g_target_right,
        (float)g_right_measured_speed, 0.01f);
    left_pwm = MotorDirectionGuard_Apply(
        &g_left_direction_guard, left_pwm, DIRECTION_BRAKE_TICKS);
    right_pwm = MotorDirectionGuard_Apply(
        &g_right_direction_guard, right_pwm, DIRECTION_BRAKE_TICKS);

    if (MotorDirectionGuard_IsBraking(&g_left_direction_guard) != 0U)
    {
        MotorPwmRamp_Reset(&g_left_pwm_ramp);
        left_pwm = 0;
    }
    else
    {
        left_pwm = MotorPwmRamp_Apply(
            &g_left_pwm_ramp, left_pwm, PWM_RAMP_STEP_TICKS);
    }

    if (MotorDirectionGuard_IsBraking(&g_right_direction_guard) != 0U)
    {
        MotorPwmRamp_Reset(&g_right_pwm_ramp);
        right_pwm = 0;
    }
    else
    {
        right_pwm = MotorPwmRamp_Apply(
            &g_right_pwm_ramp, right_pwm, PWM_RAMP_STEP_TICKS);
    }

    if (StallMonitor_Update(
            &g_left_stall, g_target_left, g_left_measured_speed) != 0U ||
        StallMonitor_Update(
            &g_right_stall, g_target_right, g_right_measured_speed) != 0U)
    {
        StopLineFollow(RUN_STOP_STALL);
        return;
    }

    left_output = MotorSafety_MakeOutput(
        left_pwm, PWM_PERIOD_TICKS, LEFT_MOTOR_INVERT, 0U);
    right_output = MotorSafety_MakeOutput(
        right_pwm, PWM_PERIOD_TICKS, RIGHT_MOTOR_INVERT, 0U);
    ApplyMotorOutput(&left_output, &right_output);
}

static void UpdateDisplayStatus(void)
{
    uint32_t speed_sum =
        (uint32_t)AbsoluteSpeed(g_left_measured_speed) +
        AbsoluteSpeed(g_right_measured_speed);
    uint16_t acceleration_magnitude = (uint16_t)(
        g_forward_profile.acceleration_mm_s2 < 0
            ? -g_forward_profile.acceleration_mm_s2
            : g_forward_profile.acceleration_mm_s2);

    g_display.mode = TaskMenu_GetMode(&g_menu);
    if (TaskMenu_IsRunning(&g_menu) != 0U)
    {
        if (acceleration_magnitude > g_peak_acceleration_mm_s2)
        {
            g_peak_acceleration_mm_s2 = acceleration_magnitude;
        }
        if (g_vision_state.valid != 0U &&
            AbsoluteSpeed(g_vision_state.x_tenths_mm) >
                AbsoluteSpeed(g_peak_ball_x_tenths_mm))
        {
            g_peak_ball_x_tenths_mm =
                g_vision_state.x_tenths_mm;
        }
    }
    if (g_equipment_state == EQUIPMENT_FAULT)
    {
        g_display.phase = RUN_DISPLAY_FAULT;
    }
    else if (g_equipment_state == EQUIPMENT_HOME)
    {
        g_display.phase = RUN_DISPLAY_HOME;
    }
    else if (g_equipment_state == EQUIPMENT_LEVELING)
    {
        g_display.phase = RUN_DISPLAY_LEVEL;
    }
    else if (TaskMenu_IsRunning(&g_menu) != 0U)
    {
        g_display.phase = RUN_DISPLAY_RUNNING;
    }
    else if (TaskMenu_IsConfirmed(&g_menu) != 0U)
    {
        g_display.phase = RUN_DISPLAY_WAIT;
    }
    else if (g_stop_reason != RUN_STOP_NONE)
    {
        g_display.phase = RUN_DISPLAY_STOPPED;
    }
    else
    {
        g_display.phase = RUN_DISPLAY_SELECT;
    }
    g_display.maximum_acceleration_mm_s2 =
        ProfileMaximumAcceleration();
    g_display.stop_reason = g_stop_reason;
    g_display.peak_acceleration_mm_s2 =
        g_peak_acceleration_mm_s2;
    g_display.peak_ball_x_tenths_mm =
        g_peak_ball_x_tenths_mm;
    g_display.current_speed_mm_s = (uint16_t)(speed_sum / 2U);
    g_display.elapsed_centiseconds = g_elapsed_ticks;
    g_display.ball_valid = g_vision_state.valid;
    g_display.ball_x_tenths_mm =
        g_vision_state.x_tenths_mm;
    g_display.motor_angle_cdeg =
        StepperService_GetRelativeAngleCdeg(&g_stepper);
    g_display.vision_diagnostic_code =
        VisionService_GetDiagnosticCode(&g_vision);
    g_display.task4_fault_code = (uint8_t)g_task4.fault;
    g_display.task4_vision_fault_diagnostic_code =
        g_task4_vision_fault_diagnostic_code;
    g_display.task4_vision_fault_age_ms =
        g_task4_vision_fault_age_ms;
    g_display.task4_vision_fault_checksum_errors =
        g_task4_vision_fault_checksum_errors;
    g_display.task4_vision_fault_rx_overflows =
        g_task4_vision_fault_rx_overflows;

    if (++g_oled_refresh_ticks >= OLED_REFRESH_TICKS)
    {
        g_oled_refresh_ticks = 0U;
        g_oled_dirty = 1U;
    }
}

static void ControlTick(void)
{
    uint8_t line_mask;

    g_system_time_ms += CONTROL_TICK_MS;
    g_vision_state = VisionService_GetState(
        &g_vision, g_system_time_ms, VISION_DEFAULT_TIMEOUT_MS);
    StepperService_Update(&g_stepper, g_system_time_ms);
    if (StepperService_GetStatus(&g_stepper) ==
        STEPPER_STATUS_FAULT)
    {
        EnterEquipmentFault();
    }
    else if (g_equipment_state == EQUIPMENT_LEVELING &&
             StepperService_GetStatus(&g_stepper) ==
                 STEPPER_STATUS_READY)
    {
        g_equipment_state = EQUIPMENT_READY;
        g_oled_dirty = 1U;
    }

    HandleKeys();
    TryStartConfirmedTask();
    if (g_task4.state == TASK4_RUN_ACTIVE)
    {
        Task4Run_Update(&g_task4, &g_stepper,
                        &g_vision_state,
                        (int16_t)g_forward_profile.acceleration_mm_s2,
                        g_system_time_ms);
        if (g_task4.state == TASK4_RUN_FAULT)
        {
            if (g_task4.fault == TASK4_FAULT_VISION_STALE)
            {
                g_task4_vision_fault_diagnostic_code =
                    VisionService_GetDiagnosticCode(&g_vision);
                g_task4_vision_fault_age_ms =
                    g_vision_state.age_ms;
                g_task4_vision_fault_checksum_errors =
                    g_k230_protocol.checksum_error_count -
                    g_task4_k230_checksum_start;
                g_task4_vision_fault_rx_overflows =
                    g_k230_link.overflow_count -
                    g_task4_k230_overflow_start;
            }
            EnterEquipmentFault();
        }
    }
    line_mask = GrayscaleScan();
    if (TaskMenu_IsRunning(&g_menu) != 0U)
    {
        if (line_mask == 0U)
        {
            if (++g_lost_line_ticks >= LOST_LINE_STOP_TICKS)
            {
                StopLineFollow(RUN_STOP_LINE);
            }
        }
        else
        {
            g_lost_line_ticks = 0U;
        }

        if (TaskMenu_IsRunning(&g_menu) != 0U)
        {
            if (TaskMenu_GetMode(&g_menu) == TASK_MODE_T2_LAP18 &&
                g_elapsed_ticks >= T2_FINISH_ARM_TICKS &&
                Grayscale_ActiveCount(line_mask) >=
                    FINISH_ACTIVE_SENSOR_COUNT)
            {
                if (++g_finish_confirm_ticks >= FINISH_CONFIRM_TICKS)
                {
                    StopLineFollow(RUN_STOP_FINISH);
                }
            }
            else
            {
                g_finish_confirm_ticks = 0U;
            }
        }

        if (TaskMenu_IsRunning(&g_menu) != 0U)
        {
            if (g_elapsed_ticks >= RunStopTicks())
            {
                StopLineFollow(RUN_STOP_TIME);
            }
            else
            {
                ++g_elapsed_ticks;
            }
        }
    }
    UpdateMotionTargets(line_mask);
    UpdateSpeedControl();
    UpdateDisplayStatus();
}

int main(void)
{
    uint8_t index;

    SYSCFG_DL_init();
    TaskMenu_Init(&g_menu);
    X42Protocol_Init(&g_x42_protocol);
    K230BallProtocol_Init(&g_k230_protocol);
    StepperService_Init(
        &g_stepper, WriteX42Frame, (void *)0);
    VisionService_Init(&g_vision);
    Task4Run_Init(&g_task4);
    g_equipment_state = EQUIPMENT_HOME;
    SpeedPid_Init(
        &g_left_pid, 0.8f, 2.0f, 0.002f, PWM_PERIOD_TICKS, 600.0f);
    SpeedPid_Init(
        &g_right_pid, 0.8f, 2.0f, 0.002f, PWM_PERIOD_TICKS, 600.0f);
    StallMonitor_Init(&g_left_stall, STALL_MIN_TARGET_MM_S, 30, 100U);
    StallMonitor_Init(&g_right_stall, STALL_MIN_TARGET_MM_S, 30, 100U);
    MotorDirectionGuard_Init(&g_left_direction_guard);
    MotorDirectionGuard_Init(&g_right_direction_guard);
    MotorPwmRamp_Init(&g_left_pwm_ramp);
    MotorPwmRamp_Init(&g_right_pwm_ramp);
    for (index = 0U; index < 4U; ++index)
    {
        g_keys[index].stable_pressed = ReadKeyPressed(index);
        g_keys[index].debounce_ticks = 0U;
    }
    g_right_encoder_state = (uint8_t)(
        ReadPin(CAR_GPIO_RIGHT_ENCODER_A_PORT,
                CAR_GPIO_RIGHT_ENCODER_A_PIN) |
        (ReadPin(CAR_GPIO_RIGHT_ENCODER_B_PORT,
                 CAR_GPIO_RIGHT_ENCODER_B_PIN) << 1U));
    BrakeNow();
    DL_TimerG_startCounter(MOTOR_PWM_INST);
    DL_TimerG_startCounter(LEFT_QEI_INST);
    DL_TimerG_startCounter(CONTROL_TIMER_INST);
    NVIC_EnableIRQ(CAR_GPIO_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(X42_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_A_UART_INST_INT_IRQN);
    (void)OLED_Init();
    UpdateDisplayStatus();
    OLED_ShowLineStatus(&g_display);

    while (1)
    {
        ProcessX42Rx();
        ProcessK230Rx();
        if (g_control_ticks_pending != 0U)
        {
            __disable_irq();
            --g_control_ticks_pending;
            __enable_irq();
            ControlTick();
        }
        else if (g_oled_dirty != 0U)
        {
            g_oled_dirty = 0U;
            OLED_ShowLineStatus(&g_display);
        }
        else
        {
            __WFI();
        }
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t pins =
        CAR_GPIO_RIGHT_ENCODER_A_PIN | CAR_GPIO_RIGHT_ENCODER_B_PIN;
    uint32_t status =
        DL_GPIO_getEnabledInterruptStatus(GPIOA, pins);

    if ((status & pins) != 0U)
    {
        uint8_t next = (uint8_t)(
            ReadPin(CAR_GPIO_RIGHT_ENCODER_A_PORT,
                    CAR_GPIO_RIGHT_ENCODER_A_PIN) |
            (ReadPin(CAR_GPIO_RIGHT_ENCODER_B_PORT,
                     CAR_GPIO_RIGHT_ENCODER_B_PIN) << 1U));
        uint8_t valid;

        g_right_count += EncoderLogic_Transition(
            g_right_encoder_state, next, &valid);
        g_right_encoder_state = next;
        DL_GPIO_clearInterruptStatus(GPIOA, status & pins);
    }
}

void TIMG6_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST) ==
        DL_TIMER_IIDX_ZERO)
    {
        if (g_control_ticks_pending < 255U)
        {
            ++g_control_ticks_pending;
        }
    }
}

void UART1_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(X42_UART_INST) ==
        DL_UART_MAIN_IIDX_RX)
    {
        while (!DL_UART_Main_isRXFIFOEmpty(X42_UART_INST))
        {
            uint8_t next =
                (uint8_t)((g_x42_rx_head + 1U) &
                          (X42_RX_BUFFER_SIZE - 1U));
            uint8_t byte =
                (uint8_t)DL_UART_Main_receiveData(X42_UART_INST);

            if (next == g_x42_rx_tail)
            {
                g_x42_rx_overflow = 1U;
            }
            else
            {
                g_x42_rx_buffer[g_x42_rx_head] = byte;
                g_x42_rx_head = next;
            }
        }
    }
}

void UART3_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(K230_A_UART_INST) ==
        DL_UART_MAIN_IIDX_RX)
    {
        while (!DL_UART_Main_isRXFIFOEmpty(K230_A_UART_INST))
        {
            uint8_t next =
                (uint8_t)((g_k230_rx_head + 1U) &
                          (K230_RX_BUFFER_SIZE - 1U));
            uint8_t byte = (uint8_t)DL_UART_Main_receiveData(
                K230_A_UART_INST);

            if (next == g_k230_rx_tail)
            {
                g_k230_rx_overflow = 1U;
            }
            else
            {
                g_k230_rx_buffer[g_k230_rx_head] = byte;
                g_k230_rx_head = next;
            }
        }
    }
}
