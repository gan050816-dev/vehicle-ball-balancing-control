/** H题车载平衡滚球运动控制系统首版任务框架。 */
#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#include "app_controller.h"
#include "balance_drive_run.h"
#include "balance_vision_service.h"
#include "button_driver.h"
#include "encoder_logic.h"
#include "grayscale_logic.h"
#include "k230_ball_protocol.h"
#include "line_drive_service.h"
#include "oled_ssd1306.h"
#include "safety_manager.h"
#include "stepper_service.h"
#include "task3_controller.h"
#include "x42_protocol.h"

static volatile uint32_t g_system_time_ms;

#define K230_RX_BUFFER_SIZE 128U
#define X42_RX_BUFFER_SIZE 64U
#define X42_TX_TIMEOUT_MS 20U
#define OLED_RUNNING_REFRESH_PERIOD_MS 2000U
/* USB转TTL的T3 CSV日志代码保留，但当前不参与编译和运行。 */
#define TASK3_USB_TTL_LOG_ENABLED 0
#if TASK3_USB_TTL_LOG_ENABLED
#define TASK3_LOG_PERIOD_MS 100U
#define TASK3_LOG_TX_BUFFER_SIZE 32U
#endif
#define K230_CAPTURE_RESPONSE_TIMEOUT_MS 2500U
#define K230_CAPTURE_FRESH_FRAME_TIMEOUT_MS 500U
#define K230_TX_TIMEOUT_MS 100U
#define VISION_READY_ACCEPTED_FRAME_COUNT 3U

typedef struct
{
    K230BallProtocol protocol;
    K230BallFrame latest;
    uint8_t latest_available;
    uint8_t sequence_seen;
    uint8_t last_sequence;
    uint8_t capture_request_id;
    uint8_t capture_pending;
    uint8_t capture_waiting_for_frame;
    uint8_t capture_status;
    uint16_t captured_center_x_px;
    uint8_t tx_frame[K230_BALL_CAPTURE_REQUEST_FRAME_SIZE];
    uint8_t tx_length;
    uint8_t tx_position;
    uint32_t tx_deadline_ms;
    uint32_t capture_deadline_ms;
    uint32_t sequence_drop_count;
    uint32_t duplicate_count;
    uint32_t overflow_count;
    uint32_t stale_capture_result_count;
    uint32_t capture_timeout_count;
    uint32_t tx_timeout_count;
} MainK230Link;

MainK230Link g_k230_link;

typedef struct
{
    X42Protocol protocol;
    uint32_t overflow_count;
} MainX42Link;

MainX42Link g_x42_link;

#if TASK3_USB_TTL_LOG_ENABLED
typedef struct
{
    uint8_t tx_buffer[TASK3_LOG_TX_BUFFER_SIZE];
    uint8_t tx_length;
    uint8_t tx_position;
    uint8_t run_active;
    uint8_t header_sent;
    uint32_t next_sample_ms;
} MainTask3Log;

MainTask3Log g_task3_log;
#endif

static volatile uint8_t g_k230_rx_buffer[K230_RX_BUFFER_SIZE];
static volatile uint8_t g_k230_rx_head;
static volatile uint8_t g_k230_rx_tail;
static volatile uint8_t g_k230_rx_overflow;
static volatile uint8_t g_x42_rx_buffer[X42_RX_BUFFER_SIZE];
static volatile uint8_t g_x42_rx_head;
static volatile uint8_t g_x42_rx_tail;
static volatile uint8_t g_x42_rx_overflow;
static volatile uint8_t g_line_drive_ticks_pending;
static volatile int32_t g_right_encoder_count;
static int32_t g_left_encoder_count;
static uint16_t g_left_encoder_raw;
static uint8_t g_right_encoder_state;
static LineDriveService g_line_drive;
static uint32_t g_balance_vision_not_before_ms;
static uint8_t g_vision_ready_streak;

void SysTick_Handler(void)
{
    ++g_system_time_ms;
}

static uint8_t Main_ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) & pin) != 0U ? 1U : 0U;
}

static void Main_WritePin(GPIO_Regs *port, uint32_t pin, uint8_t high)
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

static uint8_t Main_GrayscaleScan(void)
{
    uint8_t raw = 0U;

    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X0_PORT, CAR_GPIO_GRAY_X0_PIN) << 0U;
    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X1_PORT, CAR_GPIO_GRAY_X1_PIN) << 1U;
    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X2_PORT, CAR_GPIO_GRAY_X2_PIN) << 2U;
    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X3_PORT, CAR_GPIO_GRAY_X3_PIN) << 3U;
    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X4_PORT, CAR_GPIO_GRAY_X4_PIN) << 4U;
    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X5_PORT, CAR_GPIO_GRAY_X5_PIN) << 5U;
    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X6_PORT, CAR_GPIO_GRAY_X6_PIN) << 6U;
    raw |= Main_ReadPin(
        CAR_GPIO_GRAY_X7_PORT, CAR_GPIO_GRAY_X7_PIN) << 7U;
    return Grayscale_ActiveMask(raw, 0U);
}

static void Main_ApplyDriveOutput(const LineDriveService *drive)
{
    const MotorOutput *left = &drive->left_output;
    const MotorOutput *right = &drive->right_output;

    Main_WritePin(
        CAR_GPIO_MOTOR_AIN1_PORT,
        CAR_GPIO_MOTOR_AIN1_PIN,
        left->in1);
    Main_WritePin(
        CAR_GPIO_MOTOR_AIN2_PORT,
        CAR_GPIO_MOTOR_AIN2_PIN,
        left->in2);
    Main_WritePin(
        CAR_GPIO_MOTOR_BIN2_PORT,
        CAR_GPIO_MOTOR_BIN2_PIN,
        right->in1);
    Main_WritePin(
        CAR_GPIO_MOTOR_BIN1_PORT,
        CAR_GPIO_MOTOR_BIN1_PIN,
        right->in2);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST,
        (uint32_t)(1600U - left->pwm_compare),
        GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST,
        (uint32_t)(1600U - right->pwm_compare),
        GPIO_MOTOR_PWM_C1_IDX);
}

static bool Main_WriteStepperFrame(
    void *context, const uint8_t *data, uint8_t length)
{
    uint8_t index;
    uint32_t deadline_ms =
        g_system_time_ms + X42_TX_TIMEOUT_MS;

    (void)context;
    for (index = 0U; index < length; ++index)
    {
        while (!DL_UART_Main_transmitDataCheck(
            STEPPER_UART_INST, data[index]))
        {
            if ((int32_t)(g_system_time_ms - deadline_ms) >= 0)
            {
                return false;
            }
        }
    }
    return true;
}

#if TASK3_USB_TTL_LOG_ENABLED
static void Main_Task3LogAppendChar(MainTask3Log *log, char value)
{
    if (log->tx_length < TASK3_LOG_TX_BUFFER_SIZE)
    {
        log->tx_buffer[log->tx_length++] = (uint8_t)value;
    }
}

static void Main_Task3LogAppendText(MainTask3Log *log, const char *text)
{
    while (*text != '\0')
    {
        Main_Task3LogAppendChar(log, *text++);
    }
}

static void Main_Task3LogAppendU32(MainTask3Log *log, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (count != 0U)
    {
        Main_Task3LogAppendChar(log, digits[--count]);
    }
}

static void Main_Task3LogAppendTenthsMm(
    MainTask3Log *log, int16_t x_tenths_mm)
{
    int32_t signed_value = x_tenths_mm;
    uint32_t magnitude;

    if (signed_value < 0)
    {
        Main_Task3LogAppendChar(log, '-');
        magnitude = (uint32_t)(-signed_value);
    }
    else
    {
        magnitude = (uint32_t)signed_value;
    }
    Main_Task3LogAppendU32(log, magnitude / 10U);
    Main_Task3LogAppendChar(log, '.');
    Main_Task3LogAppendChar(log, (char)('0' + (magnitude % 10U)));
}

static void Main_Task3LogQueueHeader(MainTask3Log *log)
{
    log->tx_length = 0U;
    log->tx_position = 0U;
    Main_Task3LogAppendText(log, "t_ms,x_mm,phase\r\n");
}

static void Main_Task3LogQueueSample(
    MainTask3Log *log,
    const AppController *app,
    const Task3Controller *task3,
    uint32_t now_ms)
{
    VisionState vision = {0};

    log->tx_length = 0U;
    log->tx_position = 0U;
    Main_Task3LogAppendU32(log, AppController_GetElapsedMs(app));
    Main_Task3LogAppendChar(log, ',');
    if (Task3Controller_GetVisionState(task3, now_ms, &vision))
    {
        Main_Task3LogAppendTenthsMm(log, vision.x_filtered_tenths_mm);
    }
    else
    {
        Main_Task3LogAppendText(log, "NA");
    }
    Main_Task3LogAppendChar(log, ',');
    Main_Task3LogAppendU32(log, (uint32_t)task3->phase);
    Main_Task3LogAppendText(log, "\r\n");
}

static void Main_ServiceTask3LogTx(MainTask3Log *log)
{
    while (log->tx_position < log->tx_length)
    {
        if (!DL_UART_Main_transmitDataCheck(
                T3_LOG_UART_INST,
                log->tx_buffer[log->tx_position]))
        {
            break;
        }
        log->tx_position++;
    }
    if (log->tx_position >= log->tx_length)
    {
        log->tx_length = 0U;
        log->tx_position = 0U;
    }
}

static void Main_ProcessTask3Log(
    MainTask3Log *log,
    const AppController *app,
    const Task3Controller *task3,
    uint32_t now_ms)
{
    bool running = AppController_GetState(app) == APP_STATE_RUNNING &&
        AppController_GetTask(app) == TASK_ID_3_BALL_MOVE;
    uint32_t elapsed_ms;

    if (!running)
    {
        Main_ServiceTask3LogTx(log);
        if (log->tx_length == 0U)
        {
            log->run_active = 0U;
            log->header_sent = 0U;
            log->tx_position = 0U;
        }
        return;
    }

    elapsed_ms = AppController_GetElapsedMs(app);
    if (log->run_active == 0U)
    {
        log->run_active = 1U;
        log->next_sample_ms = elapsed_ms + TASK3_LOG_PERIOD_MS;
    }

    Main_ServiceTask3LogTx(log);
    if (log->tx_length != 0U)
    {
        return;
    }
    if (log->header_sent == 0U)
    {
        Main_Task3LogQueueHeader(log);
        log->header_sent = 1U;
    }
    else if ((int32_t)(elapsed_ms - log->next_sample_ms) >= 0)
    {
        Main_Task3LogQueueSample(log, app, task3, now_ms);
        log->next_sample_ms = elapsed_ms + TASK3_LOG_PERIOD_MS;
    }
    Main_ServiceTask3LogTx(log);
}
#endif

static AppKey Main_ButtonToAppKey(uint8_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_SELECT: return APP_KEY_SELECT;
        case BUTTON_EVENT_CONFIRM: return APP_KEY_CONFIRM;
        case BUTTON_EVENT_RESET: return APP_KEY_RESET;
        case BUTTON_EVENT_CAPTURE: return APP_KEY_CAPTURE;
        default: return APP_KEY_RESET;
    }
}

static uint8_t Main_GetSingleButtonEvent(uint8_t events)
{
    if (events == BUTTON_EVENT_NONE)
    {
        return BUTTON_EVENT_NONE;
    }

    // 复位键具有最高优先级。
    if ((events & BUTTON_EVENT_RESET) != 0U)
    {
        return BUTTON_EVENT_RESET;
    }

    // 同时按下多个普通按键时忽略本次输入，避免误启动。
    if ((events & (uint8_t)(events - 1U)) != 0U)
    {
        return BUTTON_EVENT_NONE;
    }
    return events;
}

static bool Main_IsBalanceDriveTask(TaskId task)
{
    return task == TASK_ID_4_TO_B_BALANCE ||
           task == TASK_ID_5_LAP_CENTER ||
           task == TASK_ID_6_LAP_TARGET;
}

static bool Main_TaskNeedsVision(TaskId task)
{
    return task == TASK_ID_3_BALL_MOVE ||
           Main_IsBalanceDriveTask(task);
}

static bool Main_AllowsIdleStepperQueryRetry(AppState state)
{
    return state == APP_STATE_MENU ||
           state == APP_STATE_PLACE_BALL ||
           state == APP_STATE_CAPTURE_REQUIRED ||
           state == APP_STATE_CAPTURE_READY ||
           state == APP_STATE_READY;
}

static bool Main_VisionIsFresh(
    const BalanceVisionService *vision_service, uint32_t now_ms)
{
    BalanceVisionState vision = BalanceVisionService_GetState(
        vision_service, now_ms, BALANCE_VISION_DEFAULT_TIMEOUT_MS);

    return vision.valid != 0U &&
           (int32_t)(vision_service->last_frame_ms -
                     g_balance_vision_not_before_ms) >= 0;
}

static bool Main_BallCandidateIsValid(const K230BallFrame *frame)
{
    return frame->valid != 0U &&
           frame->multiple_candidates == 0U &&
           frame->confidence_permille >=
               BALANCE_VISION_MIN_CONFIDENCE_PERMILLE;
}

static bool Main_BallCandidateIsFresh(
    const MainK230Link *k230, uint32_t now_ms)
{
    return k230->latest_available != 0U &&
           Main_BallCandidateIsValid(&k230->latest) &&
           (now_ms - k230->latest.received_at_ms) <=
               BALANCE_VISION_DEFAULT_TIMEOUT_MS &&
           (int32_t)(k230->latest.received_at_ms -
                     g_balance_vision_not_before_ms) >= 0;
}

static LineDriveMode Main_GetBalanceDriveMode(TaskId task)
{
    return task == TASK_ID_4_TO_B_BALANCE
               ? LINE_DRIVE_MODE_REQUIREMENT_4
               : LINE_DRIVE_MODE_REQUIREMENT_5;
}

static void Main_CancelK230Capture(MainK230Link *k230)
{
    k230->capture_pending = 0U;
    k230->capture_waiting_for_frame = 0U;
    k230->tx_length = 0U;
    k230->tx_position = 0U;
}

static void Main_RequestK230Capture(MainK230Link *k230, uint32_t now_ms)
{
    Main_CancelK230Capture(k230);
    k230->capture_request_id++;
    k230->capture_status = K230_BALL_CAPTURE_STATUS_TIMEOUT;
    k230->tx_position = 0U;
    k230->tx_length = K230BallProtocol_MakeCaptureRequest(
        k230->capture_request_id, k230->tx_frame);
    k230->tx_deadline_ms = now_ms + K230_TX_TIMEOUT_MS;
    k230->capture_deadline_ms =
        now_ms + K230_CAPTURE_RESPONSE_TIMEOUT_MS;
    k230->capture_pending = k230->tx_length != 0U ? 1U : 0U;
}

static void Main_RequestK230DefaultOrigin(
    MainK230Link *k230, uint32_t now_ms)
{
    Main_CancelK230Capture(k230);
    k230->tx_length = K230BallProtocol_MakeUseDefaultOrigin(
        k230->tx_frame);
    k230->tx_deadline_ms = now_ms + K230_TX_TIMEOUT_MS;
}

static void Main_ServiceK230Tx(MainK230Link *k230, uint32_t now_ms)
{
    while (k230->tx_position < k230->tx_length)
    {
        if (!DL_UART_Main_transmitDataCheck(
                K230_A_UART_INST,
                k230->tx_frame[k230->tx_position]))
        {
            break;
        }
        k230->tx_position++;
    }

    if (k230->tx_position >= k230->tx_length)
    {
        k230->tx_length = 0U;
        k230->tx_position = 0U;
    }
    else if ((int32_t)(now_ms - k230->tx_deadline_ms) >= 0)
    {
        k230->tx_timeout_count++;
        Main_CancelK230Capture(k230);
    }
}

static void Main_HandleAction(AppController *app, SafetyManager *safety,
                              StepperService *stepper,
                              MainK230Link *k230,
                              Task3Controller *task3,
                              LineDriveService *line_drive,
                              BalanceVisionService *balance_vision,
                              BalanceDriveRun *balance_run,
                              AppAction action, uint32_t now_ms)
{
    switch (action)
    {
        case APP_ACTION_CONFIRM_HOME:
            SafetyManager_ConfirmHome(safety);
            if (!SafetyManager_TryEnableMotion(safety) ||
                !StepperService_BeginLeveling(stepper, now_ms))
            {
                AppController_TaskFailed(app);
            }
            break;

        case APP_ACTION_START_TASK:
            if (Main_TaskNeedsVision(AppController_GetTask(app)) &&
                !Main_VisionIsFresh(balance_vision, now_ms))
            {
                g_vision_ready_streak = 0U;
                AppController_PreparationLost(app);
            }
            else if (!SafetyManager_TryEnableMotion(safety))
            {
                AppController_TaskFailed(app);
            }
            else if (AppController_GetTask(app) ==
                     TASK_ID_3_BALL_MOVE)
            {
                Task3Controller_Start(task3, now_ms);
            }
            else if (AppController_GetTask(app) ==
                     TASK_ID_2_LAP_STOP)
            {
                if (LineDriveService_Start(
                        line_drive,
                        LINE_DRIVE_MODE_T2,
                        g_left_encoder_count,
                        g_right_encoder_count) == 0U)
                {
                    AppController_TaskFailed(app);
                }
            }
            else if (Main_IsBalanceDriveTask(
                         AppController_GetTask(app)))
            {
                BalanceVisionState vision =
                    BalanceVisionService_GetState(
                        balance_vision,
                        now_ms,
                        BALANCE_VISION_DEFAULT_TIMEOUT_MS);
                LineDriveMode drive_mode = Main_GetBalanceDriveMode(
                    AppController_GetTask(app));

                if (BalanceDriveRun_Start(
                        balance_run, stepper, &vision, now_ms) == 0U ||
                    LineDriveService_Start(
                        line_drive,
                        drive_mode,
                        g_left_encoder_count,
                        g_right_encoder_count) == 0U)
                {
                    if (balance_run->state ==
                        BALANCE_DRIVE_ACTIVE)
                    {
                        BalanceDriveRun_EmergencyStop(
                            balance_run, stepper);
                    }
                    AppController_TaskFailed(app);
                }
            }
            break;

        case APP_ACTION_STOP_AND_RESET:
            Main_CancelK230Capture(k230);
            Task3Controller_Cancel(task3);
            BalanceDriveRun_Init(balance_run);
            BalanceVisionService_Init(balance_vision);
            g_vision_ready_streak = 0U;
            LineDriveService_EmergencyStop(line_drive);
            Main_ApplyDriveOutput(line_drive);
            StepperService_EndTracking(stepper);
            (void)StepperService_Stop(stepper);
            /*
             * 复位后界面要求重新人工放到机械下限，因此停止后解除
             * 电机使能；任务完成仅停止运动，仍保持水平位置。
             */
            (void)StepperService_Disable(stepper);
            SafetyManager_RequestStop(safety, SAFETY_STOP_RESET, true);
            break;

        case APP_ACTION_PREPARE_TASK:
            /*
             * 每次准备任务都先清除K230上一次T6捕获点。T3～T5继续
             * 使用固定机械O点；T6随后必须再次按PB27建立本次目标。
             */
            Main_RequestK230DefaultOrigin(k230, now_ms);
            if (Main_TaskNeedsVision(AppController_GetTask(app)))
            {
                if (Main_IsBalanceDriveTask(
                        AppController_GetTask(app)))
                {
                    BalanceDriveRun_Init(balance_run);
                }
                BalanceVisionService_Init(balance_vision);
                g_balance_vision_not_before_ms = now_ms + 1U;
                g_vision_ready_streak = 0U;
            }
            break;

        case APP_ACTION_REQUEST_CAPTURE:
            g_vision_ready_streak = 0U;
            if (Main_BallCandidateIsFresh(k230, now_ms))
            {
                Main_RequestK230Capture(k230, now_ms);
            }
            break;

        case APP_ACTION_NONE:
        default:
            break;
    }
}

static void Main_AcceptK230Frame(MainK230Link *k230,
                                 const K230BallFrame *frame)
{
    if (k230->sequence_seen != 0U)
    {
        uint8_t expected = (uint8_t)(k230->last_sequence + 1U);

        if (frame->sequence == k230->last_sequence)
        {
            k230->duplicate_count++;
        }
        else if (frame->sequence != expected)
        {
            k230->sequence_drop_count +=
                (uint8_t)(frame->sequence - expected);
        }
    }

    k230->sequence_seen = 1U;
    k230->last_sequence = frame->sequence;
    k230->latest = *frame;
    k230->latest_available = 1U;
}

static void Main_ProcessX42Rx(MainX42Link *link,
                              StepperService *stepper,
                              uint32_t now_ms)
{
    uint8_t overflow;
    X42Frame frame;

    while (g_x42_rx_tail != g_x42_rx_head)
    {
        uint8_t byte = g_x42_rx_buffer[g_x42_rx_tail];

        g_x42_rx_tail =
            (uint8_t)((g_x42_rx_tail + 1U) &
                      (X42_RX_BUFFER_SIZE - 1U));
        X42Protocol_PushByte(&link->protocol, byte, now_ms);
        while (X42Protocol_TakeFrame(
                   &link->protocol, &frame) != 0U)
        {
            StepperService_HandleX42Frame(
                stepper, &frame, now_ms);
        }
    }

    __disable_irq();
    overflow = g_x42_rx_overflow;
    g_x42_rx_overflow = 0U;
    __enable_irq();
    if (overflow != 0U)
    {
        link->overflow_count++;
        StepperService_ReportRxOverflow(stepper);
    }
}

static void Main_ProcessK230(MainK230Link *k230,
                             AppController *app,
                             BalanceVisionService *balance_vision,
                             BalanceDriveRun *balance_run,
                             uint32_t now_ms)
{
    uint8_t overflow;
    K230BallFrame frame;
    K230BallCaptureResult capture_result;

    Main_ServiceK230Tx(k230, now_ms);

    while (g_k230_rx_tail != g_k230_rx_head)
    {
        uint8_t byte = g_k230_rx_buffer[g_k230_rx_tail];

        g_k230_rx_tail =
            (uint8_t)((g_k230_rx_tail + 1U) &
                      (K230_RX_BUFFER_SIZE - 1U));
        K230BallProtocol_PushByte(&k230->protocol, byte, now_ms);
        while (K230BallProtocol_TakeLatest(
                   &k230->protocol, &frame) != 0U)
        {
            uint8_t accepted;
            uint8_t fresh;
            AppState state;
            TaskId task;

            Main_AcceptK230Frame(k230, &frame);
            accepted = BalanceVisionService_PushFrame(
                balance_vision, &frame);
            fresh = (int32_t)(frame.received_at_ms -
                              g_balance_vision_not_before_ms) >= 0
                        ? 1U
                        : 0U;
            state = AppController_GetState(app);
            task = AppController_GetTask(app);

            if (k230->capture_waiting_for_frame != 0U &&
                state == APP_STATE_CAPTURE_REQUIRED &&
                task == TASK_ID_6_LAP_TARGET)
            {
                if (accepted != 0U && fresh != 0U)
                {
                    if (g_vision_ready_streak <
                        VISION_READY_ACCEPTED_FRAME_COUNT)
                    {
                        g_vision_ready_streak++;
                    }
                }
                else
                {
                    g_vision_ready_streak = 0U;
                }
                if (g_vision_ready_streak >=
                    VISION_READY_ACCEPTED_FRAME_COUNT)
                {
                    k230->capture_waiting_for_frame = 0U;
                    (void)AppController_CaptureSucceeded(app);
                }
            }
            else if ((state == APP_STATE_PLACE_BALL ||
                      state == APP_STATE_CAPTURE_REQUIRED) &&
                     Main_TaskNeedsVision(task) &&
                     k230->capture_pending == 0U)
            {
                uint8_t preparation_sample_valid = accepted;

                if (state == APP_STATE_CAPTURE_REQUIRED &&
                    task == TASK_ID_6_LAP_TARGET)
                {
                    preparation_sample_valid =
                        Main_BallCandidateIsValid(&frame) ? 1U : 0U;
                }
                if (preparation_sample_valid != 0U && fresh != 0U)
                {
                    if (g_vision_ready_streak <
                        VISION_READY_ACCEPTED_FRAME_COUNT)
                    {
                        g_vision_ready_streak++;
                    }
                }
                else
                {
                    g_vision_ready_streak = 0U;
                }
                if (g_vision_ready_streak >=
                    VISION_READY_ACCEPTED_FRAME_COUNT)
                {
                    (void)AppController_VisionReady(app);
                }
            }
        }
    }

    __disable_irq();
    overflow = g_k230_rx_overflow;
    g_k230_rx_overflow = 0U;
    __enable_irq();
    if (overflow != 0U)
    {
        k230->overflow_count++;
        g_vision_ready_streak = 0U;
        Main_CancelK230Capture(k230);
        if (AppController_GetState(app) == APP_STATE_RUNNING &&
            Main_IsBalanceDriveTask(AppController_GetTask(app)))
        {
            BalanceDriveRun_ReportVisionFault(balance_run);
            AppController_TaskFailed(app);
        }
    }

    while (K230BallProtocol_TakeCaptureResult(
               &k230->protocol, &capture_result) != 0U)
    {
        if (k230->capture_pending == 0U ||
            capture_result.request_id != k230->capture_request_id)
        {
            k230->stale_capture_result_count++;
        }
        else
        {
            k230->capture_pending = 0U;
            k230->capture_status = capture_result.status;
            k230->captured_center_x_px = capture_result.center_x_px;
            if (capture_result.status == K230_BALL_CAPTURE_STATUS_OK)
            {
                BalanceDriveRun_Init(balance_run);
                BalanceVisionService_Init(balance_vision);
                g_balance_vision_not_before_ms = now_ms + 1U;
                g_vision_ready_streak = 0U;
                k230->capture_deadline_ms =
                    now_ms + K230_CAPTURE_FRESH_FRAME_TIMEOUT_MS;
                k230->capture_waiting_for_frame = 1U;
            }
        }
    }

    if ((k230->capture_pending != 0U ||
         k230->capture_waiting_for_frame != 0U) &&
        (int32_t)(now_ms - k230->capture_deadline_ms) >= 0)
    {
        k230->capture_timeout_count++;
        k230->capture_status = K230_BALL_CAPTURE_STATUS_TIMEOUT;
        Main_CancelK230Capture(k230);
    }
}

static void Main_EnforceSafeStop(const AppController *app,
                                 SafetyManager *safety,
                                 StepperService *stepper,
                                 Task3Controller *task3,
                                 LineDriveService *line_drive,
                                 BalanceDriveRun *balance_run)
{
    AppState state;
    TaskId task;
    SafetyStopReason reason;

    if (!AppController_IsSafeStopRequested(app) ||
        !SafetyManager_IsMotionAllowed(safety))
    {
        return;
    }

    state = AppController_GetState(app);
    task = AppController_GetTask(app);
    if (Main_IsBalanceDriveTask(task) &&
        state == APP_STATE_DONE &&
        StepperService_GetCommandState(stepper) !=
            STEPPER_COMMAND_POSITION_REACHED)
    {
        /* The chassis is already braked at the calibrated stop time. Keep
         * monitoring X42 until the queued return-to-level is confirmed. */
        return;
    }
    reason = state == APP_STATE_DONE
                 ? SAFETY_STOP_TASK_DONE
                 : SAFETY_STOP_FAULT;
    Task3Controller_Cancel(task3);
    if (line_drive->state == LINE_DRIVE_RUNNING)
    {
        LineDriveService_EmergencyStop(line_drive);
        Main_ApplyDriveOutput(line_drive);
    }
    if (Main_IsBalanceDriveTask(task))
    {
        if (state != APP_STATE_DONE &&
            StepperService_GetStatus(stepper) !=
                STEPPER_STATUS_STOPPED)
        {
            BalanceDriveRun_EmergencyStop(
                balance_run, stepper);
        }
    }
    else
    {
        StepperService_EndTracking(stepper);
        (void)StepperService_Stop(stepper);
    }
    SafetyManager_RequestStop(safety, reason, true);
}

static void Main_MakeTaskLine(char line[9], const char *prefix, TaskId task)
{
    uint8_t index = 0U;

    while (prefix[index] != '\0' && index < 5U)
    {
        line[index] = prefix[index];
        ++index;
    }
    line[index++] = ' ';
    line[index++] = 'T';
    line[index++] = (char)('0' + (uint8_t)task);
    line[index] = '\0';
}

static void Main_MakeTimeLine(char line[9], uint32_t elapsed_ms)
{
    uint32_t seconds = elapsed_ms / 1000U;

    if (seconds > 999U)
    {
        seconds = 999U;
    }

    line[0] = 'T';
    line[1] = 'I';
    line[2] = 'M';
    line[3] = 'E';
    line[4] = ' ';
    line[5] = (char)('0' + (seconds / 100U));
    line[6] = (char)('0' + ((seconds / 10U) % 10U));
    line[7] = (char)('0' + (seconds % 10U));
    line[8] = '\0';
}

static void Main_MakeBalanceExtremaLine(
    char line[16],
    int16_t maximum_positive_x_tenths_mm,
    int16_t maximum_negative_x_tenths_mm)
{
    uint16_t positive = maximum_positive_x_tenths_mm > 0 ?
        (uint16_t)maximum_positive_x_tenths_mm : 0U;
    uint16_t negative = maximum_negative_x_tenths_mm < 0 ?
        (uint16_t)(-(int32_t)maximum_negative_x_tenths_mm) : 0U;

    if (positive > 9999U)
    {
        positive = 9999U;
    }
    if (negative > 9999U)
    {
        negative = 9999U;
    }

    line[0] = 'P';
    line[1] = '+';
    line[2] = (char)('0' + (positive / 1000U));
    line[3] = (char)('0' + ((positive / 100U) % 10U));
    line[4] = (char)('0' + ((positive / 10U) % 10U));
    line[5] = '.';
    line[6] = (char)('0' + (positive % 10U));
    line[7] = ' ';
    line[8] = 'N';
    line[9] = '-';
    line[10] = (char)('0' + (negative / 1000U));
    line[11] = (char)('0' + ((negative / 100U) % 10U));
    line[12] = (char)('0' + ((negative / 10U) % 10U));
    line[13] = '.';
    line[14] = (char)('0' + (negative % 10U));
    line[15] = '\0';
}

static void Main_MakeTimingVelocityLine(
    char line[17],
    char prefix,
    bool valid,
    uint32_t elapsed_ms,
    int32_t velocity_tenths_mm_per_s)
{
    uint32_t speed_tenths_mm_per_s;
    uint32_t speed_whole_mm_per_s;

    if (elapsed_ms > 9999U)
    {
        elapsed_ms = 9999U;
    }

    line[0] = prefix;
    if (valid)
    {
        line[1] = (char)('0' + (elapsed_ms / 1000U));
        line[2] =
            (char)('0' + ((elapsed_ms / 100U) % 10U));
        line[3] =
            (char)('0' + ((elapsed_ms / 10U) % 10U));
        line[4] = (char)('0' + (elapsed_ms % 10U));
    }
    else
    {
        line[1] = '-';
        line[2] = '-';
        line[3] = '-';
        line[4] = '-';
    }
    line[5] = 'm';
    line[6] = 's';
    line[7] = ' ';
    line[8] = 'V';

    if (valid)
    {
        if (velocity_tenths_mm_per_s < 0)
        {
            line[9] = '-';
            speed_tenths_mm_per_s =
                (uint32_t)(-(velocity_tenths_mm_per_s + 1)) +
                1U;
        }
        else
        {
            line[9] = '+';
            speed_tenths_mm_per_s =
                (uint32_t)velocity_tenths_mm_per_s;
        }

        if (speed_tenths_mm_per_s > 99999U)
        {
            speed_tenths_mm_per_s = 99999U;
        }
        speed_whole_mm_per_s = speed_tenths_mm_per_s / 10U;
        line[10] =
            (char)('0' + (speed_whole_mm_per_s / 1000U));
        line[11] =
            (char)('0' +
                   ((speed_whole_mm_per_s / 100U) % 10U));
        line[12] =
            (char)('0' +
                   ((speed_whole_mm_per_s / 10U) % 10U));
        line[13] =
            (char)('0' + (speed_whole_mm_per_s % 10U));
        line[14] = '.';
        line[15] =
            (char)('0' + (speed_tenths_mm_per_s % 10U));
    }
    else
    {
        line[9] = '-';
        line[10] = '-';
        line[11] = '-';
        line[12] = '-';
        line[13] = '-';
        line[14] = '-';
        line[15] = '-';
    }
    line[16] = '\0';
}

static bool Main_TaskNeedsTime(TaskId task)
{
    return task != TASK_ID_1_VIDEO;
}

static bool Main_ShowState(
    const AppController *app,
    const Task3Controller *task3,
    const BalanceDriveRun *balance_run,
    const LineDriveService *line_drive)
{
    char title[9];
    char time_line[9];
    char positive_line[17];
    char negative_line[17];
    char extrema_line[16];
    const char *line1 = "";
    const char *line2 = "";
    const char *line3 = "";
    AppState state = AppController_GetState(app);
    TaskId task = AppController_GetTask(app);

    switch (state)
    {
        case APP_STATE_HOME_REQUIRED:
            return OLED_ShowLines("HOME", "RACK LOW", "PRESS OK", "");

        case APP_STATE_MOVING_TO_LEVEL:
            return OLED_ShowLines("LEVEL", "RACK MOVING", "PLEASE WAIT", "");

        case APP_STATE_MENU:
            Main_MakeTaskLine(title, "TASK", task);
            return OLED_ShowLines(title, "SELECT", "PRESS OK", "");

        case APP_STATE_PLACE_BALL:
            Main_MakeTaskLine(title, "PLACE", task);
            return OLED_ShowLines(
                title,
                task == TASK_ID_6_LAP_TARGET
                    ? "BALL TARGET"
                    : "BALL AT O",
                "WAIT VISION",
                "");

        case APP_STATE_CAPTURE_REQUIRED:
            Main_MakeTaskLine(title, "CAP", task);
            return OLED_ShowLines(
                title, "BALL TARGET", "PLEASE WAIT", "");

        case APP_STATE_CAPTURE_READY:
            Main_MakeTaskLine(title, "CAP", task);
            return OLED_ShowLines(
                title, "BALL STABLE", "PRESS CAP", "");

        case APP_STATE_READY:
            Main_MakeTaskLine(title, "READY", task);
            return OLED_ShowLines(
                title,
                Main_TaskNeedsVision(task) ? "VISION OK" : "",
                "PRESS OK",
                "");

        case APP_STATE_RUNNING:
            Main_MakeTaskLine(title, "RUN", task);
            if (task == TASK_ID_3_BALL_MOVE)
            {
                Task3ControllerTimingResult timing;

                Task3Controller_GetTimingResult(task3, &timing);
                Main_MakeTimingVelocityLine(
                    positive_line,
                    'P',
                    timing.positive_reached,
                    timing.positive_from_start_ms,
                    timing.positive_velocity_tenths_mm_per_s);
                Main_MakeTimingVelocityLine(
                    negative_line,
                    'N',
                    timing.negative_reached,
                    timing.negative_from_reverse_ms,
                    timing.negative_velocity_tenths_mm_per_s);
                Main_MakeTimeLine(
                    time_line, AppController_GetElapsedMs(app));
                return OLED_ShowLines(
                    title,
                    positive_line,
                    negative_line,
                    time_line);
            }
            if (Main_TaskNeedsTime(task))
            {
                Main_MakeTimeLine(time_line,
                                  AppController_GetElapsedMs(app));
                line1 = time_line;
            }
            if (Main_IsBalanceDriveTask(task))
            {
                Main_MakeBalanceExtremaLine(
                    extrema_line,
                    balance_run->maximum_positive_x_tenths_mm,
                    balance_run->maximum_negative_x_tenths_mm);
                line1 = extrema_line;
                line2 = time_line;
                line3 = "RESET STOP";
            }
            else
            {
                line2 = "RESET STOP";
            }
            break;

        case APP_STATE_DONE:
            Main_MakeTaskLine(title, "DONE", task);
            if (task == TASK_ID_3_BALL_MOVE)
            {
                Task3ControllerTimingResult timing;

                Task3Controller_GetTimingResult(task3, &timing);
                Main_MakeTimingVelocityLine(
                    positive_line,
                    'P',
                    timing.positive_reached,
                    timing.positive_from_start_ms,
                    timing.positive_velocity_tenths_mm_per_s);
                Main_MakeTimingVelocityLine(
                    negative_line,
                    'N',
                    timing.negative_reached,
                    timing.negative_from_reverse_ms,
                    timing.negative_velocity_tenths_mm_per_s);
                Main_MakeTimeLine(
                    time_line, AppController_GetElapsedMs(app));
                return OLED_ShowLines(
                    title,
                    positive_line,
                    negative_line,
                    time_line);
            }
            if (Main_TaskNeedsTime(task))
            {
                Main_MakeTimeLine(time_line,
                                  AppController_GetElapsedMs(app));
                line1 = time_line;
            }
            line2 = "PRESS OK";
            break;

        case APP_STATE_FAULT:
        default:
            Main_MakeTaskLine(title, "FAULT", task);
            if (Main_TaskNeedsTime(task))
            {
                Main_MakeTimeLine(
                    time_line, AppController_GetElapsedMs(app));
                line3 = time_line;
            }
            if (Main_IsBalanceDriveTask(task))
            {
                if (balance_run->fault ==
                    BALANCE_DRIVE_FAULT_VISION_STALE)
                {
                    line1 = "VISION";
                }
                else if (balance_run->fault ==
                         BALANCE_DRIVE_FAULT_STEPPER)
                {
                    line1 = "X42";
                }
                else if (balance_run->fault ==
                         BALANCE_DRIVE_FAULT_COMMAND_REJECTED)
                {
                    line1 = "COMMAND";
                }
                else if (line_drive->stop_reason ==
                         LINE_DRIVE_STOP_LINE_LOST)
                {
                    line1 = "LINE";
                }
                else if (line_drive->stop_reason ==
                         LINE_DRIVE_STOP_STALL)
                {
                    line1 = "STALL";
                }
                else
                {
                    line1 = "START";
                }
                Main_MakeBalanceExtremaLine(
                    extrema_line,
                    balance_run->maximum_positive_x_tenths_mm,
                    balance_run->maximum_negative_x_tenths_mm);
                line2 = extrema_line;
                break;
            }
            line1 = "PRESS RESET";
            break;
    }

    return OLED_ShowLines(title, line1, line2, line3);
}

int main(void)
{
    AppController app;
    SafetyManager safety;
    StepperService stepper;
    Task3Controller task3;
    BalanceVisionService balance_vision;
    BalanceDriveRun balance_run;
    bool systick_available;
    bool oled_available;
    uint32_t last_poll_ms;
    AppState displayed_state = (AppState)0xFFU;
    TaskId displayed_task = (TaskId)0U;
    uint32_t displayed_refresh_slot = UINT32_MAX;

    SYSCFG_DL_init();
    AppController_Init(&app);
    SafetyManager_Init(&safety);
    StepperService_Init(
        &stepper, Main_WriteStepperFrame, (void *)0);
    X42Protocol_Init(&g_x42_link.protocol);
    K230BallProtocol_Init(&g_k230_link.protocol);
    Task3Controller_Init(&task3);
    BalanceVisionService_Init(&balance_vision);
    BalanceDriveRun_Init(&balance_run);
    LineDriveService_Init(&g_line_drive);
    ButtonDriver_Init();
    g_left_encoder_raw =
        (uint16_t)DL_TimerG_getTimerCount(LEFT_QEI_INST);
    g_right_encoder_state = (uint8_t)(
        Main_ReadPin(
            CAR_GPIO_RIGHT_ENCODER_A_PORT,
            CAR_GPIO_RIGHT_ENCODER_A_PIN) |
        (Main_ReadPin(
             CAR_GPIO_RIGHT_ENCODER_B_PORT,
             CAR_GPIO_RIGHT_ENCODER_B_PIN) << 1U));
    Main_ApplyDriveOutput(&g_line_drive);
    DL_TimerG_startCounter(MOTOR_PWM_INST);
    DL_TimerG_startCounter(LEFT_QEI_INST);
    DL_TimerG_startCounter(CONTROL_TIMER_INST);

    systick_available = DL_SYSTICK_config(CPUCLK_FREQ / 1000U) == 0U;
    oled_available = OLED_Init();
    if (!systick_available || !oled_available)
    {
        SafetyManager_RequestStop(
            &safety, SAFETY_STOP_FAULT, true);
        AppController_TaskFailed(&app);
    }
    NVIC_EnableIRQ(K230_A_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CAR_GPIO_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);

    last_poll_ms = g_system_time_ms;

    while (1)
    {
        uint32_t now_ms = g_system_time_ms;
        uint32_t elapsed_ms = now_ms - last_poll_ms;
        AppState state;
        TaskId task;
        uint32_t refresh_slot;

        Main_ProcessX42Rx(&g_x42_link, &stepper, now_ms);
        Main_ProcessK230(
            &g_k230_link, &app, &balance_vision,
            &balance_run, now_ms);
#if TASK3_USB_TTL_LOG_ENABLED
        Main_ProcessTask3Log(
            &g_task3_log, &app, &task3, now_ms);
#endif
        if (elapsed_ms != 0U)
        {
            uint8_t events;
            uint8_t event;

            last_poll_ms = now_ms;
            AppController_AdvanceTime(&app, elapsed_ms);
            events = ButtonDriver_Update(elapsed_ms);
            event = Main_GetSingleButtonEvent(events);
            if (event != BUTTON_EVENT_NONE)
            {
                AppAction action = AppController_OnKey(
                    &app, Main_ButtonToAppKey(event));
                Main_HandleAction(
                    &app, &safety, &stepper, &g_k230_link, &task3,
                    &g_line_drive,
                    &balance_vision, &balance_run,
                    action, now_ms);
            }
        }

        StepperService_SetIdleQueryRetry(
            &stepper,
            Main_AllowsIdleStepperQueryRetry(
                AppController_GetState(&app)));

        if (g_line_drive_ticks_pending != 0U)
        {
            uint16_t left_now;
            int16_t left_delta;

            __disable_irq();
            --g_line_drive_ticks_pending;
            __enable_irq();
            left_now =
                (uint16_t)DL_TimerG_getTimerCount(LEFT_QEI_INST);
            left_delta = EncoderLogic_Delta16(
                left_now, g_left_encoder_raw);
            g_left_encoder_raw = left_now;
            g_left_encoder_count += left_delta;
            LineDriveService_Update(
                &g_line_drive,
                Main_GrayscaleScan(),
                g_left_encoder_count,
                g_right_encoder_count);
            Main_ApplyDriveOutput(&g_line_drive);

            if (AppController_GetState(&app) == APP_STATE_RUNNING &&
                (AppController_GetTask(&app) ==
                     TASK_ID_2_LAP_STOP ||
                 Main_IsBalanceDriveTask(
                     AppController_GetTask(&app))) &&
                g_line_drive.state != LINE_DRIVE_RUNNING)
            {
                if (AppController_GetTask(&app) ==
                        TASK_ID_2_LAP_STOP &&
                    g_line_drive.state == LINE_DRIVE_STOPPED &&
                    g_line_drive.stop_reason ==
                        LINE_DRIVE_STOP_FINISH_LINE)
                {
                    AppController_TaskFinished(&app);
                }
                else if (Main_IsBalanceDriveTask(
                             AppController_GetTask(&app)) &&
                         g_line_drive.state ==
                             LINE_DRIVE_STOPPED &&
                         g_line_drive.stop_reason ==
                             LINE_DRIVE_STOP_CALIBRATED_TIME &&
                         BalanceDriveRun_Stop(
                             &balance_run, &stepper) != 0U)
                {
                    AppController_TaskFinished(&app);
                }
                else
                {
                    AppController_TaskFailed(&app);
                }
            }
        }

        StepperService_Update(&stepper, now_ms);
        {
            AppState current_state =
                AppController_GetState(&app);
            StepperStatus stepper_status =
                StepperService_GetStatus(&stepper);

            if (stepper_status == STEPPER_STATUS_FAULT &&
                current_state != APP_STATE_FAULT)
            {
                if (Main_IsBalanceDriveTask(
                        AppController_GetTask(&app)))
                {
                    BalanceDriveRun_EmergencyStop(
                        &balance_run, &stepper);
                    LineDriveService_EmergencyStop(
                        &g_line_drive);
                    Main_ApplyDriveOutput(&g_line_drive);
                }
                AppController_TaskFailed(&app);
            }
            else if (current_state ==
                         APP_STATE_MOVING_TO_LEVEL &&
                     stepper_status == STEPPER_STATUS_READY)
            {
                (void)AppController_LevelReached(&app);
            }
        }

        if (AppController_GetState(&app) == APP_STATE_RUNNING &&
            AppController_GetTask(&app) == TASK_ID_3_BALL_MOVE)
        {
            VisionSample ball_sample;
            const VisionSample *ball_sample_pointer = 0;
            int16_t target_angle_centi_degrees = 0;
            Task3ControllerUpdateResult update;
            Task3TuningConfig tuning;

            if (g_k230_link.latest_available != 0U)
            {
                ball_sample.valid =
                    g_k230_link.latest.valid != 0U;
                ball_sample.sequence =
                    g_k230_link.latest.sequence;
                ball_sample.x_tenths_mm =
                    g_k230_link.latest.x_tenths_mm;
                ball_sample.received_at_ms =
                    g_k230_link.latest.received_at_ms;
                ball_sample_pointer = &ball_sample;
            }
            update = Task3Controller_Update(
                &task3,
                now_ms,
                ball_sample_pointer,
                &target_angle_centi_degrees);
            Task3Controller_GetConfig(&task3, &tuning);

            if (update ==
                TASK3_CONTROLLER_UPDATE_COMMAND_DISCRETE)
            {
                StepperService_EndTracking(&stepper);
                if (!StepperService_MoveRelativeAngleCentiDegrees(
                        &stepper,
                        target_angle_centi_degrees,
                        tuning.stepper_speed_rpm,
                        tuning.stepper_acceleration))
                {
                    AppController_TaskFailed(&app);
                }
            }
            else if (update ==
                     TASK3_CONTROLLER_UPDATE_COMMAND_TRACKING)
            {
                if ((!StepperService_IsTracking(&stepper) &&
                     !StepperService_BeginTracking(&stepper)) ||
                    !StepperService_QueueTrackingTargetAngleCentiDegrees(
                        &stepper,
                        target_angle_centi_degrees,
                        tuning.stepper_speed_rpm,
                        tuning.stepper_acceleration))
                {
                    AppController_TaskFailed(&app);
                }
            }
            else if (update ==
                     TASK3_CONTROLLER_UPDATE_DONE)
            {
                StepperService_EndTracking(&stepper);
                AppController_TaskFinished(&app);
            }
        }

        if (AppController_GetState(&app) == APP_STATE_RUNNING &&
            Main_IsBalanceDriveTask(AppController_GetTask(&app)))
        {
            BalanceVisionState vision =
                BalanceVisionService_GetState(
                    &balance_vision,
                    now_ms,
                    BALANCE_VISION_DEFAULT_TIMEOUT_MS);

            BalanceDriveRun_Update(
                &balance_run,
                &stepper,
                &vision,
                LineDriveService_GetForwardAccelerationMmS2(
                    &g_line_drive),
                now_ms);
            if (balance_run.state == BALANCE_DRIVE_FAULT)
            {
                LineDriveService_EmergencyStop(&g_line_drive);
                Main_ApplyDriveOutput(&g_line_drive);
                AppController_TaskFailed(&app);
            }
        }

        // 任务完成或故障时，安全状态必须先于后续界面操作撤销运动许可。
        Main_EnforceSafeStop(
            &app, &safety, &stepper, &task3,
            &g_line_drive, &balance_run);

        state = AppController_GetState(&app);
        task = AppController_GetTask(&app);
        refresh_slot = AppController_GetElapsedMs(&app) /
                       OLED_RUNNING_REFRESH_PERIOD_MS;
        if (oled_available &&
            g_line_drive_ticks_pending == 0U &&
            (state != displayed_state || task != displayed_task ||
             (state == APP_STATE_RUNNING &&
              refresh_slot != displayed_refresh_slot)))
        {
            if (!Main_ShowState(
                    &app, &task3, &balance_run, &g_line_drive))
            {
                oled_available = false;
                SafetyManager_RequestStop(
                    &safety, SAFETY_STOP_FAULT, true);
                AppController_TaskFailed(&app);
            }
            displayed_state = state;
            displayed_task = task;
            displayed_refresh_slot = refresh_slot;
        }

        if (g_line_drive_ticks_pending == 0U)
        {
            __WFI();
        }
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t pins =
        CAR_GPIO_RIGHT_ENCODER_A_PIN |
        CAR_GPIO_RIGHT_ENCODER_B_PIN;
    uint32_t status =
        DL_GPIO_getEnabledInterruptStatus(GPIOA, pins);

    if ((status & pins) != 0U)
    {
        uint8_t next = (uint8_t)(
            Main_ReadPin(
                CAR_GPIO_RIGHT_ENCODER_A_PORT,
                CAR_GPIO_RIGHT_ENCODER_A_PIN) |
            (Main_ReadPin(
                 CAR_GPIO_RIGHT_ENCODER_B_PORT,
                 CAR_GPIO_RIGHT_ENCODER_B_PIN) << 1U));
        uint8_t valid;

        g_right_encoder_count += EncoderLogic_Transition(
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
        if (g_line_drive_ticks_pending < 255U)
        {
            ++g_line_drive_ticks_pending;
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
            uint8_t byte =
                (uint8_t)DL_UART_Main_receiveData(K230_A_UART_INST);

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

void UART1_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(STEPPER_UART_INST) ==
        DL_UART_MAIN_IIDX_RX)
    {
        while (!DL_UART_Main_isRXFIFOEmpty(STEPPER_UART_INST))
        {
            uint8_t next =
                (uint8_t)((g_x42_rx_head + 1U) &
                          (X42_RX_BUFFER_SIZE - 1U));
            uint8_t byte =
                (uint8_t)DL_UART_Main_receiveData(STEPPER_UART_INST);

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
