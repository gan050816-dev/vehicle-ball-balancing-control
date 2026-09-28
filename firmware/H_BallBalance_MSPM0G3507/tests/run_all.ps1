$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $true

$gcc = 'C:\msys64\ucrt64\bin\gcc.exe'
$projectRoot = Split-Path -Parent $PSScriptRoot
$tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$outputDir = Join-Path $tempRoot (
    'h_ball_balance_tests_' + [guid]::NewGuid().ToString('N'))

if (-not (Test-Path -LiteralPath $gcc)) {
    throw "GCC not found: $gcc"
}

New-Item -ItemType Directory -Path $outputDir | Out-Null

try {
    $projectXml = [xml](Get-Content -Raw -Encoding utf8 -LiteralPath (
        Join-Path $projectRoot '.project'))
    [xml](Get-Content -Raw -Encoding utf8 -LiteralPath (
        Join-Path $projectRoot '.cproject')) | Out-Null
    [xml](Get-Content -Raw -Encoding utf8 -LiteralPath (
        Join-Path $projectRoot '.ccsproject')) | Out-Null
    if ($projectXml.projectDescription.name -ne
        'H_BallBalance_MSPM0G3507') {
        throw 'CCS project name does not match the agreed project name'
    }
    $cproject = Get-Content -Raw -Encoding utf8 -LiteralPath (
        Join-Path $projectRoot '.cproject')
    if (-not $cproject.Contains('${PROJECT_ROOT}/src/comm')) {
        throw 'CCS compiler include path is missing src/comm'
    }
    $mainSource = Get-Content -Raw -Encoding utf8 -LiteralPath (
        Join-Path $projectRoot 'main.c')
    $requiredK230Integration = @(
        '#include "k230_ball_protocol.h"',
        'NVIC_EnableIRQ(K230_A_UART_INST_INT_IRQN)',
        'void UART3_IRQHandler(void)',
        'K230BallProtocol_PushByte',
        'K230BallProtocol_MakeCaptureRequest',
        'K230BallProtocol_MakeUseDefaultOrigin',
        'Main_RequestK230DefaultOrigin',
        'K230BallProtocol_TakeCaptureResult',
        'DL_UART_Main_transmitDataCheck',
        'g_k230_rx_buffer',
        'VisionSample',
        'g_k230_link.latest.received_at_ms',
        'Task3Controller_GetTimingResult',
        'Task3Controller_Update',
        '#define TASK3_USB_TTL_LOG_ENABLED 0',
        '#if TASK3_USB_TTL_LOG_ENABLED',
        '#define TASK3_LOG_PERIOD_MS 100U',
        'Main_ProcessTask3Log(',
        'Main_Task3LogQueueSample(',
        'Task3Controller_GetVisionState(task3, now_ms, &vision)',
        't_ms,x_mm,phase\r\n',
        'Main_ServiceTask3LogTx(log);',
        'log->next_sample_ms = elapsed_ms + TASK3_LOG_PERIOD_MS',
        'log->tx_buffer[log->tx_position]',
        'Task3Controller_GetConfig',
        'StepperService_BeginTracking',
        'StepperService_MoveRelativeAngleCentiDegrees',
        'StepperService_QueueTrackingTargetAngleCentiDegrees',
        'StepperService_EndTracking',
        'Main_MakeTimingVelocityLine',
        'timing.positive_from_start_ms',
        'timing.negative_from_reverse_ms',
        'timing.positive_velocity_tenths_mm_per_s',
        'timing.negative_velocity_tenths_mm_per_s',
        '#include "x42_protocol.h"',
        'NVIC_EnableIRQ(STEPPER_UART_INST_INT_IRQN)',
        'void UART1_IRQHandler(void)',
        'X42Protocol_PushByte',
        'g_x42_rx_buffer',
        '#define X42_TX_TIMEOUT_MS 20U',
        'DL_UART_Main_transmitDataCheck(',
        'stepper_status == STEPPER_STATUS_FAULT',
        'TASK3_CONTROLLER_UPDATE_DONE',
        'AppController_TaskFinished(&app)',
        '#include "line_drive_service.h"',
        '#include "balance_drive_run.h"',
        '#include "balance_vision_service.h"',
        'LineDriveService_Start(',
        'LINE_DRIVE_MODE_REQUIREMENT_4',
        'LINE_DRIVE_MODE_REQUIREMENT_5',
        'static bool Main_IsBalanceDriveTask(TaskId task)',
        'task == TASK_ID_6_LAP_TARGET',
        'static LineDriveMode Main_GetBalanceDriveMode(TaskId task)',
        'BalanceDriveRun_Start(',
        'BalanceDriveRun_Update(',
        'BalanceDriveRun_Stop(',
        'BalanceVisionService_PushFrame(',
        'capture_waiting_for_frame',
        'g_balance_vision_not_before_ms = now_ms + 1U',
        '#define K230_CAPTURE_FRESH_FRAME_TIMEOUT_MS 500U',
        'k230->capture_status = K230_BALL_CAPTURE_STATUS_TIMEOUT',
        'accepted != 0U &&',
        'DL_TimerG_startCounter(MOTOR_PWM_INST)',
        'DL_TimerG_startCounter(LEFT_QEI_INST)',
        'DL_TimerG_startCounter(CONTROL_TIMER_INST)',
        'void GROUP1_IRQHandler(void)',
        'void TIMG6_IRQHandler(void)',
        'static bool Main_TaskNeedsTime(TaskId task)',
        'return task != TASK_ID_1_VIDEO;',
        '#define OLED_RUNNING_REFRESH_PERIOD_MS 2000U',
        'StepperService_SetIdleQueryRetry(',
        'Main_AllowsIdleStepperQueryRetry(',
        'uint32_t displayed_refresh_slot = UINT32_MAX;',
        'refresh_slot = AppController_GetElapsedMs(&app) /',
        'OLED_RUNNING_REFRESH_PERIOD_MS;',
        'refresh_slot != displayed_refresh_slot',
        'static void Main_MakeBalanceExtremaLine(',
        'balance_run->maximum_positive_x_tenths_mm',
        'balance_run->maximum_negative_x_tenths_mm',
        'T3_LOG_UART_INST'
    )
    foreach ($requiredText in $requiredK230Integration) {
        if (-not $mainSource.Contains($requiredText)) {
            throw "Missing K230 integration contract: $requiredText"
        }
    }
    $idleRetryFunction = [regex]::Match(
        $mainSource,
        '(?s)static bool Main_AllowsIdleStepperQueryRetry\(AppState state\)\s*\{(?<body>.*?)\n\}')
    if (-not $idleRetryFunction.Success) {
        throw 'Missing idle X42 query retry state policy'
    }
    $idleRetryBody = $idleRetryFunction.Groups['body'].Value
    foreach ($requiredIdleState in @(
        'APP_STATE_MENU',
        'APP_STATE_PLACE_BALL',
        'APP_STATE_CAPTURE_REQUIRED',
        'APP_STATE_CAPTURE_READY',
        'APP_STATE_READY')) {
        if (-not $idleRetryBody.Contains($requiredIdleState)) {
            throw "Idle X42 retry is missing state: $requiredIdleState"
        }
    }
    if ($idleRetryBody.Contains('APP_STATE_RUNNING') -or
        $idleRetryBody.Contains('APP_STATE_MOVING_TO_LEVEL') -or
        $idleRetryBody.Contains('APP_STATE_DONE')) {
        throw 'Idle X42 retry leaked into a strict or unapproved state'
    }
    $idleRetryCallIndex = $mainSource.IndexOf(
        'StepperService_SetIdleQueryRetry(')
    $lineDriveUpdateIndex = $mainSource.IndexOf(
        'if (g_line_drive_ticks_pending != 0U)')
    $stepperUpdateIndex = $mainSource.IndexOf(
        'StepperService_Update(&stepper, now_ms);')
    if ($idleRetryCallIndex -lt 0 -or
        $idleRetryCallIndex -gt $lineDriveUpdateIndex -or
        $idleRetryCallIndex -gt $stepperUpdateIndex) {
        throw 'Strict X42 policy must be selected before motion updates'
    }
    if ($mainSource -match
        '(?m)^\s*#define\s+TASK3_USB_TTL_LOG_ENABLED\s+(?!0(?:\s|$))') {
        throw 'T3 USB-to-TTL CSV logging must remain compile-time disabled'
    }
    foreach ($removedBluetoothText in @(
        '#include "bluetooth_tuning_protocol.h"',
        '#include "bluetooth_tuning_service.h"',
        'NVIC_EnableIRQ(BLUETOOTH_UART_INST_INT_IRQN)',
        'void UART0_IRQHandler(void)',
        'BluetoothTuningService_Poll',
        'BLUETOOTH_UART_INST')) {
        if ($mainSource.Contains($removedBluetoothText)) {
            throw "Retired Bluetooth path remains in main.c: $removedBluetoothText"
        }
    }
    $balanceTaskStart = $mainSource.IndexOf(
        'static bool Main_IsBalanceDriveTask(TaskId task)')
    $balanceModeStart = $mainSource.IndexOf(
        'static LineDriveMode Main_GetBalanceDriveMode(TaskId task)')
    $cancelCaptureStart = $mainSource.IndexOf(
        'static void Main_CancelK230Capture', $balanceModeStart)
    if ($balanceTaskStart -lt 0 -or
        $balanceModeStart -le $balanceTaskStart -or
        $cancelCaptureStart -le $balanceModeStart) {
        throw 'T6 balance-task helpers are missing or out of order'
    }
    $balanceTaskBody = $mainSource.Substring(
        $balanceTaskStart, $balanceModeStart - $balanceTaskStart)
    foreach ($requiredTask in @(
        'TASK_ID_4_TO_B_BALANCE',
        'TASK_ID_5_LAP_CENTER',
        'TASK_ID_6_LAP_TARGET')) {
        if (-not $balanceTaskBody.Contains($requiredTask)) {
            throw "Balance task helper is missing $requiredTask"
        }
    }
    $balanceModeBody = $mainSource.Substring(
        $balanceModeStart, $cancelCaptureStart - $balanceModeStart)
    if (-not $balanceModeBody.Contains('LINE_DRIVE_MODE_REQUIREMENT_4') -or
        -not $balanceModeBody.Contains('LINE_DRIVE_MODE_REQUIREMENT_5')) {
        throw 'T6 must reuse the requirement-5 lap drive mode'
    }
    $captureResultGate = $mainSource.IndexOf(
        'k230->capture_waiting_for_frame = 1U;')
    $captureFrameGate = $mainSource.IndexOf(
        'accepted != 0U &&')
    $captureReady = $mainSource.IndexOf(
        '(void)AppController_CaptureSucceeded(app);', $captureFrameGate)
    if ($captureResultGate -lt 0 -or
        $captureFrameGate -lt 0 -or
        $captureReady -le $captureFrameGate) {
        throw 'T6 capture result must wait for a fresh accepted dynamic frame'
    }
    $runningStateStart = $mainSource.IndexOf('case APP_STATE_RUNNING:')
    $doneStateStart = $mainSource.IndexOf('case APP_STATE_DONE:')
    $faultStateStart = $mainSource.IndexOf('case APP_STATE_FAULT:')
    if ($runningStateStart -lt 0 -or
        $doneStateStart -le $runningStateStart -or
        $faultStateStart -le $doneStateStart) {
        throw 'OLED task state branches are missing or out of order'
    }
    $runningStateBody = $mainSource.Substring(
        $runningStateStart, $doneStateStart - $runningStateStart)
    $doneStateBody = $mainSource.Substring(
        $doneStateStart, $faultStateStart - $doneStateStart)
    foreach ($stateBody in @($runningStateBody, $doneStateBody)) {
        if (-not $stateBody.Contains('TASK_ID_3_BALL_MOVE') -or
            -not $stateBody.Contains('Main_MakeTimeLine(') -or
            -not $stateBody.Contains(
                'time_line, AppController_GetElapsedMs(app)')) {
            throw 'T3 running and done pages must show total elapsed time'
        }
    }
    $showStateEnd = $mainSource.IndexOf(
        'return OLED_ShowLines(title, line1, line2, line3);',
        $faultStateStart)
    if ($showStateEnd -le $faultStateStart) {
        throw 'OLED state renderer end is missing'
    }
    $faultStateBody = $mainSource.Substring(
        $faultStateStart, $showStateEnd - $faultStateStart)
    if (-not $faultStateBody.Contains('Main_TaskNeedsTime(task)') -or
        -not $faultStateBody.Contains('line3 = time_line;')) {
        throw 'T2-T6 fault pages must retain the final elapsed time'
    }
    $lineDriveSource = Get-Content -Raw -Encoding utf8 -LiteralPath (
        Join-Path $projectRoot 'src\services\line_drive_service.c')
    foreach ($requiredText in @(
        '#define T4_FORWARD_SPEED_MM_S 208',
        '#define T5_T6_FORWARD_SPEED_MM_S 230',
        '#define BALANCE_MIN_INNER_WHEEL_SPEED_MM_S 20',
        '#define BALANCE_MAX_DIFFERENTIAL_MM_S 100',
        '#define BALANCE_DIFFERENTIAL_SLEW_STEP_MM_S 20',
        'LineDriveService_ShapeBalanceDifferential(',
        'service->mode == LINE_DRIVE_MODE_T2 &&',
        'service->elapsed_ticks >= T2_TIMEOUT_TICKS')) {
        if (-not $lineDriveSource.Contains($requiredText)) {
            throw "Missing line-drive contract: $requiredText"
        }
    }
    foreach ($forbiddenText in @(
        'BALANCE_SWITCH_TICKS',
        'BALANCE_AB_SPEED_MM_S',
        'BALANCE_AFTER_B_SPEED_MM_S',
        'REQUIREMENT_4_STOP_TICKS',
        'REQUIREMENT_5_STOP_TICKS')) {
        if ($lineDriveSource.Contains($forbiddenText)) {
            throw "Removed balance-drive contract must not return: $forbiddenText"
        }
    }
    $repositoryRoot = Split-Path -Parent (Split-Path -Parent $projectRoot)
    $k230MainPath = Join-Path $repositoryRoot 'vision\k230\main.py'
    $k230Main = Get-Content -Raw -Encoding utf8 -LiteralPath $k230MainPath
    $requiredK230Output = @(
        'FRAME_HEAD = b"\xA5\x5A"',
        'CMD_BALL_STATE = 0x10',
        'CMD_CAPTURE_RESULT = 0x11',
        'CMD_CAPTURE_REQUEST = 0x90',
        'CMD_USE_DEFAULT_ORIGIN = 0x91',
        'SEND_PERIOD_MS = 20',
        'PRINT_UART_DATA = False',
        'BALL_FLAG_VALID = 0x01',
        'BALL_FLAG_TARGET_CENTER_VALID = 0x08',
        'CAPTURE_SAMPLE_COUNT = 15',
        'CAPTURE_TIMEOUT_MS = 2000',
        'DEFAULT_ORIGIN_OFFSET_CM = 0.2',
        'FIXED_TARGET_ACTUAL_CM = 5.0',
        'POSITIVE_TARGET_RAW_CM = 5.15',
        'FIXED_CALIBRATION_SAMPLE_COUNT = 15',
        'FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM = (',
        '(-0.68, -1.0)',
        '(-2.13, -2.0)',
        '(-3.44, -3.0)',
        '(-4.38, -4.0)',
        '(-6.22, -5.0)',
        '(-9.02, -6.0)',
        'NEGATIVE_ACTUAL_TO_K230_SLOPE = 0.6898',
        'NEGATIVE_ACTUAL_TO_K230_INTERCEPT_MM = -2.8626',
        'NEGATIVE_FORMULA_START_ACTUAL_MM = -10.0',
        'NEGATIVE_FORMULA_START_K230_MM = (',
        'NEGATIVE_TARGET_ACTUAL_MM = -50.0',
        'NEGATIVE_TARGET_CURRENT_OUTPUT_CM = (',
        '_fixed_positive_target_x = 0',
        '_fixed_negative_target_x = 0',
        '_fixed_target_lines_valid = False',
        '_fixed_calibration_valid_frames = 0',
        'def fixed_offset_cm(center_x, origin_x, positive_target_x):',
        'positive_span_px = origin_x - positive_target_x',
        '* FIXED_TARGET_ACTUAL_CM',
        '/ positive_span_px',
        'def current_fixed_negative_output_cm(offset_cm):',
        'if offset_cm >= 0.0:',
        'FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[1:]',
        'FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[-1]',
        'def fixed_input_cm_for_current_negative_output(current_output_cm):',
        'def correct_current_negative_output_cm(current_output_cm):',
        'current_output_mm = current_output_cm * 10.0',
        'current_output_mm >= NEGATIVE_FORMULA_START_K230_MM',
        'def correct_fixed_negative_offset_cm(offset_cm):',
        'correct_current_negative_output_cm(',
        'current_fixed_negative_output_cm(offset_cm)',
        '_rx_length == 0',
        '_target_center_valid = False',
        '_default_origin_center_valid = False',
        'global _default_origin_center_valid',
        'if transmit_valid and not _default_origin_center_valid:',
        '_default_origin_center_x = clamp(',
        '_default_origin_center_y = transmit_center_y',
        '_default_origin_center_valid = True',
        'single_ball_for_fixed_calibration = (',
        'transmit_valid and len(valid_boxes) == 1',
        'and not _fixed_target_lines_valid',
        '_fixed_calibration_valid_frames += 1',
        '>= FIXED_CALIBRATION_SAMPLE_COUNT',
        '_default_origin_center_x - positive_target_x',
        'negative_target_input_cm = (',
        'fixed_input_cm_for_current_negative_output(',
        'NEGATIVE_TARGET_CURRENT_OUTPUT_CM',
        '- negative_target_input_cm',
        '_fixed_positive_target_x = positive_target_x',
        '_fixed_negative_target_x = negative_target_x',
        '_fixed_target_lines_valid = True',
        'not single_ball_for_fixed_calibration',
        '_fixed_calibration_valid_frames = 0',
        'effective_target_center_valid = (',
        '_target_center_valid or _default_origin_center_valid',
        'coordinate_valid = transmit_valid and (',
        '_target_center_valid or _fixed_target_lines_valid',
        'runtime_calibrated = coordinate_valid and cm_per_pixel > 0.0',
        'valid=coordinate_valid',
        'calibrated=runtime_calibrated',
        'target_center_valid=effective_target_center_valid',
        'DEFAULT_ORIGIN_OFFSET_CM',
        '/ cm_per_pixel',
        'origin_center_x = _target_center_x',
        'origin_center_x = _default_origin_center_x',
        'origin_center_valid = _default_origin_center_valid',
        'if origin_center_valid:',
        'ZERO:----,---px REQ:%03d',
        'elif _fixed_target_lines_valid:',
        'correct_fixed_negative_offset_cm(',
        'fixed_offset_cm(',
        '_fixed_positive_target_x,',
        '_fixed_negative_target_x,',
        'not _target_center_valid',
        'ACT:+5cm',
        'ACT:-5cm',
        '(255, 165, 0)',
        '(0, 255, 0)',
        'poll_uart_commands()',
        'update_capture(',
        '52, distance_text',
        '22, state_text',
        'x_tenths_mm',
        'send_ball_state('
    )
    foreach ($requiredText in $requiredK230Output) {
        if (-not $k230Main.Contains($requiredText)) {
            throw "Missing K230 output contract: $requiredText"
        }
    }
    if ($k230Main.Contains('CALIBRATED = False') -or
        $k230Main.Contains('calibrated=CALIBRATED') -or
        $k230Main.Contains('target_center_valid=_target_center_valid')) {
        throw 'K230 fixed origin must set calibrated and target-valid state flags'
    }
    $obsoleteFixedCalibration = @(
        'FIXED_OUTPUT_CORRECTION_CM',
        'NEGATIVE_TARGET_ACTUAL_CM',
        'NEGATIVE_TARGET_RAW_CM',
        'NEGATIVE_CALIBRATION_POINTS_CM',
        'NEGATIVE_RULER_TO_K230_SLOPE',
        'NEGATIVE_RULER_TO_K230_INTERCEPT_CM',
        'NEGATIVE_RULER_ORIGIN_CM',
        'NEGATIVE_TARGET_INPUT_CM',
        'POSITIVE_FIXED_SCALE',
        'NEGATIVE_FIXED_SCALE',
        'correct_fixed_offset_cm'
    )
    foreach ($obsoleteText in $obsoleteFixedCalibration) {
        if ($k230Main.Contains($obsoleteText)) {
            throw "Obsolete moving or superseded fixed calibration remains: $obsoleteText"
        }
    }
    $originOffsetMatch = [regex]::Match(
        $k230Main,
        '(?m)^DEFAULT_ORIGIN_OFFSET_CM = ([0-9]+(?:\.[0-9]+)?)\s*$')
    $fixedTargetActualMatch = [regex]::Match(
        $k230Main,
        '(?m)^FIXED_TARGET_ACTUAL_CM = ([0-9]+(?:\.[0-9]+)?)\s*$')
    $positiveRawMatch = [regex]::Match(
        $k230Main,
        '(?m)^POSITIVE_TARGET_RAW_CM = ([0-9]+(?:\.[0-9]+)?)\s*$')
    $sampleCountMatch = [regex]::Match(
        $k230Main,
        '(?m)^FIXED_CALIBRATION_SAMPLE_COUNT = ([0-9]+)\s*$')
    $negativeSlopeMatch = [regex]::Match(
        $k230Main,
        '(?m)^NEGATIVE_ACTUAL_TO_K230_SLOPE = ([0-9]+(?:\.[0-9]+)?)\s*$')
    $negativeInterceptMatch = [regex]::Match(
        $k230Main,
        '(?m)^NEGATIVE_ACTUAL_TO_K230_INTERCEPT_MM = (-?[0-9]+(?:\.[0-9]+)?)\s*$')
    $formulaStartMatch = [regex]::Match(
        $k230Main,
        '(?m)^NEGATIVE_FORMULA_START_ACTUAL_MM = (-?[0-9]+(?:\.[0-9]+)?)\s*$')
    $negativeTargetMatch = [regex]::Match(
        $k230Main,
        '(?m)^NEGATIVE_TARGET_ACTUAL_MM = (-?[0-9]+(?:\.[0-9]+)?)\s*$')
    if (-not $originOffsetMatch.Success -or
        -not $fixedTargetActualMatch.Success -or
        -not $positiveRawMatch.Success -or
        -not $sampleCountMatch.Success -or
        -not $negativeSlopeMatch.Success -or
        -not $negativeInterceptMatch.Success -or
        -not $formulaStartMatch.Success -or
        -not $negativeTargetMatch.Success) {
        throw 'Missing numeric K230 fixed-origin calibration constants'
    }
    $originOffsetCm = [double]::Parse(
        $originOffsetMatch.Groups[1].Value,
        [System.Globalization.CultureInfo]::InvariantCulture)
    $fixedTargetActualCm = [double]::Parse(
        $fixedTargetActualMatch.Groups[1].Value,
        [System.Globalization.CultureInfo]::InvariantCulture)
    $positiveRawCm = [double]::Parse(
        $positiveRawMatch.Groups[1].Value,
        [System.Globalization.CultureInfo]::InvariantCulture)
    $sampleCount = [int]::Parse($sampleCountMatch.Groups[1].Value)
    $negativeSlope = [double]::Parse(
        $negativeSlopeMatch.Groups[1].Value,
        [System.Globalization.CultureInfo]::InvariantCulture)
    $negativeInterceptMm = [double]::Parse(
        $negativeInterceptMatch.Groups[1].Value,
        [System.Globalization.CultureInfo]::InvariantCulture)
    $formulaStartActualMm = [double]::Parse(
        $formulaStartMatch.Groups[1].Value,
        [System.Globalization.CultureInfo]::InvariantCulture)
    $negativeTargetActualMm = [double]::Parse(
        $negativeTargetMatch.Groups[1].Value,
        [System.Globalization.CultureInfo]::InvariantCulture)
    $yellowRealignmentMm = (0.6 - $originOffsetCm) * 10.0
    if ([math]::Abs($yellowRealignmentMm - 4.0) -gt 0.000001 -or
        [math]::Abs($fixedTargetActualCm - 5.0) -gt 0.000001 -or
        [math]::Abs($positiveRawCm - 5.15) -gt 0.000001 -or
        $sampleCount -ne 15 -or
        [math]::Abs($negativeSlope - 0.6898) -gt 0.000001 -or
        [math]::Abs($negativeInterceptMm + 2.8626) -gt 0.000001 -or
        [math]::Abs($formulaStartActualMm + 10.0) -gt 0.000001 -or
        [math]::Abs($negativeTargetActualMm + 50.0) -gt 0.000001) {
        throw 'K230 must preserve origin/+5 cm and use the measured negative-mm fit'
    }

    $currentNegativeCalibration = @(
        @(0.0, 0.0),
        @(-0.68, -1.0),
        @(-2.13, -2.0),
        @(-3.44, -3.0),
        @(-4.38, -4.0),
        @(-6.22, -5.0),
        @(-9.02, -6.0)
    )
    function Convert-CurrentFixedNegativeOutput([double] $offsetCm) {
        if ($offsetCm -ge 0.0) {
            return $offsetCm
        }
        $previousInputCm = $currentNegativeCalibration[0][0]
        $previousOutputCm = $currentNegativeCalibration[0][1]
        for ($index = 1; $index -lt $currentNegativeCalibration.Count; $index++) {
            $inputCm = $currentNegativeCalibration[$index][0]
            $outputCm = $currentNegativeCalibration[$index][1]
            if ($offsetCm -ge $inputCm) {
                return $previousOutputCm + (
                    ($offsetCm - $previousInputCm) *
                    ($outputCm - $previousOutputCm) /
                    ($inputCm - $previousInputCm))
            }
            $previousInputCm = $inputCm
            $previousOutputCm = $outputCm
        }
        $lastIndex = $currentNegativeCalibration.Count - 1
        $inputCm = $currentNegativeCalibration[$lastIndex][0]
        $outputCm = $currentNegativeCalibration[$lastIndex][1]
        $previousInputCm = $currentNegativeCalibration[$lastIndex - 1][0]
        $previousOutputCm = $currentNegativeCalibration[$lastIndex - 1][1]
        return $outputCm + (
            ($offsetCm - $inputCm) *
            ($outputCm - $previousOutputCm) /
            ($inputCm - $previousInputCm))
    }

    function Convert-CurrentNegativeOutputToFixedInput([double] $currentOutputCm) {
        if ($currentOutputCm -ge 0.0) {
            return $currentOutputCm
        }
        $previousInputCm = $currentNegativeCalibration[0][0]
        $previousOutputCm = $currentNegativeCalibration[0][1]
        for ($index = 1; $index -lt $currentNegativeCalibration.Count; $index++) {
            $inputCm = $currentNegativeCalibration[$index][0]
            $outputCm = $currentNegativeCalibration[$index][1]
            if ($currentOutputCm -ge $outputCm) {
                return $previousInputCm + (
                    ($currentOutputCm - $previousOutputCm) *
                    ($inputCm - $previousInputCm) /
                    ($outputCm - $previousOutputCm))
            }
            $previousInputCm = $inputCm
            $previousOutputCm = $outputCm
        }
        $lastIndex = $currentNegativeCalibration.Count - 1
        $inputCm = $currentNegativeCalibration[$lastIndex][0]
        $outputCm = $currentNegativeCalibration[$lastIndex][1]
        $previousInputCm = $currentNegativeCalibration[$lastIndex - 1][0]
        $previousOutputCm = $currentNegativeCalibration[$lastIndex - 1][1]
        return $inputCm + (
            ($currentOutputCm - $outputCm) *
            ($inputCm - $previousInputCm) /
            ($outputCm - $previousOutputCm))
    }

    $formulaStartK230Mm =
        $negativeSlope * $formulaStartActualMm + $negativeInterceptMm
    function Convert-CurrentOutputToActual([double] $currentOutputCm) {
        if ($currentOutputCm -ge 0.0) {
            return $currentOutputCm
        }
        $currentOutputMm = $currentOutputCm * 10.0
        if ($currentOutputMm -ge $formulaStartK230Mm) {
            return (
                $currentOutputMm * $formulaStartActualMm /
                $formulaStartK230Mm / 10.0)
        }
        return (
            ($currentOutputMm - $negativeInterceptMm) /
            $negativeSlope / 10.0)
    }

    function Convert-FixedNegativeOffset([double] $offsetCm) {
        return Convert-CurrentOutputToActual(
            Convert-CurrentFixedNegativeOutput $offsetCm)
    }

    foreach ($point in $currentNegativeCalibration) {
        $currentOutputCm = Convert-CurrentFixedNegativeOutput $point[0]
        if ([math]::Abs($currentOutputCm - $point[1]) -gt 0.000001) {
            throw "K230 current negative calibration misses anchor $($point[0]) cm"
        }
    }
    for ($actualMm = -10.0; $actualMm -ge -60.0; $actualMm -= 10.0) {
        $currentOutputMm = $negativeSlope * $actualMm + $negativeInterceptMm
        $correctedMm =
            (Convert-CurrentOutputToActual ($currentOutputMm / 10.0)) * 10.0
        if ([math]::Abs($correctedMm - $actualMm) -gt 0.000001) {
            throw "K230 negative-mm fit misses actual position $actualMm mm"
        }
    }
    if ([math]::Abs((Convert-FixedNegativeOffset 1.25) - 1.25) -gt 0.000001 -or
        [math]::Abs((Convert-FixedNegativeOffset 0.0)) -gt 0.000001) {
        throw 'K230 negative calibration must not change zero or positive coordinates'
    }
    $previousCorrectedCm = 0.0
    for ($step = 1; $step -le 902; $step++) {
        $inputCm = -$step / 100.0
        $correctedCm = Convert-FixedNegativeOffset $inputCm
        if ($correctedCm -gt $previousCorrectedCm + 0.000001) {
            throw 'K230 negative calibration must remain monotonic'
        }
        $previousCorrectedCm = $correctedCm
    }
    if ((Convert-FixedNegativeOffset -10.0) -ge -6.0) {
        throw 'K230 negative calibration must extrapolate monotonically past -6 cm'
    }

    $negativeTargetCurrentOutputCm = (
        $negativeSlope * $negativeTargetActualMm + $negativeInterceptMm) / 10.0
    $negativeTargetInputCm = Convert-CurrentNegativeOutputToFixedInput(
        $negativeTargetCurrentOutputCm)
    $originPixel = 640
    $positivePixel = 480
    $spanPixel = $originPixel - $positivePixel
    $negativePixel = [int][math]::Truncate(
        $originPixel -
        $negativeTargetInputCm * $spanPixel / $fixedTargetActualCm)
    $positiveEndpointCm =
        ($originPixel - $positivePixel) * $fixedTargetActualCm / $spanPixel
    $zeroEndpointCm =
        ($originPixel - $originPixel) * $fixedTargetActualCm / $spanPixel
    $negativeInputAtLineCm =
        ($originPixel - $negativePixel) * $fixedTargetActualCm / $spanPixel
    $negativeEndpointCm = Convert-FixedNegativeOffset $negativeInputAtLineCm
    if ([math]::Abs($positiveEndpointCm - 5.0) -gt 0.000001 -or
        [math]::Abs($negativeEndpointCm + 5.0) -gt 0.01 -or
        [math]::Abs($zeroEndpointCm) -gt 0.000001 -or
        $negativePixel -le $originPixel) {
        throw 'K230 locked pixel anchors must map to accurate +5/0/recalibrated -5 cm'
    }
    $dynamicBranchStart = $k230Main.IndexOf(
        '# T6 capture defines its own zero')
    $fixedBranchStart = $k230Main.IndexOf(
        'elif _fixed_target_lines_valid:', $dynamicBranchStart)
    $sendStart = $k230Main.IndexOf('now_ms = time.ticks_ms()', $fixedBranchStart)
    $negativeCorrectionCallIndex = $k230Main.IndexOf(
        'correct_fixed_negative_offset_cm(', $fixedBranchStart)
    $fixedScaleCallIndex = $k230Main.IndexOf(
        'fixed_offset_cm(', $negativeCorrectionCallIndex)
    $effectiveTargetIndex = $k230Main.IndexOf(
        'effective_target_center_valid = (', $fixedBranchStart)
    $runtimeCalibrationIndex = $k230Main.IndexOf(
        'runtime_calibrated = coordinate_valid and cm_per_pixel > 0.0',
        $fixedBranchStart)
    if ($dynamicBranchStart -lt 0 -or
        $fixedBranchStart -lt 0 -or
        $negativeCorrectionCallIndex -lt $fixedBranchStart -or
        $negativeCorrectionCallIndex -gt $sendStart -or
        $fixedScaleCallIndex -lt $negativeCorrectionCallIndex -or
        $fixedScaleCallIndex -gt $sendStart) {
        throw 'K230 fixed pixel mapping must not affect T6 dynamic origin'
    }
    if (
        $effectiveTargetIndex -lt $fixedBranchStart -or
        $effectiveTargetIndex -gt $sendStart -or
        $runtimeCalibrationIndex -lt $effectiveTargetIndex -or
        $runtimeCalibrationIndex -gt $sendStart) {
        throw 'K230 runtime state flags must be derived before sending the ball state'
    }
    $sendStateIndex = $k230Main.IndexOf(
        'send_ball_state(', $sendStart)
    $calibratedArgumentIndex = $k230Main.IndexOf(
        'calibrated=runtime_calibrated', $sendStateIndex)
    $targetValidArgumentIndex = $k230Main.IndexOf(
        'target_center_valid=effective_target_center_valid',
        $sendStateIndex)
    $targetLinesIndex = $k230Main.IndexOf(
        '# Fixed-origin lines and UART coordinates share these locked')
    $yellowMarkerIndex = $k230Main.IndexOf(
        'if origin_center_valid:', $targetLinesIndex)
    if ($sendStateIndex -lt 0 -or
        $calibratedArgumentIndex -lt $sendStateIndex -or
        $calibratedArgumentIndex -gt $targetLinesIndex -or
        $targetValidArgumentIndex -lt $calibratedArgumentIndex -or
        $targetValidArgumentIndex -gt $targetLinesIndex -or
        $targetLinesIndex -lt $sendStateIndex -or
        $yellowMarkerIndex -lt $targetLinesIndex) {
        throw 'K230 target lines must remain display-only after UART output and before the origin marker'
    }
    $targetLineBlock = $k230Main.Substring(
        $targetLinesIndex, $yellowMarkerIndex - $targetLinesIndex)
    if ($targetLineBlock.Contains('cm_per_pixel') -or
        -not $targetLineBlock.Contains('_fixed_positive_target_x') -or
        -not $targetLineBlock.Contains('_fixed_negative_target_x')) {
        throw 'K230 fixed lines must use locked pixel anchors without frame-scale recomputation'
    }
    if ($k230Main.Contains('$X,') -or
        $k230Main.Contains('uart.write("$X,0.0')) {
        throw 'Legacy text K230 output must not coexist with binary protocol'
    }

    $sysConfig = Get-Content -Raw -Encoding utf8 -LiteralPath (
        Join-Path $projectRoot 'H_BallBalance_MSPM0G3507.syscfg')
    $requiredSysConfigText = @(
        '["KEY_SELECT",  "PORTA", "15"]',
        '["KEY_CAPTURE", "PORTB", "27"]',
        '["KEY_CONFIRM", "PORTB", "23"]',
        '["KEY_RESET",   "PORTA", "29"]',
        'oledI2C.peripheral.sdaPin.$assign = "PA28"',
        'oledI2C.peripheral.sclPin.$assign = "PA31"',
        'task3LogUART.$name = "T3_LOG_UART"',
        'task3LogUART.targetBaudRate = 9600',
        'task3LogUART.enableFIFO = false',
        'task3LogUART.peripheral.$assign = "UART0"',
        'task3LogUART.peripheral.txPin.$assign = "PB0"',
        'task3LogUART.peripheral.rxPin.$assign = "PB1"',
        'stepperUART.$name = "STEPPER_UART"',
        'stepperUART.targetBaudRate = 115200',
        'stepperUART.enableFIFO = false',
        'stepperUART.enabledInterrupts = ["RX"]',
        'stepperUART.peripheral.$assign = "UART1"',
        'stepperUART.peripheral.txPin.$assign = "PA8"',
        'stepperUART.peripheral.rxPin.$assign = "PA9"',
        'k230AUART.$name = "K230_A_UART"',
        'k230AUART.targetBaudRate = 115200',
        'k230AUART.enableFIFO = false',
        'k230AUART.enabledInterrupts = ["RX"]',
        'k230AUART.peripheral.$assign = "UART3"',
        'k230AUART.peripheral.txPin.$assign = "PB2"',
        'k230AUART.peripheral.rxPin.$assign = "PB3"',
        'carGPIO.$name = "CAR_GPIO"',
        '["MOTOR_AIN1", "PORTB", "17"]',
        '["MOTOR_AIN2", "PORTB", "19"]',
        '["MOTOR_BIN2", "PORTA", "16"]',
        '["MOTOR_BIN1", "PORTB", "24"]',
        '["GRAY_X0", "PORTB", "20"]',
        '["GRAY_X1", "PORTA", "24"]',
        '["GRAY_X2", "PORTB", "25"]',
        '["GRAY_X3", "PORTB", "26"]',
        '["GRAY_X4", "PORTB", "10"]',
        '["GRAY_X5", "PORTB", "14"]',
        '["GRAY_X6", "PORTA", "17"]',
        '["GRAY_X7", "PORTA", "7"]',
        'carGPIO.associatedPins[12].assignedPin = "14"',
        'carGPIO.associatedPins[13].assignedPin = "25"',
        'motorPWM.peripheral.$assign = "TIMG0"',
        'motorPWM.peripheral.ccp0Pin.$assign = "PA12"',
        'motorPWM.peripheral.ccp1Pin.$assign = "PA13"',
        'leftQEI.peripheral.$assign = "TIMG8"',
        'leftQEI.peripheral.ccp0Pin.$assign = "PA26"',
        'leftQEI.peripheral.ccp1Pin.$assign = "PA27"',
        'controlTimer.timerPeriod = "10 ms"',
        'controlTimer.peripheral.$assign = "TIMG6"'
    )
    foreach ($requiredText in $requiredSysConfigText) {
        if (-not $sysConfig.Contains($requiredText)) {
            throw "Missing SysConfig contract: $requiredText"
        }
    }
    if ($sysConfig -match 'BLUETOOTH|bluetooth|task3LogUART\.enabledInterrupts') {
        throw 'Retired Bluetooth naming or UART0 RX interrupt remains in SysConfig'
    }
    $uartInstanceCount = [regex]::Matches(
        $sysConfig, '\bUART\.addInstance\(\)').Count
    if ($uartInstanceCount -ne 3) {
        throw "Expected exactly 3 configured UART instances, found $uartInstanceCount"
    }
    if ($sysConfig -match 'CORNER|RECENTER') {
        throw 'Obsolete discrete corner/recenter states entered the hardware baseline'
    }
    if ($sysConfig -match
        'peripheral\.\$assign\s*=\s*"UART2"|K230_B_UART|k230BUART') {
        throw 'UART2 must remain free and K230-B must not consume a TI UART'
    }
    Write-Output 'PASS project_contract'

    $tests = @(
        @{
            Name = 'k230_ball_protocol'
            Includes = @('src\comm')
            Sources = @('src\comm\k230_ball_protocol.c')
        },
        @{
            Name = 'x42_protocol'
            Includes = @('src\comm')
            Sources = @('src\comm\x42_protocol.c')
        },
        @{
            Name = 'stepper_service'
            Includes = @('src\services', 'src\comm')
            Sources = @('src\services\stepper_service.c')
        },
        @{
            Name = 'vision_service'
            Includes = @('src\services')
            Sources = @('src\services\vision_service.c')
        },
        @{
            Name = 'balance_vision_service'
            Includes = @('src\services', 'src\comm')
            Sources = @('src\services\balance_vision_service.c')
        },
        @{
            Name = 'ball_pd'
            Includes = @('src\control')
            Sources = @('src\control\ball_pd.c')
        },
        @{
            Name = 'drive_logic'
            Includes = @('src\control')
            Sources = @('src\control\drive_logic.c')
        },
        @{
            Name = 'encoder_logic'
            Includes = @('src\control')
            Sources = @('src\control\encoder_logic.c')
        },
        @{
            Name = 'motor_safety'
            Includes = @('src\control')
            Sources = @('src\control\motor_safety.c')
        },
        @{
            Name = 'speed_pid'
            Includes = @('src\control')
            Sources = @('src\control\speed_pid.c')
        },
        @{
            Name = 'grayscale_logic'
            Includes = @('src\sensors')
            Sources = @('src\sensors\grayscale_logic.c')
        },
        @{
            Name = 'line_drive_service'
            Includes = @('src\services', 'src\control', 'src\sensors')
            Sources = @(
                'src\services\line_drive_service.c',
                'src\control\drive_logic.c',
                'src\control\motor_safety.c',
                'src\control\speed_pid.c',
                'src\sensors\grayscale_logic.c'
            )
        },
        @{
            Name = 'balance_drive_run'
            Includes = @('src\tasks', 'src\services', 'src\control', 'src\comm')
            Sources = @(
                'src\tasks\balance_drive_run.c',
                'src\control\ball_pd.c',
                'src\services\stepper_service.c'
            )
        },
        @{
            Name = 'task3_open_loop'
            Includes = @('src\tasks')
            Sources = @('src\tasks\task3_open_loop.c')
        },
        @{
            Name = 'task3_controller'
            Includes = @('src\tasks', 'src\services')
            Sources = @(
                'src\tasks\task3_controller.c',
                'src\services\vision_service.c'
            )
        },
        @{
            Name = 'gyro_parser'
            Includes = @('src\sensors')
            Sources = @('src\sensors\gyro_parser.c')
        },
        @{
            Name = 'task_menu'
            Includes = @('src\ui')
            Sources = @('src\ui\task_menu.c')
        },
        @{
            Name = 'app_controller'
            Includes = @('src\app', 'src\ui', 'src\safety')
            Sources = @(
                'src\app\app_controller.c',
                'src\ui\task_menu.c',
                'src\safety\safety_manager.c'
            )
        }
    )

    Push-Location $projectRoot
    try {
        foreach ($test in $tests) {
            $arguments = @('-std=c11', '-Wall', '-Wextra', '-Werror')
            foreach ($include in $test.Includes) {
                $arguments += '-I' + $include
            }
            $arguments += 'tests\test_' + $test.Name + '.c'
            $arguments += $test.Sources
            $output = Join-Path $outputDir ('test_' + $test.Name + '.exe')
            $arguments += @('-o', $output)

            & $gcc @arguments
            if ($LASTEXITCODE -ne 0) {
                throw "Compile failed: $($test.Name)"
            }

            & $output
            if ($LASTEXITCODE -ne 0) {
                throw "Test failed: $($test.Name)"
            }
            Write-Output "PASS $($test.Name)"
        }
    }
    finally {
        Pop-Location
    }
}
finally {
    $resolvedOutput = [System.IO.Path]::GetFullPath($outputDir)
    if (-not $resolvedOutput.StartsWith(
            $tempRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to clean path outside temp: $resolvedOutput"
    }
    if (Test-Path -LiteralPath $resolvedOutput) {
        Remove-Item -LiteralPath $resolvedOutput -Recurse -Force
    }
}
