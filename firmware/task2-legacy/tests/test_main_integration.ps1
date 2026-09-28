$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$main = Get-Content -Raw -Encoding utf8 -LiteralPath (Join-Path $root 'main.c')

foreach ($required in @(
    '#define T2_FORWARD_ACCEL_MM_S2 800U',
    '#define T4_FORWARD_ACCEL_MM_S2 300U',
    '#define T2_FORWARD_JERK_MM_S3 2000U',
    '#define T4_FORWARD_JERK_MM_S3 800U',
    '#define T2_SPEED_MM_S 350',
    '#define T4_AB_SPEED_MM_S 208',
    '#define T4_AFTER_B_SPEED_MM_S 244',
    '#define T4_SWITCH_TICKS 800U',
    '#define T2_FINISH_ARM_TICKS 900U',
    '#define T4_STOP_TICKS 3100U',
    '#define FINISH_ACTIVE_SENSOR_COUNT 4U',
    '#define FINISH_CONFIRM_TICKS 3U',
    '#define LOST_LINE_SPEED_MM_S 120',
    '#define LINE_CENTER_DEADBAND 20',
    '#define T2_HIGH_GAIN_ERROR 60',
    '#define T2_LOW_KP_NUMERATOR 2',
    '#define T2_HIGH_KP_NUMERATOR 4',
    '#define T2_KD_NUMERATOR 1',
    '#define T2_PD_DENOMINATOR 2',
    '#define T2_LOW_DERIVATIVE_LIMIT 20',
    '#define T2_HIGH_DERIVATIVE_LIMIT 40',
    '#define T4_LINE_KP_NUMERATOR 3',
    '#define T4_LINE_KD_NUMERATOR 1',
    '#define T4_PD_DENOMINATOR 4',
    '#define T4_DERIVATIVE_LIMIT 20',
    '#define T2_LOST_LINE_DIFFERENTIAL_MM_S 270',
    '#define T4_LOST_LINE_DIFFERENTIAL_MM_S 120',
    '#define STEERING_REVERSAL_CONFIRM_TICKS 2U',
    '#define PWM_RAMP_STEP_TICKS 80U',
    '#define LOST_LINE_STOP_TICKS 100U',
    'Grayscale_ActiveCount(line_mask)',
    'TaskMenu_GetMode(&g_menu) == TASK_MODE_T2_LAP18 &&',
    'Grayscale_ActiveMask(raw, 0U)',
    'TaskMenu_SelectNext',
    'TaskMenu_Confirm',
    'TryStartConfirmedTask',
    'ResetToHome',
    'if (key1 != 0U &&',
    'if (key2 != 0U)',
    'if (key3 != 0U)',
    'TaskMenu_IsConfirmed(&g_menu) != 0U',
    'g_vision_state.valid == 0U',
    'g_task4_vision_not_before_ms',
    'g_system_time_ms + CONTROL_TICK_MS',
    'g_vision.last_frame_ms -',
    'g_forward_speed = DriveMotionProfile_Update',
    'ProfileMaximumAcceleration()',
    'ProfileMaximumJerk()',
    '(int16_t)g_forward_profile.acceleration_mm_s2',
    'differential = FilterSteeringReversal',
    'DriveLogic_MixDifferential',
    'OLED_ShowLineStatus',
    'StallMonitor_Update',
    'MotorDirectionGuard_Apply',
    'StepperService_BeginLeveling',
    'RequestK230DefaultOrigin',
    'K230BallProtocol_MakeUseDefaultOrigin',
    'VisionService_PushFrame',
    'Task4Run_Start',
    'Task4Run_Update',
    'Task4Run_Stop',
    'g_display.task4_fault_code = (uint8_t)g_task4.fault;',
    'g_display.task4_vision_fault_diagnostic_code =',
    'g_display.task4_vision_fault_age_ms =',
    'g_display.task4_vision_fault_checksum_errors =',
    'g_display.task4_vision_fault_rx_overflows =',
    'g_task4.fault == TASK4_FAULT_VISION_STALE',
    'VisionService_GetDiagnosticCode(&g_vision)',
    'g_vision_state.age_ms;',
    'g_k230_protocol.checksum_error_count -',
    'g_task4_k230_checksum_start;',
    'g_k230_link.overflow_count -',
    'g_task4_k230_overflow_start;',
    'StepperService_GetRelativeAngleCdeg',
    'X42_RX_BUFFER_SIZE 64U',
    'K230_RX_BUFFER_SIZE 128U',
    'NVIC_EnableIRQ(X42_UART_INST_INT_IRQN)',
    'NVIC_EnableIRQ(K230_A_UART_INST_INT_IRQN)',
    'void UART1_IRQHandler(void)',
    'void UART3_IRQHandler(void)',
    'RUN_DISPLAY_HOME',
    'RUN_DISPLAY_LEVEL',
    'RUN_DISPLAY_WAIT',
    'RUN_DISPLAY_STOPPED',
    'RUN_STOP_STALL',
    'RUN_STOP_LINE',
    'RUN_STOP_FINISH',
    'RUN_STOP_TIME',
    'StopLineFollow(RUN_STOP_STALL)',
    'StopLineFollow(RUN_STOP_LINE)',
    'StopLineFollow(RUN_STOP_FINISH)',
    'StopLineFollow(RUN_STOP_TIME)',
    'RUN_DISPLAY_FAULT'
)) {
    if ($main -notmatch [regex]::Escape($required)) {
        throw "Missing main contract: $required"
    }
}
foreach ($forbidden in @(
    'PURE_LINE_FOLLOW_MODE', 'LINE_CONTROL_', 'CORNER', 'RECENTER',
    'MAX_TURN', 'Yaw', 'yaw', 'Gyro', 'gyro',
    'Aim', 'Route_',
    'K230BallProtocol_MakeCaptureRequest',
    'TaskMenu_GetMaximumAcceleration',
    'M28_',
    'TASK_MODE_LOOP28',
    '"L28"',
    'DriveLogic_SlewSpeedTarget',
    '#define LINE_KP ',
    '#define LINE_KD ',
    '#define LINE_DERIVATIVE_LIMIT ',
    '#define LOST_LINE_DIFFERENTIAL_MM_S ',
    '#define PWM_RAMP_STEP_TICKS 40U',
    'CAR_GPIO_GRAY_AD0_',
    'CAR_GPIO_GRAY_AD1_',
    'CAR_GPIO_GRAY_AD2_',
    'CAR_GPIO_GRAY_OUT_',
    'Grayscale_ActiveMask(raw, 1U)',
    'delay_cycles(32U)',
    'g_target_left = DriveLogic_SlewSpeedTarget',
    'g_target_right = DriveLogic_SlewSpeedTarget',
    'StopLineFollow();',
    'rack_tenths_mm',
    'StepperService_GetMeasuredTenthsMm',
    'StepperService_GetCommandedTenthsMm'
)) {
    if ($main -match [regex]::Escape($forbidden)) {
        throw "Forbidden main dependency remains: $forbidden"
    }
}

foreach ($mapping in @(
    @{ Name = 'KEY1 leveling'; Pattern =
        'if \(key1 != 0U &&\s+StepperService_BeginLeveling' },
    @{ Name = 'KEY2 selection'; Pattern =
        'if \(key2 != 0U\)\s+\{\s+TaskMenu_SelectNext' },
    @{ Name = 'KEY3 confirm'; Pattern =
        'if \(key3 != 0U\)\s+\{\s+TaskMenu_Confirm' },
    @{ Name = 'KEY4 reset'; Pattern =
        'if \(key4 != 0U\)\s+\{\s+ResetToHome' }
)) {
    if ($main -notmatch $mapping.Pattern) {
        throw "Incorrect key mapping: $($mapping.Name)"
    }
}

function Get-Mode18Differential(
    [int]$error,
    [int]$previousError
) {
    if ([Math]::Abs($error) -le 20) {
        $error = 0
    }
    if ([Math]::Abs($error) -ge 60) {
        $kpNumerator = 4
        $derivativeLimit = 40
    } else {
        $kpNumerator = 2
        $derivativeLimit = 20
    }
    $derivative = [Math]::Max(
        -$derivativeLimit,
        [Math]::Min($derivativeLimit, $error - $previousError)
    )
    return [int](($kpNumerator * $error + $derivative) / 2)
}

foreach ($case in @(
    @{ Error = 20; Expected = 0 },
    @{ Error = 45; Expected = 55 },
    @{ Error = 70; Expected = 160 },
    @{ Error = 120; Expected = 260 },
    @{ Error = 144; Expected = 308 }
)) {
    $actual = Get-Mode18Differential $case.Error 0
    if ($actual -ne $case.Expected) {
        throw "M18 segmented PD drifted for error $($case.Error): $actual"
    }
}

function Get-T4Differential(
    [int]$error,
    [int]$previousError
) {
    if ([Math]::Abs($error) -le 20) {
        $error = 0
    }
    $derivative = [Math]::Max(
        -20,
        [Math]::Min(20, $error - $previousError)
    )
    return [int][Math]::Truncate(
        (3 * $error + $derivative) / 4.0
    )
}

foreach ($case in @(
    @{ Error = 20; Previous = 0; Expected = 0 },
    @{ Error = 70; Previous = 0; Expected = 57 },
    @{ Error = 70; Previous = 70; Expected = 52 },
    @{ Error = 144; Previous = 70; Expected = 113 },
    @{ Error = 144; Previous = 144; Expected = 108 }
)) {
    $actual = Get-T4Differential $case.Error $case.Previous
    if ($actual -ne $case.Expected) {
        throw "T4 low-speed PD drifted for error $($case.Error): $actual"
    }
}

$appliedSign = 0
$pendingSign = 0
$confirmationTicks = 0
function Invoke-SteeringReversalFilter(
    [int]$requested,
    [bool]$immediate
) {
    $requestedSign = [Math]::Sign($requested)
    if ($requestedSign -eq 0) {
        $script:pendingSign = 0
        $script:confirmationTicks = 0
        return 0
    }
    if ($immediate -or $script:appliedSign -eq 0 -or
        $requestedSign -eq $script:appliedSign) {
        $script:appliedSign = $requestedSign
        $script:pendingSign = 0
        $script:confirmationTicks = 0
        return $requested
    }
    if ($script:pendingSign -ne $requestedSign) {
        $script:pendingSign = $requestedSign
        $script:confirmationTicks = 1
        return 0
    }
    if ($script:confirmationTicks -lt 2) {
        ++$script:confirmationTicks
    }
    if ($script:confirmationTicks -lt 2) {
        return 0
    }
    $script:appliedSign = $requestedSign
    $script:pendingSign = 0
    $script:confirmationTicks = 0
    return $requested
}

if ((Invoke-SteeringReversalFilter 260 $false) -ne 260) {
    throw 'Same-direction steering must pass immediately'
}
if ((Invoke-SteeringReversalFilter -160 $false) -ne 0) {
    throw 'First reversed steering tick must be suppressed'
}
if ((Invoke-SteeringReversalFilter -180 $false) -ne -180) {
    throw 'Confirmed reversed steering must pass on the second tick'
}
if ((Invoke-SteeringReversalFilter 270 $true) -ne 270) {
    throw 'Immediate lost-line correction must bypass reversal filtering'
}

Write-Output 'PASS main_integration'
