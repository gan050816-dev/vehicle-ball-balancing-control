$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$syscfg = Get-Content -Raw -Encoding utf8 -LiteralPath (
    Join-Path $root 'task2.syscfg')

foreach ($required in @(
    'MOTOR_AIN1', 'MOTOR_AIN2', 'MOTOR_BIN1', 'MOTOR_BIN2',
    'KEY1', 'KEY2', 'KEY3', 'KEY4',
    '["GRAY_X0", "PORTB", "20"]',
    '["GRAY_X1", "PORTA", "24"]',
    '["GRAY_X2", "PORTB", "25"]',
    '["GRAY_X3", "PORTB", "26"]',
    '["GRAY_X4", "PORTB", "10"]',
    '["GRAY_X5", "PORTB", "14"]',
    '["GRAY_X6", "PORTA", "17"]',
    '["GRAY_X7", "PORTA", "7"]',
    'RIGHT_ENCODER_A', 'RIGHT_ENCODER_B',
    'MOTOR_PWM', 'LEFT_QEI', 'CONTROL_TIMER', 'OLED_I2C',
    'x42UART.$name = "X42_UART"',
    'x42UART.targetBaudRate = 115200',
    'x42UART.enabledInterrupts = ["RX"]',
    'x42UART.peripheral.$assign = "UART1"',
    'x42UART.peripheral.txPin.$assign = "PA8"',
    'x42UART.peripheral.rxPin.$assign = "PA9"',
    'k230AUART.$name = "K230_A_UART"',
    'k230AUART.targetBaudRate = 115200',
    'k230AUART.enabledInterrupts = ["RX"]',
    'k230AUART.peripheral.$assign = "UART3"',
    'k230AUART.peripheral.txPin.$assign = "PB2"',
    'k230AUART.peripheral.rxPin.$assign = "PB3"'
)) {
    if ($syscfg -notmatch [regex]::Escape($required)) {
        throw "Missing SysConfig item: $required"
    }
}
foreach ($forbidden in @(
    'YAW', 'F32C', 'GYRO_UART', 'UART0', 'UART2',
    'LAP_BUZZER', 'LAP_LED',
    'GRAY_AD0', 'GRAY_AD1', 'GRAY_AD2', 'GRAY_OUT'
)) {
    if ($syscfg -match [regex]::Escape($forbidden)) {
        throw "Forbidden SysConfig item remains: $forbidden"
    }
}
Write-Output 'PASS minimal_line_config'
