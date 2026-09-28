$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $true
$Gcc = 'C:\msys64\ucrt64\bin\gcc.exe'
$Root = Split-Path -Parent $PSScriptRoot
$TempRoot = Join-Path ([System.IO.Path]::GetTempPath()) (
    'task2_tests_' + [guid]::NewGuid().ToString('N'))

New-Item -ItemType Directory -Path $TempRoot | Out-Null
Push-Location $Root
try {
    & (Join-Path $PSScriptRoot 'test_minimal_line_config.ps1')
    & (Join-Path $PSScriptRoot 'test_main_integration.ps1')
    & (Join-Path $PSScriptRoot 'test_cproject_contract.ps1')

    $tests = @(
        @{ Name='grayscale_logic'; Includes=@('src\sensors');
           Sources=@('src\sensors\grayscale_logic.c') },
        @{ Name='drive_logic'; Includes=@('src\control');
           Sources=@('src\control\drive_logic.c') },
        @{ Name='encoder_logic'; Includes=@('src\control');
           Sources=@('src\control\encoder_logic.c') },
        @{ Name='speed_pid'; Includes=@('src\control');
           Sources=@('src\control\speed_pid.c') },
        @{ Name='motor_safety'; Includes=@('src\control');
           Sources=@('src\control\motor_safety.c') },
        @{ Name='ball_pd'; Includes=@('src\control');
           Sources=@('src\control\ball_pd.c') },
        @{ Name='k230_ball_protocol'; Includes=@('src\comm');
           Sources=@('src\comm\k230_ball_protocol.c') },
        @{ Name='x42_protocol'; Includes=@('src\comm');
           Sources=@('src\comm\x42_protocol.c') },
        @{ Name='stepper_service';
           Includes=@('src\comm', 'src\services');
           Sources=@('src\services\stepper_service.c') },
        @{ Name='vision_service';
           Includes=@('src\comm', 'src\services');
           Sources=@('src\services\vision_service.c') },
        @{ Name='task4_run';
           Includes=@(
               'src\comm', 'src\control',
               'src\services', 'src\tasks');
           Sources=@(
               'src\tasks\task4_run.c',
               'src\control\ball_pd.c',
               'src\services\stepper_service.c') },
        @{ Name='task_menu'; Includes=@('src\ui');
           Sources=@('src\ui\task_menu.c') },
        @{ Name='run_status'; Includes=@('src\ui');
           Sources=@('src\ui\run_status.c') }
    )
    foreach ($test in $tests) {
        $exe = Join-Path $TempRoot ('test_' + $test.Name + '.exe')
        $arguments = @('-std=c11', '-Wall', '-Wextra', '-Werror')
        foreach ($include in $test.Includes) {
            $arguments += '-I' + $include
        }
        $arguments += 'tests\test_' + $test.Name + '.c'
        $arguments += $test.Sources
        $arguments += @('-o', $exe)
        & $Gcc @arguments
        if ($LASTEXITCODE -ne 0) {
            throw "Compile failed: $($test.Name)"
        }
        & $exe
        if ($LASTEXITCODE -ne 0) {
            throw "Test failed: $($test.Name)"
        }
        Write-Output "PASS $($test.Name)"
    }
}
finally {
    Pop-Location
    if (Test-Path -LiteralPath $TempRoot) {
        Remove-Item -LiteralPath $TempRoot -Recurse -Force
    }
}
