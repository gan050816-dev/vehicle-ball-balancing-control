@echo off
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0"
title K230 RTSP Recorder

set "RTSP_URL=rtsp://192.168.1.200:8554/video"

set "RECORDER=%~dp0record_rtsp.py"
if not exist "!RECORDER!" (
    set "RECORDER="
    for %%F in ("%~dp0record_rtsp*.py") do (
        if not defined RECORDER if exist "%%~fF" set "RECORDER=%%~fF"
    )
)

if not defined RECORDER (
    echo [ERROR] Cannot find record_rtsp.py or record_rtsp*.py
    echo [ERROR] Put this BAT file in the same folder as the Python recorder.
    pause
    exit /b 2
)

where python >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python was not found in PATH.
    pause
    exit /b 3
)

set /a "TASK_INDEX=2"
:find_output_file
if exist "%~dp0task!TASK_INDEX!.mp4" goto next_task_index
if exist "%~dp0task!TASK_INDEX!_parts\" goto next_task_index
set "OUTPUT_FILE=%~dp0task!TASK_INDEX!.mp4"
goto output_file_found

:next_task_index
set /a "TASK_INDEX+=1"
goto find_output_file

:output_file_found

echo ========================================
echo   K230 RTSP Recorder
echo ========================================
echo   RTSP:    !RTSP_URL!
echo   Helper:  !RECORDER!
echo   Output:  !OUTPUT_FILE!
echo.
echo   Press any key to start recording ...
pause >nul

echo.
echo [START] Recording ...
echo [TIP] Press Q to stop
echo.

python "!RECORDER!" record --url "!RTSP_URL!" --output "!OUTPUT_FILE!"
set "RECORDER_EXIT=!ERRORLEVEL!"

if not "!RECORDER_EXIT!"=="0" (
    echo.
    echo [ERROR] Recorder exited with code !RECORDER_EXIT!.
    pause
    exit /b !RECORDER_EXIT!
)

echo.
echo [DONE] Saved: !OUTPUT_FILE!
pause
exit /b 0
