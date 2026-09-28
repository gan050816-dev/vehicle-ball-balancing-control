$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$projectPath = Join-Path $root '.cproject'
[xml]$project = Get-Content -Raw -Encoding utf8 -LiteralPath $projectPath
if ($null -eq $project.cproject) {
    throw '.cproject XML root is invalid'
}
$text = Get-Content -Raw -Encoding utf8 -LiteralPath $projectPath
foreach ($path in @(
    'src/control', 'src/comm', 'src/sensors',
    'src/services', 'src/tasks', 'src/ui')) {
    if ($text -notmatch [regex]::Escape('${PROJECT_ROOT}/' + $path)) {
        throw "Missing compiler include path: $path"
    }
}
foreach ($path in @('src/app', 'src/drivers')) {
    if ($text -match [regex]::Escape('${PROJECT_ROOT}/' + $path)) {
        throw "Forbidden compiler include path remains: $path"
    }
}
$name = Get-Content -Raw -Encoding utf8 -LiteralPath (
    Join-Path $root '.project')
if ($name -notmatch '<name>task2</name>') {
    throw 'CCS project name is not task2'
}
Write-Output 'PASS cproject_contract'
