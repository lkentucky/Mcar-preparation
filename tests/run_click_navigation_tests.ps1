param([string]$Gcc = 'C:\Program Files (x86)\cpeditor\mingw64\bin\gcc.exe')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path $PSScriptRoot -Parent
$testOutput=Join-Path $env:TEMP 'mcar-click-navigation-tests'
New-Item -ItemType Directory -Force $testOutput | Out-Null
Push-Location $projectRoot
try {
    $commonFlags=@('-std=c99','-Wall','-Wextra','-Werror','-I','tests/motor_stubs','-I','project/code','-I','libraries/zf_driver','-I','libraries/zf_device')
    $transportExe=Join-Path $testOutput 'click_navigation_test.exe'
    & $Gcc @commonFlags tests/click_navigation_test.c project/code/wifispi.c project/code/pid.c project/code/PID_config.c -lm -o $transportExe
    if ($LASTEXITCODE -ne 0) { throw 'Navigation transport test compile failed' }
    & $transportExe
    if ($LASTEXITCODE -ne 0) { throw 'Navigation transport test failed' }
    $motorExe=Join-Path $testOutput 'click_motor_control_test.exe'
    & $Gcc @commonFlags tests/click_motor_control_test.c project/code/Motor.c project/code/PID.c project/code/PID_config.c project/code/app_control.c project/code/app_navigation.c project/code/navigation_fusion.c project/code/position_control.c -lm -o $motorExe
    if ($LASTEXITCODE -ne 0) { throw 'Remote motor control test compile failed' }
    & $motorExe
    if ($LASTEXITCODE -ne 0) { throw 'Remote motor control test failed' }
} finally {
    Pop-Location
}
