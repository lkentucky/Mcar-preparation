param([string]$Keil = 'C:\Keil_v5\UV4\UV4.exe')

$ErrorActionPreference = 'Stop'
$projectPath = Join-Path $PSScriptRoot 'project\mdk\rt1064.uvprojx'
$logPath = Join-Path $PSScriptRoot 'project\mdk\build.log'
if (-not (Test-Path -LiteralPath $Keil)) {
    throw "Keil UV4 not found: $Keil"
}

$buildArgs = '-b "' + $projectPath + '" -o "' + $logPath + '"'
$buildProcess = Start-Process -FilePath $Keil -ArgumentList $buildArgs `
    -WindowStyle Hidden -Wait -PassThru
if (Test-Path -LiteralPath $logPath) {
    Get-Content -LiteralPath $logPath
}
if ($buildProcess.ExitCode -gt 1) {
    throw "Keil build failed, exit code $($buildProcess.ExitCode)"
}
