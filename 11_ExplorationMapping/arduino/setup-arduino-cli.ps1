<#
.SYNOPSIS
    Puts arduino-cli into <repo>\.tools\arduino-cli (gitignored) and installs the
    AVR core (Uno / Nano / Mega), so ExploreLauncher can upload sensor.ino
    ("Sketch'i yukle" in step 3).

    Downloads about 30 MB (arduino-cli) + 150 MB (arduino:avr core, into
    %LOCALAPPDATA%\Arduino15 - the same place the Arduino IDE uses).

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File 11_ExplorationMapping\arduino\setup-arduino-cli.ps1
#>
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"   # Invoke-WebRequest is very slow with the progress bar
$repo = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$dir = Join-Path $repo ".tools\arduino-cli"
$exe = Join-Path $dir "arduino-cli.exe"

if (-not (Test-Path $exe)) {
    New-Item -ItemType Directory -Force $dir | Out-Null
    $zip = Join-Path $dir "arduino-cli.zip"
    $url = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip"
    Write-Host ">> Downloading arduino-cli: $url"
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    Invoke-WebRequest -Uri $url -OutFile $zip -UseBasicParsing
    Expand-Archive -Path $zip -DestinationPath $dir -Force
    Remove-Item $zip
}
Write-Host ">> arduino-cli: $exe"
& $exe version
if ($LASTEXITCODE -ne 0) { throw "arduino-cli does not run." }

Write-Host ">> Updating the board index..."
& $exe core update-index
if ($LASTEXITCODE -ne 0) { throw "core update-index failed (exit $LASTEXITCODE)." }

Write-Host ">> Installing the AVR core (arduino:avr)..."
& $exe core install arduino:avr
if ($LASTEXITCODE -ne 0) { throw "core install arduino:avr failed (exit $LASTEXITCODE)." }

Write-Host ">> Ready. ExploreLauncher: Gercek robot -> 3. adim -> Sketch'i yukle" -ForegroundColor Green
