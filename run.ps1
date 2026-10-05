<#
.SYNOPSIS
    Builds a target and runs its .exe with the robot's COM port - in one command.
    Kills any previous robot app first, so the COM port is always free.

.EXAMPLE
    .\run.ps1 1              # Example01 (square)
    .\run.ps1 2              # Example02 (out-and-back + spin)
    .\run.ps1 sample         # interactive teleop demo (KobukiNativeVS)
    .\run.ps1 stop           # just stop whatever is running
    .\run.ps1 2 -Port COM4   # override auto-detected port
    .\run.ps1 1 -NoBuild     # skip build, run last built exe
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Target = "sample",

    [string]$Port,

    [ValidateSet("debug", "release")]
    [string]$Config = "debug",

    [switch]$NoBuild,

    [switch]$ListPorts
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$preset = "x64-$Config"

# Short aliases -> CMake target names. Add new examples here.
$aliases = @{
    "1"      = "Example01"
    "2"      = "Example02"
    "3"      = "Example03"
    "sample" = "KobukiNativeVS"
    "teleop" = "KobukiNativeVS"
}
if ($aliases.ContainsKey($Target.ToLower())) { $Target = $aliases[$Target.ToLower()] }

# Every robot app we may have started; killed before each run to free the COM port.
$knownApps = @("KobukiNativeVS", "Example01", "Example02", "Example03")

function Stop-RobotApps {
    $stopped = Get-Process -Name $knownApps -ErrorAction SilentlyContinue
    if ($stopped) {
        $stopped | Stop-Process -Force
        Start-Sleep -Milliseconds 400   # give Windows time to release the COM handle
        Write-Host ">> Stopped: $($stopped.Name -join ', ') (COM port released)" -ForegroundColor Yellow
    }
}

if ($Target -ieq "stop") {
    Stop-RobotApps
    Write-Host ">> All robot apps stopped." -ForegroundColor Green
    exit 0
}

function Find-VcVars {
    $vswhereCandidates = @(
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe",
        "${env:ProgramFiles}\Microsoft Visual Studio\Installer\vswhere.exe"
    ) | Where-Object { $_ -and (Test-Path $_) }

    foreach ($vswhere in $vswhereCandidates) {
        $vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($vs) {
            $vcvars = Join-Path $vs "VC\Auxiliary\Build\vcvars64.bat"
            if (Test-Path $vcvars) { return $vcvars }
        }
    }

    $installRoots = @(
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\Community",
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\Professional",
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\Enterprise",
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\BuildTools",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\Community",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\Professional",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\Enterprise",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\BuildTools"
    )

    foreach ($installRoot in $installRoots) {
        if (-not $installRoot) { continue }
        $vcvars = Join-Path $installRoot "VC\Auxiliary\Build\vcvars64.bat"
        if (Test-Path $vcvars) { return $vcvars }
    }

    if (Get-Command cl.exe -ErrorAction SilentlyContinue) {
        return $null
    }

    throw "Could not find vcvars64.bat or cl.exe. Install Visual Studio with Desktop development with C++, or run this from a Developer PowerShell."
}

function Get-RobotPorts {
    # Prefer FTDI / USB serial adapters (Kobuki uses an FTDI chip).
    $pnp = Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%(COM%'" -ErrorAction SilentlyContinue
    $all = @()
    foreach ($d in $pnp) {
        if ($d.Name -match '\((COM\d+)\)') {
            $all += [pscustomobject]@{
                Port = $Matches[1]
                Name = $d.Name
                IsUsbSerial = ($d.Name -match 'FTDI|FT232|USB Serial|USB-SERIAL|CP210|CH340' -or $d.DeviceID -match 'FTDIBUS|VID_0403')
            }
        }
    }
    return $all
}

if ($ListPorts) {
    $ports = Get-RobotPorts
    if (-not $ports) { Write-Host "No COM ports found."; exit 0 }
    $ports | Format-Table Port, Name, IsUsbSerial -AutoSize
    exit 0
}

function Find-TargetExe {
    $searchDirs = @(
        (Join-Path $root "out\build\$preset"),
        (Join-Path $root "cmake-build-$Config"),
        (Join-Path $root "cmake-build-debug"),
        (Join-Path $root "cmake-build-release")
    ) | Where-Object { $_ -and (Test-Path $_) } | Select-Object -Unique

    foreach ($searchDir in $searchDirs) {
        $exe = Get-ChildItem $searchDir -Recurse -Filter "$Target.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($exe) { return $exe }
    }

    return $null
}

# --- 1. Free the COM port, then build ---
Stop-RobotApps

if (-not $NoBuild) {
    try {
        $vcvars = Find-VcVars
    }
    catch {
        $exe = Find-TargetExe
        if ($exe) {
            Write-Host "!! Build tools were not found; using existing executable instead:" -ForegroundColor Yellow
            Write-Host "   $($exe.FullName)" -ForegroundColor Yellow
            $NoBuild = $true
        }
        else {
            throw
        }
    }

    if (-not $NoBuild) {
        Write-Host ">> Building target '$Target' ($preset)..." -ForegroundColor Cyan
        if ($vcvars) {
            cmd /c "`"$vcvars`" >nul && cd /d `"$root`" && cmake --preset $preset && cmake --build --preset $preset --target $Target"
        }
        else {
            cmake --preset $preset
            if ($LASTEXITCODE -ne 0) { throw "Configure failed (exit $LASTEXITCODE)." }
            cmake --build --preset $preset --target $Target
        }
        if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)." }
    }
}

# --- 2. Locate the executable ---
$exe = Find-TargetExe
if (-not $exe) { throw "$Target.exe not found. Build it in CLion first, or install Visual Studio Build Tools so this script can build it." }

# --- 3. Determine COM port ---
if (-not $Port) {
    $ports = Get-RobotPorts
    $usb = @($ports | Where-Object IsUsbSerial)
    if ($usb.Count -ge 1) {
        $Port = $usb[0].Port
        Write-Host ">> Auto-detected robot port: $Port ($($usb[0].Name))" -ForegroundColor Green
        if ($usb.Count -gt 1) { Write-Host "   (multiple USB serial ports found; use -Port to override)" -ForegroundColor Yellow }
    }
    elseif ($ports.Count -eq 1) {
        $Port = $ports[0].Port
        Write-Host ">> Using the only COM port present: $Port ($($ports[0].Name))" -ForegroundColor Yellow
    }
    else {
        Write-Host "!! Could not auto-detect the robot COM port." -ForegroundColor Red
        if ($ports) { $ports | Format-Table Port, Name -AutoSize }
        Write-Host "   Connect the robot or specify one, e.g.:  .\run.ps1 $Target -Port COM4"
        exit 1
    }
}

# --- 4. Run ---
Write-Host ">> Running: $($exe.FullName) $Port`n" -ForegroundColor Cyan
& $exe.FullName $Port
exit $LASTEXITCODE
