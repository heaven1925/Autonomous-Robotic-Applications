<#
.SYNOPSIS
    Builds a target and runs its .exe with the robot's COM port - in one command.
    Kills any previous robot app first, so the COM port is always free.

.EXAMPLE
    .\run.ps1 1              # Example01 (square)
    .\run.ps1 2              # Example02 (out-and-back + spin)
    .\run.ps1 3              # Example03 (waypoints)
    .\run.ps1 4              # Example04 (waypoints + bumper avoidance)
    .\run.ps1 6              # FollowTheGap (real HC-SR04 via Arduino + Kobuki)
    .\run.ps1 7              # Bug0 (3x HC-SR04 via Arduino + Kobuki)
    .\run.ps1 8              # TangentBug (3x HC-SR04 via Arduino + Kobuki)
    .\run.ps1 8 -Sim         # TangentBug in its built-in simulator (no hardware needed)
    .\run.ps1 8 -SensorTest  # TangentBug sensor check: Arduino only, shows what the planner sees
    .\run.ps1 9              # Mapping: autonomous occupancy-grid mapper (3x HC-SR04 via Arduino + Kobuki)
    .\run.ps1 9 -Sim         # Mapping in its built-in simulator (no hardware needed)
    .\run.ps1 9 -Manual      # Mapping while you drive the robot from the keyboard (also with -Sim)
    .\run.ps1 10             # AStar: plan on maps\assignment.txt and drive the real Kobuki
    .\run.ps1 10 -Sim        # AStar in its built-in simulator (no hardware needed)
    .\run.ps1 10 -PlanOnly   # AStar: only plan, show and save the path (no robot)
    .\run.ps1 10 -Sim -Map 10_AStarPathPlanning\maps\example_20x10.txt -Goal 18,5
    .\run.ps1 10 -Sim --smooth --block 10,0   # anything after the known options goes to the app
    .\run.ps1 sample         # interactive teleop demo (KobukiNativeVS)
    .\run.ps1 stop           # just stop whatever is running
    .\run.ps1 2 -Port COM4   # override auto-detected Kobuki port
    .\run.ps1 6 -Port COM6 -ArduinoPort COM8   # sensor apps (6/7/8/9): override both ports
    .\run.ps1 10 -Port COM6                    # AStar needs only the Kobuki port
    .\run.ps1 1 -NoBuild     # skip build, run last built exe
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Target = "sample",

    [string]$Port,

    [string]$ArduinoPort,

    [ValidateSet("debug", "release")]
    [string]$Config = "debug",

    [switch]$NoBuild,

    [switch]$ListPorts,

    # Run the app's built-in simulator instead of real hardware (TangentBug, Mapping).
    [switch]$Sim,

    # Sensor check with the Arduino only, no Kobuki (TangentBug only).
    [switch]$SensorTest,

    # Drive the robot from the keyboard while the map is built (Mapping only).
    [switch]$Manual,

    # AStar only: map file (0/1 text grid), start / goal cell as column,row.
    [string]$Map,
    [string[]]$Start,
    [string[]]$Goal,

    # AStar only: plan, show and save the path without any robot.
    [switch]$PlanOnly,

    # Everything else (e.g. --smooth, --radius 0.18) is passed to the app.
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Rest
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$preset = "x64-$Config"

# Short aliases -> CMake target names. A plain number N maps to ExampleNN
# (4 -> Example04), so new examples need no change here. Targets that don't
# follow that pattern (e.g. FollowTheGap) get an explicit numeric override.
$numericOverrides = @{
    "6" = "FollowTheGap"
    "7" = "Bug0"
    "8" = "TangentBug"
    "9" = "Mapping"
    "10" = "AStar"
}
$aliases = @{
    "sample"       = "KobukiNativeVS"
    "teleop"       = "KobukiNativeVS"
    "gap"          = "FollowTheGap"
    "followthegap" = "FollowTheGap"
    "bug0"         = "Bug0"
    "tangent"      = "TangentBug"
    "tangentbug"   = "TangentBug"
    "map"          = "Mapping"
    "mapping"      = "Mapping"
    "astar"        = "AStar"
    "path"         = "AStar"
}
if ($numericOverrides.ContainsKey($Target)) { $Target = $numericOverrides[$Target] }
elseif ($Target -match '^\d+$') { $Target = "Example{0:D2}" -f [int]$Target }
elseif ($aliases.ContainsKey($Target.ToLower())) { $Target = $aliases[$Target.ToLower()] }

# Targets that take a second argument: the Arduino sensor COM port.
$sensorApps = @("FollowTheGap", "Bug0", "TangentBug", "Mapping")
$needsArduino = $sensorApps -contains $Target

# Targets with a built-in simulator (run as "<exe> --sim", no COM ports).
$simApps = @("TangentBug", "Mapping", "AStar")
if ($Sim -and -not ($simApps -contains $Target)) {
    throw "-Sim is only supported for: $($simApps -join ', ')"
}
# Targets with an Arduino-only sensor check ("<exe> --sensors <port>").
$sensorTestApps = @("TangentBug")
if ($SensorTest -and -not ($sensorTestApps -contains $Target)) {
    throw "-SensorTest is only supported for: $($sensorTestApps -join ', ')"
}
# Extra arguments passed through to the app.
$appArgs = @()
if ($Manual) {
    if ($Target -ne "Mapping") { throw "-Manual is only supported for: Mapping" }
    $appArgs += "--manual"
}
if (($Map -or $Start -or $Goal -or $PlanOnly) -and $Target -ne "AStar") {
    throw "-Map, -Start, -Goal and -PlanOnly are only supported for: AStar"
}
if ($Map) {
    $mapFile = Resolve-Path -LiteralPath $Map -ErrorAction SilentlyContinue
    if (-not $mapFile) { $mapFile = Resolve-Path -LiteralPath (Join-Path $root $Map) -ErrorAction SilentlyContinue }
    if (-not $mapFile) { throw "Map file not found: $Map" }
    $appArgs += $mapFile.Path
}
if ($Start) { $appArgs += "--start"; $appArgs += ($Start -join ",") }
if ($Goal) { $appArgs += "--goal"; $appArgs += ($Goal -join ",") }
if ($Rest) { $appArgs += $Rest }

# Every robot app we may have started; killed before each run to free the COM port.
$knownApps = @("KobukiNativeVS", "Example*", "FollowTheGap", "Bug0", "TangentBug", "Mapping", "AStar")

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
    # Kobuki's native adapter is FTDI (VID_0403). Arduino Uno clones (and
    # the sensor board this project uses) are almost always CH340
    # (VID_1A86) - keep the two apart so auto-detect never mixes them up
    # when both are plugged in at once (FollowTheGap needs both).
    $pnp = Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%(COM%'" -ErrorAction SilentlyContinue
    $all = @()
    foreach ($d in $pnp) {
        if ($d.Name -match '\((COM\d+)\)') {
            # Capture the port number BEFORE any other -match call below -
            # every -match overwrites $Matches, so this must come first.
            $portNumber = $Matches[1]
            $isFtdi = ($d.Name -match 'FTDI|FT232' -or $d.DeviceID -match 'FTDIBUS|VID_0403')
            # "IsCh340" = the Arduino sensor board: CH340/CH341 clones, or a genuine
            # Arduino (VID_2341 / VID_2A03, or "Arduino" in the device name).
            $isCh340 = ($d.Name -match 'CH340|CH341|Arduino' -or $d.DeviceID -match 'VID_1A86|VID_2341|VID_2A03')
            $all += [pscustomobject]@{
                Port = $portNumber
                Name = $d.Name
                IsFtdi = $isFtdi
                IsCh340 = $isCh340
                IsUsbSerial = ($isFtdi -or $isCh340 -or $d.Name -match 'USB Serial|USB-SERIAL|CP210')
            }
        }
    }
    return $all
}

if ($ListPorts) {
    $ports = Get-RobotPorts
    if (-not $ports) { Write-Host "No COM ports found."; exit 0 }
    $ports | Format-Table Port, Name, IsFtdi, IsCh340 -AutoSize
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

# --- 3. Determine COM port(s) ---
if ($Sim) {
    Write-Host ">> Running: $($exe.FullName) --sim $appArgs`n" -ForegroundColor Cyan
    & $exe.FullName --sim @appArgs
    exit $LASTEXITCODE
}

if ($PlanOnly) {
    Write-Host ">> Running: $($exe.FullName) --plan $appArgs`n" -ForegroundColor Cyan
    & $exe.FullName --plan @appArgs
    exit $LASTEXITCODE
}

if ($SensorTest) {
    if (-not $ArduinoPort) {
        $ch340 = @(Get-RobotPorts | Where-Object IsCh340)
        if ($ch340.Count -lt 1) {
            Write-Host "!! Could not auto-detect the Arduino sensor COM port; use -ArduinoPort COMx." -ForegroundColor Red
            exit 1
        }
        $ArduinoPort = $ch340[0].Port
        Write-Host ">> Auto-detected Arduino port: $ArduinoPort ($($ch340[0].Name))" -ForegroundColor Green
    }
    Write-Host ">> Running: $($exe.FullName) --sensors $ArduinoPort`n" -ForegroundColor Cyan
    & $exe.FullName --sensors $ArduinoPort
    exit $LASTEXITCODE
}

if (-not $Port) {
    $ports = Get-RobotPorts
    $ftdi = @($ports | Where-Object IsFtdi)
    $usb = @($ports | Where-Object IsUsbSerial)
    if ($ftdi.Count -ge 1) {
        $Port = $ftdi[0].Port
        Write-Host ">> Auto-detected robot port: $Port ($($ftdi[0].Name))" -ForegroundColor Green
    }
    elseif ($usb.Count -ge 1) {
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
        if ($simApps -contains $Target) { Write-Host "   No hardware? Try the simulator:       .\run.ps1 $Target -Sim" }
        exit 1
    }
}

if ($needsArduino -and -not $ArduinoPort) {
    $ports = Get-RobotPorts
    $ch340 = @($ports | Where-Object { $_.IsCh340 -and $_.Port -ne $Port })
    if ($ch340.Count -ge 1) {
        $ArduinoPort = $ch340[0].Port
        Write-Host ">> Auto-detected Arduino port: $ArduinoPort ($($ch340[0].Name))" -ForegroundColor Green
    }
    else {
        Write-Host "!! Could not auto-detect the Arduino sensor COM port." -ForegroundColor Red
        if ($ports) { $ports | Format-Table Port, Name, IsFtdi, IsCh340 -AutoSize }
        Write-Host "   Connect the Arduino or specify one, e.g.:  .\run.ps1 $Target -ArduinoPort COM8"
        exit 1
    }
}

# --- 4. Run ---
if ($needsArduino) {
    Write-Host ">> Running: $($exe.FullName) $Port $ArduinoPort $appArgs`n" -ForegroundColor Cyan
    & $exe.FullName $Port $ArduinoPort @appArgs
}
else {
    Write-Host ">> Running: $($exe.FullName) $Port $appArgs`n" -ForegroundColor Cyan
    & $exe.FullName $Port @appArgs
}
exit $LASTEXITCODE
