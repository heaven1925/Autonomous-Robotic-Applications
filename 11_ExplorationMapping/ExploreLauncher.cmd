@echo off
rem ExploreLauncher.cmd - double-click to start the 11 Exploration Mapping launcher.
rem Builds the launcher and the four Explore_*.exe (release) if needed, then starts it.
rem   ExploreLauncher.cmd -Shortcut   also puts "11 Exploration Launcher" on the desktop
rem   ExploreLauncher.cmd -NoBuild    start the last build without building
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\run.ps1" launcher -Config release %*
if errorlevel 1 (
    echo.
    echo Launcher could not be started - see the messages above.
    pause
)
