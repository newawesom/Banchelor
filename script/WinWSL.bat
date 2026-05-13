@echo off
REM Set the path of the RflySim tools
if not defined PSP_PATH (
    SET PSP_PATH=E:\PX4PSP
    SET PSP_PATH_LINUX=/mnt/e/PX4PSP
)
cd /d %PSP_PATH%\VcXsrv
tasklist|find /i "vcxsrv.exe" >nul || Xlaunch.exe -run config1.xlaunch

cd /d "%~dp0"
wsl -d RflySim-20.04 

