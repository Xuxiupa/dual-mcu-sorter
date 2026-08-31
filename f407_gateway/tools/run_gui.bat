@echo off
REM ============================================================
REM  双 MCU 物料分拣系统 - 上位机 GUI 启动脚本 (双击即可)
REM  用 pythonw.exe (无终端窗口) 启动, GUI 直接前台展示
REM ============================================================

REM 切到脚本所在目录 (避免双击时路径错)
cd /d "%~dp0"

REM 用 start /B 让 bat 自身立即退出, 只剩 GUI
start "SorterGUI" /B "D:\msys64\mingw64\bin\pythonw.exe" "sorter_gui.py"

REM bat 自身秒退 (避免 CMD 窗口残留)
exit
