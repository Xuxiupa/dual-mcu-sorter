' ============================================================
'  双 MCU 物料分拣系统 - 上位机 GUI 静默启动 (双击即可)
'  0 = WshShell.Run 隐藏窗口, 完全不弹任何终端
' ============================================================

Dim shell, py, script
Set shell = CreateObject("WScript.Shell")

py     = "D:\msys64\mingw64\bin\pythonw.exe"
script = """D:\CubeMXproject\stm32f407_modbus_gateway\tools\sorter_gui.py"""

' 第 3 个参数 0 = 隐藏窗口; True = 等待 GUI 关闭
shell.Run """" & py & """ """ & script & """", 0, False

Set shell = Nothing
