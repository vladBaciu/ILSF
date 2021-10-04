@set DUDE=C:\Users\baciu\OneDrive\Documents\ArduinoData\packages\arduino\tools\avrdude\6.3.0-arduino17
@set FF=%DUDE%\bin\avrdude.exe
@set COMPORT=com5
@set FLAGS=-C%DUDE%\etc\avrdude.conf -p ATmega4809 -c jtag2updi -P %COMPORT%
@if %1/==/ goto Usage
mode %COMPORT% baud=12 dtr=on > nul
mode %COMPORT% baud=12 dtr=off > nul
REM timeout 2 > nul
%FF% %FLAGS% %*
@if %ERRORLEVEL% equ 0 goto exit 
@goto wait
:Usage
@echo Usage:%0 -U flash:w:hexfile.hex:i 
:wait
@pause
:exit 
@set FF=
@set FLAGS=
pause