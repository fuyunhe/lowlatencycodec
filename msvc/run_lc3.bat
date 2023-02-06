cd /d %~dp0

set EXECMD=Win32\Debug\LC3plus.exe

%EXECMD% -E -frame_ms 5 Test.wav Test.LC3 240000
:: -ept
%EXECMD% -D Test.LC3 Testout.wav

pause
