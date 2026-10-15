@echo off
cd /d "%~dp0"
echo.
echo  MPU6050 Attitude Demo - local server
echo  Open in Chrome / Edge:  http://localhost:8080
echo  Close this window or press Ctrl+C to stop the server.
echo.
where python >nul 2>nul
if %errorlevel%==0 (
  python -m http.server 8080
) else (
  py -3 -m http.server 8080
)
pause
