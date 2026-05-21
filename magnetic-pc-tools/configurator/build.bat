@echo off
echo Installing dependencies...
pip install -r requirements.txt
pip install pyinstaller

echo.
echo Building executable...
pyinstaller --noconfirm --onefile --windowed --name "MagneticConfigurator" main.py

echo.
echo Build complete! The executable is located in the "dist" folder.
pause
