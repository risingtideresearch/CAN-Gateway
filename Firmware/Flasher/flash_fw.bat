@echo off
:: --- CONFIGURATION SETTINGS ---
set COM_PORT=COM3
set BAUD_RATE=921600

:: --- BINARY FILE PATHS AND HEX ADDRESSES ---
set BOOTLOADER_ADDR=0x0
set BOOTLOADER_FILE=Gateway_MCP2518.ino.bootloader.bin

set PARTITIONS_ADDR=0x8000
set PARTITIONS_FILE=Gateway_MCP2518.ino.partitions.bin

set BOOT_APP_ADDR=0xe000
set BOOT_APP_FILE=boot_app0.bin

set APP_ADDR=0x10000
set APP_FILE=Gateway_MCP2518.ino.bin
:: ------------------------------

echo ===================================================
echo   Flashing ESP32 on %COM_PORT% at %BAUD_RATE% baud
echo ===================================================
echo.

:: Optional: Erase the flash memory first to clear old data
echo Erasing flash memory...
esptool.exe --chip esp32s3 --port %COM_PORT% erase_flash
if %errorlevel% neq 0 goto ERROR
echo.

:: Write binaries to their target memory locations
echo Writing flash...
esptool.exe --chip esp32s3 --port %COM_PORT% --baud %BAUD_RATE% --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 16MB %BOOTLOADER_ADDR% %BOOTLOADER_FILE% %PARTITIONS_ADDR% %PARTITIONS_FILE% %APP_ADDR% %APP_FILE%

if %errorlevel% neq 0 goto ERROR

echo.
echo ===================================================
echo   SUCCESS: ESP32 programmed completely!
echo ===================================================
goto END

:ERROR
echo.
echo ===================================================
echo   FAILED: An error occurred during flashing.
echo ===================================================

:END
pause
