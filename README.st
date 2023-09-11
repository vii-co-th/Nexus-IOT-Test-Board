#Uplaod Command
"C:\Users\xxxxx\.platformio\penv\Scripts\python.exe" "C:\Users\xxxxx\.platformio\packages\tool-esptoolpy\esptool.py" --chip esp32s3 --port "COM19" --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0000 C:\Users\xxxxx\.platformio\packages\framework-arduinoespressif32\tools\sdk\esp32s3\bin\bootloader_dio_80m.bin 0x8000 partitions.bin 0xe000 C:\Users\xxxxx\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin 0x10000 firmware.bin


