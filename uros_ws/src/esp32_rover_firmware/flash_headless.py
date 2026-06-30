#!/usr/bin/env python3
import time
import serial
import subprocess
import sys

PORT = '/dev/ttyUSB1'

print(f"Forcing ESP32 on {PORT} into bootloader mode...")

try:
    s = serial.Serial(PORT, 115200, timeout=1)
    # Sequence to enter bootloader
    # 1. EN = 0, IO0 = 1 (DTR=True, RTS=False)
    s.dtr = True
    s.rts = False
    time.sleep(0.1)
    
    # 2. EN = 0, IO0 = 0 (DTR=True, RTS=True)
    s.dtr = True
    s.rts = True
    time.sleep(0.1)
    
    # 3. EN = 1, IO0 = 0 (DTR=False, RTS=True)
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    
    # 4. EN = 1, IO0 = 1 (DTR=False, RTS=False) - Wait let's just close the port while IO0 is held low
    s.close()
    
    print("ESP32 should now be in bootloader mode. Running PlatformIO upload...")
    
    # Run PIO upload
    subprocess.run(["pio", "run", "-t", "upload", "-e", "esp32dev"], check=True)
    
    # Afterwards, reset to normal execution
    print("Upload complete. Rebooting ESP32...")
    s = serial.Serial(PORT, 115200, timeout=1)
    s.dtr = False
    s.rts = False
    time.sleep(0.1)
    s.dtr = True
    s.rts = False
    time.sleep(0.1)
    s.dtr = False
    s.rts = False
    s.close()
    print("Done!")
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
