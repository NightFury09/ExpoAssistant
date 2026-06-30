#!/usr/bin/env python3
import sys
import time
import serial

def reset_esp32(port='/dev/ttyESP32', baud=115200):
    try:
        print(f"Opening {port} to reset ESP32...")
        s = serial.Serial(port, baud, timeout=1)
        
        # Pull DTR and RTS low (deassert)
        s.dtr = False
        s.rts = False
        time.sleep(0.1)
        
        # Assert DTR (pulls EN low, resetting the ESP32)
        s.dtr = True
        s.rts = False
        time.sleep(0.1)
        
        # Deassert DTR (EN goes high, ESP32 boots)
        s.dtr = False
        s.rts = False
        
        s.close()
        print("ESP32 hardware reset triggered. Waiting 2 seconds for boot...")
        time.sleep(2.0)
        return 0
    except Exception as e:
        print(f"Failed to reset ESP32 on {port}: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(reset_esp32())
