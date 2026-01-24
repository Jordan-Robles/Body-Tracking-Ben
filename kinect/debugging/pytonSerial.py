import serial, time

ser = serial.Serial("COM5", 115200, timeout=1)
time.sleep(2)

while True:
    ser.write("16,30\n")
    print("TX (16,30)")
    time.sleep(1)

    ser.write("16,-30\n")
    print("TX (16,-30)")
    time.sleep(1)
