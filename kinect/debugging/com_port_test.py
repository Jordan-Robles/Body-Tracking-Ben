import serial
import time

# Try different ports
ports_to_try = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7']

for port in ports_to_try:
    try:
        print("Trying {}...".format(port))
        arduino = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        print("SUCCESS! Arduino found on {}".format(port))
        arduino.write(b"(14,45.0)\n")
        time.sleep(0.5)
        
        # Read response
        if arduino.in_waiting:
            response = arduino.readline().decode().strip()
            print("Arduino says: {}".format(response))
        
        arduino.close()
        break
    except Exception as e:
        print("Failed: {}".format(e))

print("\nDone testing ports")