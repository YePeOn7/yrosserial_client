import serial
import time
# s = serial.Serial("/dev/ttyUSB0", 9600)
s = serial.Serial("/dev/ttyACM0", 1000000)

x = 0
while 1:
    s.write(f"1234567890".encode()) 
    print(x)
    x += 1
    # exit(0)
    time.sleep(1)
