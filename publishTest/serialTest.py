import serial

# s = serial.Serial("/dev/ttyUSB0", 9600)
s = serial.Serial("/dev/ttyACM0", 9600)

s.write("123456789012345678901234567890".encode())