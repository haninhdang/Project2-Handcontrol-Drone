import serial
s=serial.Serial()
s.port = "/dev/cu.usbmodemC7FD1A6939441"
s.baudrate = 19200
s.open()
while True:
    string_privateData = s.readline().decode().strip().split(" ")
    data = [float(x) for x in string_privateData]
    print(*string_privateData)

