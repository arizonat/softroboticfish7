from __future__ import print_function
import serial
import time

# TODO: EVERYTHING

inp = raw_input("Enter testname: testname\n")

# input_list = inp.split(" ")
# params = [i.strip() for i in input_list]
# port = params[0]
# testname = params[1]
# Ports: /dev/serial0 for wired UART, /dev/ttyACM* for USB (use ls -l /dev to find ports)
# NOTE: REMOVE ANY CONSOLE REFERENCES TO SERIAL0 FROM /boot/cmdline.txt AND REBOOT

testname = inp
ser = serial.Serial('/dev/serial0', 115200, timeout=1, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
print(ser.name)

logfile = open("logs/" + testname, "w")
ser.flush()

while True:
    if ser.inWaiting():
        bytesToRead = ser.inWaiting();
        x = ser.read(bytesToRead)
        print(x, end="")
        logfile.write(x)
        # time.sleep(0.25)
        logfile.flush()
