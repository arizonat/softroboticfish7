from __future__ import print_function
import serial
import time

inp = raw_input("Enter testname: testname\n")

# Ports: /dev/serial0 for wired UART, /dev/ttyACM* for USB (use ls -l /dev to find ports)
# NOTE: REMOVE ANY CONSOLE REFERENCES TO SERIAL0 FROM /boot/cmdline.txt AND REBOOT
# NOTE: timeout=0 or 1 both seem to work

testname = inp
ser = serial.Serial('/dev/serial0', 115200, timeout=0, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
print(ser.name)

logfile = open("logs/" + testname, "w")
ser.flush()

buf = []
incomingStrLength = 79
data = ''

def convert(s): 
  
    # initialization of string to "" 
    str1 = "" 
  
    # using join function join the list s by  
    # separating words by str1 
    return(str1.join(s)) 


while True:
    if ser.inWaiting():
        x = ser.read_until()
        buf.append(x)
        #print(buf)
        #print(len(buf))
        if len(buf) > incomingStrLength:
            ser.flush()
            data = convert(buf)
            print(data)
            buf = []
        #print(x),
        #print(x, end="")
        logfile.write(data)
        # time.sleep(0.25)
        logfile.flush()
