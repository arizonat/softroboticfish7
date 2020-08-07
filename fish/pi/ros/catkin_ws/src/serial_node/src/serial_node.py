#!/usr/bin/env python
import serial
from time import time, sleep

import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from geometry_msgs.msg import PoseStamped

class SerialBridge():
    #MEASURE_TOPIC = "measurements"
    #COMMAND_TOPIC = "command"

    def __init__(self, mbedPort='/dev/serial0', mbedBaud = 115200, mbedUpdateInterval=1.25):
        #rospy.loginfo("Serial node started.")
        print('Serial node started.')
        
        self.pub = rospy.Publisher("measurements", String, queue_size=1)
        rospy.Subscriber("command", String, self.callback)

        self.cmd_arr_order = ['start', 'pitch', 'yaw', 'thrust', 'frequency']
        self._mbedSerial = serial.Serial(mbedPort, baudrate=mbedBaud, timeout=None, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
        self._mbedUpdateInterval = mbedUpdateInterval
        self.CMD_MAX = 6
        self.CMD_MIN = 0

    def writeCmdArray(self, cmd):
        bytecmds = self.safeCmdToBytes(cmd)
        self.writeBytes(bytecmds)

    def writeBytes(self, bytecmds):
        self._mbedSerial.write(bytecmds)
        if bytecmds[-1] != 0:
            self._mbedSerial.write(bytearray([0]))
        self._mbedSerial.flush()

    def safeCmdToBytes(self, cmd, cmdType='byteArray', nullTerminate=False):
        if cmdType == "byteArray":
            for i,val in enumerate(cmd):
                cmd[i] = max(min(cmd[i],self.CMD_MAX),self.CMD_MIN)
        elif cmdType == "dict":
            for k in self.self.cmd_arr_order:
                cmd[k] = max(min(cmd[k], self.CMD_MAX), self.CMD_MIN)

        return self.cmdToBytes(cmd, cmdType, nullTerminate)

    def cmdToBytes(self, cmd, cmdType='byteArray', nullTerminate=False):
        if cmdType == "dict":
            res = [cmd[cmd_key] for cmd_key in self.cmd_arr_order]
        else:
            res = cmd

        assert(len(res) == len(self.cmd_arr_order))
        if nullTerminate:
            res.append(0)

        return bytearray(res)

    def runOnce(self, cmd):
        self._mbedSerial.flushInput()
        self._mbedSerial.flushOutput()
        self.writeBytes(cmd)

    def run(self):
        while not rospy.is_shutdown():
            if self._mbedSerial.inWaiting():
                bytesToRead = self._mbedSerial.inWaiting()
                x = self._mbedSerial.read(bytesToRead)
                print(x),
                #rospy.loginfo(x)

    def callback(self, msg):
        cmd = msg.data
        arr = cmd.split(",")
        for i in range(len(arr)):
            arr[i] = int(arr[i])
        self.runOnce(arr)
        print(arr)
        #rospy.loginfo(arr)

if __name__ == '__main__':
    import sys
    # update_hz = 30
    rospy.init_node('serial', anonymous=True)
    piSerial = SerialBridge()
    piSerial.run()
    rospy.spin()
