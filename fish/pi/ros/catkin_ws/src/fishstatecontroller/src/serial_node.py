#!/usr/bin/env python

import rospy
import roslib
import serial
from time import time, sleep
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, Imu
import tf
#from geometry_msgs.msg import PoseStamped
#TODO need to look at the signs for the pitch, yaw, and thrust commands
class SerialBridge():
    #MEASURE_TOPIC = "measurements"
    #IMU_TOPIC = "imu_data"
    #COMPASS_TOPIC = "angle_to_true_north"
    #COMMAND_TOPIC = "command"

    def __init__(self, mbedPort='/dev/serial0', mbedBaud = 115200, mbedUpdateInterval=1.25):
        #rospy.loginfo("Serial node started.")
        print('Serial node started.')
        
        # self.imu_pub = rospy.Publisher("imu_data", Imu, queue_size=1)
        # self.compass_pub = rospy.Publisher("angle_to_true_north", Float64, queue_size=1)
        # rospy.Subscriber("command", String, self.callback)

        self.cmd_arr_order = ['start', 'pitch', 'yaw', 'thrust', 'frequency']
        self._mbedSerial = serial.Serial(mbedPort, baudrate=mbedBaud, timeout=0, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
        self._mbedUpdateInterval = mbedUpdateInterval
        self.CMD_MAX = 255
        self.CMD_MIN = 1

        self.buf = []
        self.incomingStringLength = 31

        self.cmd_received = False
        self.pitch = 128
        self.yaw = 128
        self.thrust = 128
        self.frequency = 128
        self.DO_NOTHING = [255,135,135,1,85]

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

    def writeOnce(self, cmd):
        self._mbedSerial.flushInput()
        self._mbedSerial.flushOutput()
        self.writeBytes(cmd)

    def convert(self, s):
        str1 = ""
        return(str1.join(s))

    def parseSensorData(self, data):
        data = data.replace(" ","")
        arr = data.split(",")
        values = [float(i) for i in arr]
        return values

    # def listen(self):
    #     while not rospy.is_shutdown():
    #         if self._mbedSerial.inWaiting():
    #             #bytesToRead = self._mbedSerial.inWaiting()
    #             x = self._mbedSerial.read_until()
    #             self.buf.append(x)
    #             if len(self.buf) > self.incomingStringLength:
    #                 self._mbedSerial.flush()
    #                 msg = self.convert(self.buf)
    #                 data = self.parseSensorData(msg)
    #                 rospy.loginfo(data)
    #                 quat_array = tf.transformations.quaternion_from_euler(data[1], data[0], data[2])
    #                 imu_msg = Imu()
    #                 imu_msg.orientation.w = quat_array[0]
    #                 imu_msg.orientation.x = quat_array[1]
    #                 imu_msg.orientation.y = quat_array[2]
    #                 imu_msg.orientation.z = quat_array[3]
    #                 self.imu_pub.publish(imu_msg)
    #                 angle_to_true_north = Float64()
    #                 angle_to_true_north.data = data[3]
    #                 self.compass_pub.publish(angle_to_true_north)
    #                 self.buf = []

    def heading_callback(self, ros_data):
        self.cmd_received = True
        self.yaw = int(128 + (127 *ros_data.data))

    def pitch_callback(self, ros_data):
        self.pitch = int(128 - (127 * ros_data.data))
    
    def thrust_callback(self, ros_data):
        self.thrust = int(128 - (127 * ros_data.data))
        #self.frequency = ?? (should frequency be dependent on the distance between the fish and the target?)
    
    def write(self):
        rate = rospy.Rate(24)
        while not rospy.is_shutdown():
            #if new commands have been received, write an updated cmd array, otherwise write (do nothing)
            if self.cmd_received:
                arr = [255, self.pitch, self.yaw, self.thrust, self.frequency]
                self.writeOnce(arr)
                print(arr)
                #rospy.loginfo(arr)
                self.cmd_received = False
            #else:
            #    self.writeOnce(self.DO_NOTHING)
            rate.sleep()

if __name__ == '__main__':
    import sys
    # update_hz = 24
    rospy.init_node('serial', anonymous=True)
    piSerial = SerialBridge()

    rospy.Subscriber('heading_cmd', Float64, piSerial.heading_callback)
    rospy.Subscriber('pitch_cmd', Float64, piSerial.pitch_callback)
    rospy.Subscriber('thrust_cmd', Float64, piSerial.thrust_callback)
    print("\nSerial Node: Beginning at 24hz")
    piSerial.write()
    print("\nSerial Node: done\n")
