#!/usr/bin/env python
import serial
from time import time, sleep
import math

import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Header
import tf
#from geometry_msgs.msg import PoseStamped

class SerialBridge():
    #MEASURE_TOPIC = "measurements"
    #IMU_TOPIC = "/imu/data"
    #COMPASS_TOPIC = "angle_to_true_north"
    #COMMAND_TOPIC = "command"

    def __init__(self, mbedPort='/dev/serial0', mbedBaud = 115200, mbedUpdateInterval=1.25):
        #rospy.loginfo("Serial node started.")
        print('Serial node started.')
        
        self.imu_pub = rospy.Publisher("/imu/data/raw", Imu, queue_size=1)
        self.compass_pub = rospy.Publisher("angle_to_true_north", Float64, queue_size=1)
        rospy.Subscriber("command", String, self.callback)

        self.cmd_arr_order = ['start', 'pitch', 'yaw', 'thrust', 'frequency']
        self._mbedSerial = serial.Serial(mbedPort, baudrate=mbedBaud, timeout=0, bytesize=serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
        self._mbedUpdateInterval = mbedUpdateInterval
        self.CMD_MAX = 6
        self.CMD_MIN = 0

        self.buf = []
        self.incomingStringLength = 79

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

    # Listens to incoming fixed length data string from mbed.
    # data[0]: roll
    # data[1]: pitch
    # data[2]: yaw
    # data[3]: gyro_x
    # data[4]: gyro_y
    # data[5]: gyro_z
    # data[6]: linaccel_x
    # data[7]: linaccel_y
    # data[8]: linaccel_z
    # data[9]: angle to true north
    def listen(self):
        while not rospy.is_shutdown():
            if self._mbedSerial.inWaiting():
                #bytesToRead = self._mbedSerial.inWaiting()
                x = self._mbedSerial.read_until()
                self.buf.append(x)
                if len(self.buf) > self.incomingStringLength:
                    self._mbedSerial.flush()
                    msg = self.convert(self.buf)
                    data = self.parseSensorData(msg)
                    rospy.loginfo(data)
                    quat_array = tf.transformations.quaternion_from_euler(data[1] * math.pi/180, data[0] * math.pi/180, data[2] * math.pi/180)
                    imu_msg = Imu()
                    h = Header()
                    h.stamp = rospy.Time.now()
                    h.frame_id = "base_link"
                    imu_msg.header = h
                    imu_msg.orientation.x = quat_array[0]
                    imu_msg.orientation.y = quat_array[1]
                    imu_msg.orientation.z = quat_array[2]
                    imu_msg.orientation.w = quat_array[3]
                    imu_msg.orientation_covariance = [0.002, 0, 0, 0, 0.002, 0, 0, 0, 0.002]
                    imu_msg.angular_velocity.x = data[3]
                    imu_msg.angular_velocity.y = data[4]
                    imu_msg.angular_velocity.z = data[5]
                    imu_msg.angular_velocity_covariance = [0.003, 0, 0, 0, 0.003, 0, 0, 0, 0.003]
                    imu_msg.linear_acceleration.x = data[6]
                    imu_msg.linear_acceleration.y = data[7]
                    imu_msg.linear_acceleration.z = data[8]
                    imu_msg.linear_acceleration_covariance = [0.60, 0, 0, 0, 0.60, 0, 0, 0, 0.60]
                    self.imu_pub.publish(imu_msg)
                    angle_to_true_north = Float64()
                    angle_to_true_north.data = data[9]
                    self.compass_pub.publish(angle_to_true_north)
                    self.buf = []

    def callback(self, msg):
        cmd = msg.data
        arr = cmd.split(",")
        for i in range(len(arr)):
            arr[i] = int(arr[i])
        self.writeOnce(arr)
        print(arr)
        #rospy.loginfo(arr)

if __name__ == '__main__':
    import sys
    # update_hz = 30
    rospy.init_node('serial', anonymous=True)
    piSerial = SerialBridge()
    piSerial.listen()
    rospy.spin()
