import serial # see http://pyserial.readthedocs.org/en/latest/pyserial_api.html
#import fishcamera
from time import time, sleep
from FishJoystick import FishJoystick
#from camera import FishCamera
import cv2

CAM_OUTPUT_DIR="/home/pi/fish_recordings"
DISPLAY_IMAGES = False

class FishMbed():
  def __init__(self, mbedPort='/dev/serial0', mbedBaud=115200, mbedUpdateInterval=1.25):
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
        cmd[k] = max(min(cmd[k],self.CMD_MAX),self.CMD_MIN)

    return self.cmdToBytes(cmd, cmdType, nullTerminate)
    
  def cmdToBytes(self, cmd, cmdType='byteArray', nullTerminate=False):
    """
    Turns a fish mbed command to bytearray (for sending to mbed)
    """
    if cmdType == "dict":
      res = [cmd[cmd_key] for cmd_key in self.cmd_arr_order]
    else:
      res = cmd
      
    assert(len(res) == len(self.cmd_arr_order))
    if nullTerminate:
      res.append(8)
    return bytearray(res)

  def run(self, cmd, runTime=None):
    self._mbedSerial.flushInput()
    self._mbedSerial.flushOutput()
    lastSendTime = 0
    lastPrintTime = 0
    startTime = time()
    while runTime is None or time() - startTime < runTime:
      if time() - lastSendTime > self._mbedUpdateInterval:
        self.writeCmdArray(cmd)
        lastSendTime = time()
      if time() - lastPrintTime > 1:
        print cmd
        lastPrintTime = time()

  def runOnce(self, cmd):
    self._mbedSerial.flushInput()
    self._mbedSerial.flushOutput()
    self.writeBytes(cmd)

if __name__ == '__main__':
  import sys
  update_hz = 30
  controller = FishMbed()

  #HARD_LEFT = [1,3,6,3,0]
  #HARD_RIGHT = [1,3,0,3,0]
  #SOFT_LEFT = [1,3,5,3,0]
  #SOFT_RIGHT = [1,3,1,3,0]
  #DO_NOTHING = [1,3,3,0,1]
  #GO_FORWARD = [1,3,3,3,3]
  #PITCH_UP = [1,6,3,3,2]
  #PITCH_DOWN = [1,0,3,3,2]

  #g = raw_input("Enter a command: ")
  # cmds = {"HARD_LEFT": [1,3,6,3,0], "HARD_RIGHT": [1,3,0,3,0], "SOFT_LEFT": [1,3,5,3,0], "SOFT_RIGHT": [1,3,1,3,0], "DO_NOTHING": [1,3,3,0,1], "GO_FORWARD": [1,3,3,3,3], "PITCH_UP": [1,6,3,3,2], "PITCH_DOWN": [1,0,3,3,2]}
  print '\nStarting Test Controller'
  print 'using update interval of ', 1./update_hz, 's'
  # controller.run(cmds[g])
  while True:
    g = raw_input("Enter a command: ")
    arr = g.split(",")
    for i in range(len(arr)):
      arr[i] = int(arr[i])
    controller.runOnce(arr)
    print(arr)
  print '\nAll done!'
