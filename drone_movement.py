from djitellopy import tello
from time import sleep
import readSensor_EKF2_dronecontrol as rs
import Wireframe_EKF_dronecontrol as wf
import pygame

ni = tello.Tello()
ni.connect()
ni.streamon()
print(ni.get_battery())

speed = 50
portName = '/dev/cu.usbmodemC7FD1A6939441'
baudRate = 19200
dataNumBytes = 2  # number of bytes of 1 data point
numParams = 9  # number of plots in 1 graph
s = rs.SerialRead(portName, baudRate, dataNumBytes, numParams)  # initializes all required variables
s.readSerialStart()  # starts background thread


block = wf.Wireframe()
running=True
loopRate = 50
while running:
    #clock.tick(loopRate)
    data = s.getSerialData()
    block.quatRotate([data[0], data[1], data[2]],
                        [data[3], data[4], data[5]],
                        [data[6], data[7], data[8]],1 / loopRate)

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    yaw, pitch, roll = block.getAttitude()
    print(yaw, pitch, roll)
        # for left and right
    if 130<roll<150:
        lr = -speed
    elif -130<roll<-150:
        lr = speed
        # for forward and backward
        #if kp.getKey("UP"):
         #   fb = speed
        #elif kp.getKey("DOWN"):
         #   fb = -speed
         #for up and down

        #if kp.getKey("w"):
           # ud = speed
        #elif kp.getKey("s"):
           # ud = -speed
        # for  yaw left and right
        #if kp.getKey("a"):
         #   yv = -speed
        #elif kp.getKey("d"):
          #  yv = speed
        # landing
    if pitch<-20: ni.land()
        # takeoff
    if pitch>60: ni.takeoff()

    return [lr, fb, ud, yv]



while True:
    vals = getKeyboardInput()
    ni.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    sleep(0.05)
