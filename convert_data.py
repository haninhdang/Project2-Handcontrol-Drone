from time import sleep
import readSensor_EKF2_dronecontrol as rs
import Wireframe_EKF_dronecontrol as wf

portName = '/dev/cu.usbmodemC7FD1A6939441'
baudRate = 19200
dataNumBytes = 2  # number of bytes of 1 data point
numParams = 9  # number of plots in 1 graph
s = rs.SerialRead(portName, baudRate, dataNumBytes, numParams)  # initializes all required variables
s.readSerialStart()  # starts background thread

while True:
    print(s.getSerialData())

running = True
loopRate = 50
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            sensorInstance.close()
    self.clock.tick(loopRate)
    data = sensorInstance.getSerialData()
    self.wireframe.quatRotate([data[0], data[1], data[2]],
                              [data[3], data[4], data[5]],
                              [data[6], data[7], data[8]],
                              1 / loopRate)
    # round(data[0],2),round(data[1],2),round(data[2],2),round(data[3],2),round(data[4],2),,round(data[6],2),round(data[7],2),round(data[8],2)
    # print(round(data[5],2))
    # print(block.getAttitude())
    self.display()
    pygame.display.flip()
