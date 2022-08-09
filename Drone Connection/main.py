from djitellopy import tello
import Wireframe_EKF_dronecontrol as wf
import pygame
from operator import itemgetter
import readSensor_EKF2_dronecontrol as rs
import time
import math

ni = tello.Tello()
ni.connect()
ni.streamon()
print(ni.get_battery())

class ProjectionViewer:
    """ Displays 3D objects on a Pygame screen """
    BASE = 0
    PEAK = 1
    WAIT = 2

    state = BASE
    peakValue = 0.0
    tempzVel = 0.0
    speed = 50
    direction=0
    def __init__(self, width, height, wireframe):
        self.width = width
        self.height = height
        self.wireframe = wireframe
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Attitude Determination using Quaternions')
        self.background = (10,10,50)
        self.clock = pygame.time.Clock()
        pygame.font.init()
        self.font = pygame.font.SysFont('Comic Sans MS', 30)

    def getKeyboardInput(self, yaw, pitch, roll,accx,accy,accz,gyroy):
        ACCEL_VEL_TRANSITION = 10.0 / 1000.0
        DEG_2_RAD = 0.01745329251
        zVel = (ACCEL_VEL_TRANSITION * accz / math.cos(DEG_2_RAD * accz)) * 10.0-1
        xVel = (ACCEL_VEL_TRANSITION * accx / math.cos(DEG_2_RAD * accx)) * 10.0 - 1
        lr, fb, ud, yv = 0, 0, 0, 0
        # for left and right
        if 90 < roll < 160:
            lr = -self.speed
        elif -160 < roll < -90:
            lr = self.speed
         #for forward and backward
        if -70 < pitch < -30:
            fb = self.speed
        elif 30 < pitch < 70:
            fb = -self.speed
        # for up and down
        if zVel > 1.5:
            ud = 100
            time.sleep(0.07)
        elif zVel < -1.3:
            ud = -100
            time.sleep(0.07)
        #for  yaw left and right
        if yaw > 3.5:
            yv = -100
            time.sleep(0.07)
        elif -3.5  > yaw:
            yv = 100
            time.sleep(0.07)
        # landing and take off
        if accz<=1 and accx<=1 and accy >=9.5: ni.land()
        if accz <=1 and accx<=1 and accy<=-9.5: ni.takeoff()

        return [lr, fb, ud, yv]

    def run(self, sensorInstance):
        """ Create a pygame screen until it is closed. """
        running = True
        loopRate = 20
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
                                      1/loopRate)

            yaw, pitch, roll = self.wireframe.getAttitude()
            yaw = round(yaw,0)
            pitch = round(pitch,0)
            roll = round(roll,0)
            print(yaw, pitch, roll,round(data[2],0), pitch, roll,round(data[3],0),round(data[4],0),round(data[5]))
            vals = self.getKeyboardInput(round(data[2]), pitch, roll,round(data[3],0),round(data[4],0),round(data[5]),round(data[1],2))
            ni.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    def display(self):
        """ Draw the wireframes on the screen. """
        self.screen.fill(self.background)
        # Get the current attitude
        yaw, pitch, roll = self.wireframe.getAttitude()
        self.messageDisplay("Yaw: %.1f" % yaw,
                            self.screen.get_width()*0.75,
                            self.screen.get_height()*0,
                            (220, 20, 60))      # Crimson
        self.messageDisplay("Pitch: %.1f" % pitch,
                            self.screen.get_width()*0.75,
                            self.screen.get_height()*0.1,
                            (0, 255, 255))     # Cyan
        self.messageDisplay("Roll: %.1f" % roll,
                            self.screen.get_width()*0.75,
                            self.screen.get_height()*0.2,
                            (65, 105, 225))    # Royal Blue
        # Transform nodes to perspective view
        dist = 5
        pvNodes = []
        pvDepth = []
        for node in self.wireframe.nodes:
            point = [node.x, node.y, node.z]
            newCoord = self.wireframe.rotatePoint(point)
            comFrameCoord = self.wireframe.convertToComputerFrame(newCoord)
            pvNodes.append(self.projectOthorgraphic(comFrameCoord[0], comFrameCoord[1], comFrameCoord[2],
                                                    self.screen.get_width(), self.screen.get_height(),
                                                    70, pvDepth))
            """
            pvNodes.append(self.projectOnePointPerspective(comFrameCoord[0], comFrameCoord[1], comFrameCoord[2],
                                                           self.screen.get_width(), self.screen.get_height(),
                                                           5, 10, 30, pvDepth))
            """
        # Calculate the average Z values of each face.
        avg_z = []
        for face in self.wireframe.faces:
            n = pvDepth
            z = (n[face.nodeIndexes[0]] + n[face.nodeIndexes[1]] +
                 n[face.nodeIndexes[2]] + n[face.nodeIndexes[3]]) / 4.0
            avg_z.append(z)
        # Draw the faces using the Painter's algorithm:
        for idx, val in sorted(enumerate(avg_z), key=itemgetter(1)):
            face = self.wireframe.faces[idx]
            pointList = [pvNodes[face.nodeIndexes[0]],
                         pvNodes[face.nodeIndexes[1]],
                         pvNodes[face.nodeIndexes[2]],
                         pvNodes[face.nodeIndexes[3]]]
            pygame.draw.polygon(self.screen, face.color, pointList)
    # One vanishing point perspective view algorithm

    def projectOnePointPerspective(self, x, y, z, win_width, win_height, P, S, scaling_constant, pvDepth):
        # In Pygame, the y axis is downward pointing.
        # In order to make y point upwards, a rotation around x axis by 180 degrees is needed.
        # This will result in y' = -y and z' = -z
        xPrime = x
        yPrime = -y
        zPrime = -z
        xProjected = xPrime * (S/(zPrime+P)) * scaling_constant + win_width / 2
        yProjected = yPrime * (S/(zPrime+P)) * scaling_constant + win_height / 2
        pvDepth.append(1/(zPrime+P))
        return (round(xProjected), round(yProjected))

    # Normal Projection
    def projectOthorgraphic(self, x, y, z, win_width, win_height, scaling_constant, pvDepth):
        # In Pygame, the y axis is downward pointing.
        # In order to make y point upwards, a rotation around x axis by 180 degrees is needed.
        # This will result in y' = -y and z' = -z
        xPrime = x
        yPrime = -y
        xProjected = xPrime * scaling_constant + win_width / 2
        yProjected = yPrime * scaling_constant + win_height / 2
        # Note that there is no negative sign here because our rotation to computer frame
        # assumes that the computer frame is x-right, y-up, z-out
        # so this z-coordinate below is already in the outward direction
        pvDepth.append(z)
        return (round(xProjected), round(yProjected))

    def messageDisplay(self, text, x, y, color):
        textSurface = self.font.render(text, True, color, self.background)
        textRect = textSurface.get_rect()
        textRect.topleft = (x, y)
        self.screen.blit(textSurface, textRect)

def initializeCube():
    block = wf.Wireframe()

    block_nodes = [(x, y, z) for x in (-1.5, 1.5) for y in (-1, 1) for z in (-0.1, 0.1)]
    node_colors = [(255, 255, 255)] * len(block_nodes)
    block.addNodes(block_nodes, node_colors)
    block.outputNodes()

    faces = [(0, 2, 6, 4), (0, 1, 3, 2), (1, 3, 7, 5), (4, 5, 7, 6), (2, 3, 7, 6), (0, 1, 5, 4)]
    colors = [(255, 0, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 0)]
    block.addFaces(faces, colors)
    block.outputFaces()

    return block


if __name__ == '__main__':
    portName = '/dev/cu.usbmodemC7FD1A6939441'
    baudRate = 19200
    dataNumBytes = 2 # number of bytes of 1 data point
    numParams = 9  # number of plots in 1 graph
    s = rs.SerialRead(portName, baudRate, dataNumBytes, numParams)  # initializes all required variables
    s.readSerialStart()  # starts background thread
    block = initializeCube()
    pv = ProjectionViewer(640, 480, block)
    pv.run(s)
