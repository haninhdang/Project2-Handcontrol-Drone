#!/usr/bin/env python

import numpy as np
from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import copy
import pandas as pd

class SerialRead:
    def __init__(self, serialPort='/dev/cu.usbmodemC7FD1A6939441', serialBaud=19200, plotLength=100, dataNumBytes=2, numPlots=1):
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.plotMaxLength = plotLength

        #self.numParams = numParams
        self.numPlots = numPlots
        self.rawData =bytearray()# bytearray(numParams)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        self.data = []

        for i in range(numPlots):  # give an array for each type of data and store them in a list
            self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:

            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            #self.serialConnection.open()
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
            exit()

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self, frame, lines, lineValueText, lineLabel, timeText):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)  # the first reading will be erroneous
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
        privateData = copy.deepcopy(self.rawData[:])  # so that the 3 values in our plots will be synchronized to the same sample time
        for i in range(3):
            string_privateData = privateData.decode().strip().split(" ")
            data = [float(x) for x in string_privateData]
            value=data[i]
            if i == 0:
                value = (value)# * 70)) / 180.0 * np.pi  # unit of degree+0.004544836039999999
            elif i == 1:
                value = (value)# * 70)) / 180.0 * np.pi  # +0.015100584719999999
            elif i == 2:
                value = (value)# * 70)) / 180.0 * np.pi  # -0.005473351759999999

            self.data[i].append(value)            #self.data[i] = value
            #data = privateData[(i * self.dataNumBytes):(self.dataNumBytes + i * self.dataNumBytes)]
            #value, = struct.unpack(self.dataType, data)
            #self.data[i].append(value)  # we get the latest data point and append it to our array
            lines[i].set_data(range(self.plotMaxLength), self.data[i])
            lineValueText[i].set_text('[' + lineLabel[i] + '] = ' + str(value))




    def backgroundThread(self):    # retrieve data
        time.sleep(0.05)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.rawData=self.serialConnection.readline() #not using readinto anymore, Use readline for bytes
            self.isReceiving = True
            #print(self.rawData) #<class 'bytes'>
            #print(type(self.rawData ))
            #example: b'0.01 -0.02 0.00 -6.08 -7.85 0.19 -14.00 59.22 -25.61\r\n'
            #print(type(a))



    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')

####################################################
def main():
    portName = '/dev/cu.usbmodemC7FD1A6939441'
    baudRate = 19200
    maxPlotLength = 100     # number of points in x-axis of real time plot
    dataNumBytes = 4        # number of bytes of 1 data point
    numPlots = 3            # number of plots in 1 graph
    s = SerialRead(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 50  # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = -(10)
    ymax = 10
    fig = plt.figure(figsize=(10, 8))
    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Accelerometer')
    ax.set_xlabel("Time")
    ax.set_ylabel("Accelerometer Output")

    lineLabel = ['X', 'Y', 'Z']
    style = ['r-', 'c-', 'b-']  # linestyles for the different plots
    timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    lines = []
    lineValueText = []

    for i in range(numPlots):
        lines.append(ax.plot([], [], style[i], label=lineLabel[i])[0])
        lineValueText.append(ax.text(0.70, 0.90 - i * 0.05, '', transform=ax.transAxes))
    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, lineValueText, lineLabel, timeText),
                                   interval=pltInterval)  # fargs has to be a tuple

    plt.legend(loc="upper left")
    plt.show()

    s.close()


if __name__ == '__main__':
    main()