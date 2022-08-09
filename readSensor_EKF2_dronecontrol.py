#!/usr/bin/env python

from threading import Thread
import serial
import time
#import struct
import numpy as np
import pandas as pd


class SerialRead:
    def __init__(self, serialPort='/dev/cu.usbmodemC7FD1A6939441', serialBaud=19200, dataNumBytes=2, numParams=9):
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.numParams = numParams
        self.rawData =bytearray()# bytearray(numParams)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        self.data = np.zeros(numParams)
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        # self.csvData = []
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

    def getSerialData(self):
        privateData = self.rawData[:]
        #print(privateData)
        for i in range(9):
            string_privateData = privateData.decode().strip().split(" ")
            data = [float(x) for x in string_privateData]
            value=data[i]
            #Accelerometer range set to: 8 G
            #Gyro range set to: 2000 DPS
            #Accelerometer rate set to: 1066 HZ
            #Gyro rate set to: 1066 HZ
            if i == 0:
                value = (value) #unit of rad/s
            elif i == 1:
                value = (value)
            elif i == 2:
                value = (value)
            elif i == 3:
                value = (value) #unit of m/s^2
            elif i == 4:
                value = (value)
            elif i == 5:
                value = (value)
            elif i == 6:
                value = value *0.01 #unit of gauss (micro Tesla*0.01)
            elif i == 7:
                value = value *0.01
            elif i == 8:
                value = value *0.01
            #print(i)
            self.data[i] = value
        return self.data
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
