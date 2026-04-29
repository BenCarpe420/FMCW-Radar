# ****************************************************************************
# * (C) Copyright 2020, Texas Instruments Incorporated. - www.ti.com
# ****************************************************************************
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions are
# *  met:
# *
# *    Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# *    Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the
# *     documentation and/or other materials provided with the distribution.
# *
# *    Neither the name of Texas Instruments Incorporated nor the names of its
# *    contributors may be used to endorse or promote products derived from
# *    this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# *  PARTICULAR TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# *  A PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR
# *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# *  EXEMPLARY, ORCONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# *  LIABILITY, WHETHER IN CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING
# *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *
# ****************************************************************************


# ****************************************************************************
# Sample mmW demo UART output parser script - should be invoked using python3
#       ex: python3 mmw_demo_example_script.py <recorded_dat_file_from_Visualizer>.dat
#
# Notes:
#   1. The parser_mmw_demo script will output the text version 
#      of the captured files on stdio. User can redirect that output to a log file, if desired
#   2. This example script also outputs the detected point cloud data in mmw_demo_output.csv 
#      to showcase how to use the output of parser_one_mmw_demo_output_packet
# ****************************************************************************
import serial
import time
import numpy as np
import os
import sys
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import warnings
# import the parser function 
from IWR6843_Read_Data_Python_MMWAVE_SDK_main.parser_mmw_demo import parser_one_mmw_demo_output_packet
import FMCW
# from sklearn import metrics
from sklearn.cluster import DBSCAN

# Change the configuration file name
# configFileName = 'profile_2024_03_15T00_14_41_424.cfg'


# Constants
maxBufferSize = 2**15;
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;
maxBufferSize = 2**15;
magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
detObj = {}  
frameData = {}    
currentIndex = 0
# word array to convert 4 bytes to a 32 bit number
word = [1, 2**8, 2**16, 2**24]

# Globals
g_x_data = [] # Global var so that when frame is dropped, nothing flickers and it repeats showing you the data
g_y_data = []
g_z_data = []
g_v_data = []

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName, comSerialPort, comDataPort):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Windows
    CLIport = serial.Serial(comSerialPort, 115200)
    Dataport = serial.Serial(comDataPort, 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
            digOutSampleRate = int(splitWords[11]);
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = round(float(splitWords[5]));

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters

##################################################################################
# USE parser_mmw_demo SCRIPT TO PARSE ABOVE INPUT FILES
##################################################################################
def readAndParseData14xx(Dataport, configParameters, DEBUG):
    #load from serial
    global byteBuffer, byteBufferLength

    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)

    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
    
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0

            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # Read the entire buffer
        readNumBytes = byteBufferLength
        if(DEBUG[0]):
            print("readNumBytes: ", readNumBytes)
        allBinData = byteBuffer
        if(DEBUG[0]):
            print("allBinData: ", allBinData[0], allBinData[1], allBinData[2], allBinData[3])

        # init local variables
        totalBytesParsed = 0;
        numFramesParsed = 0;

        # parser_one_mmw_demo_output_packet extracts only one complete frame at a time
        # so call this in a loop till end of file
        #             
        # parser_one_mmw_demo_output_packet function already prints the
        # parsed data to stdio. So showcasing only saving the data to arrays 
        # here for further custom processing
        parser_result, \
        headerStartIndex,  \
        totalPacketNumBytes, \
        numDetObj,  \
        numTlv,  \
        subFrameNumber,  \
        detectedX_array,  \
        detectedY_array,  \
        detectedZ_array,  \
        detectedV_array,  \
        detectedRange_array,  \
        detectedAzimuth_array,  \
        detectedElevation_array,  \
        detectedSNR_array,  \
        detectedNoise_array = parser_one_mmw_demo_output_packet(allBinData[totalBytesParsed::1], readNumBytes-totalBytesParsed,DEBUG[0])
        
        # Check the parser result
        if(DEBUG[0]):
            print ("Parser result: ", parser_result)
        if (parser_result == 0): 
            totalBytesParsed += (headerStartIndex+totalPacketNumBytes)    
            numFramesParsed+=1
            if(DEBUG[0]):
                print("totalBytesParsed: ", totalBytesParsed)
            ##################################################################################
            # TODO: use the arrays returned by above parser as needed. 
            # For array dimensions, see help(parser_one_mmw_demo_output_packet)
            # help(parser_one_mmw_demo_output_packet)
            ##################################################################################

            if DEBUG[1]:
                FMCW.demoOutputWriter.writerow("        ") # This can tell us the amount of time lost between frames     
                if (numFramesParsed == 1): # Changed from 1 
                    for obj in range(numDetObj):
                        FMCW.demoOutputWriter.writerow([numFramesParsed-1, obj, detectedX_array[obj],\
                                            detectedY_array[obj],\
                                            detectedZ_array[obj],\
                                            detectedV_array[obj],\
                                            detectedSNR_array[obj],\
                                            detectedNoise_array[obj]])
                # For example, dump all S/W objects to a csv file
            detObj = {"numObj": numDetObj, "range": detectedRange_array, \
                        "x": detectedX_array, "y": detectedY_array, "z": detectedZ_array, "v": detectedV_array}
            dataOK = 1 
        # else: 
            # error in parsing; exit the loop
            # print("error in parsing this frame; continue")

        
        shiftSize = totalPacketNumBytes            
        byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
        byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
        byteBufferLength = byteBufferLength - shiftSize
        
        # Check that there are no errors with the buffer length
        if byteBufferLength < 0:
            byteBufferLength = 0
        # All processing done; Exit
        if(DEBUG[0]):
            print("numFramesParsed: ", numFramesParsed)

    return dataOK, frameNumber, detObj



# https://pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/#download-the-code


# import the necessary packages
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
class CentroidTracker():
    def __init__(self, maxDisappeared=50):
		# initialize the next unique object ID along with two ordered
		# dictionaries used to keep track of mapping a given object
		# ID to its centroid and number of consecutive frames it has
		# been marked as "disappeared", respectively
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
		# store the number of maximum consecutive frames a given
		# object is allowed to be marked as "disappeared" until we
		# need to deregister the object from tracking
        self.maxDisappeared = maxDisappeared
        
    def register(self, centroid):
		# when registering an object we use the next available object
		# ID to store the centroid
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1
    
    def deregister(self, objectID):
		# to deregister an object ID we delete the object ID from
		# both of our respective dictionaries
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, centroids):
		# check to see if the list of input bounding box rectangles
		# is empty
        if len(centroids) == 0:
			# loop over any existing tracked objects and mark them
			# as disappeared
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1
				# if we have reached a maximum number of consecutive
				# frames where a given object has been marked as
				# missing, deregister it
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
			# return early as there are no centroids or tracking info
			# to update
            return self.objects
          
		# initialize an array of input centroids for the current frame
        inputCentroids = centroids

        # if we are currently not tracking any objects take the input
		# centroids and register each of them
        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])
                    
		# otherwise, are are currently tracking objects so we need to
		# try to match the input centroids to existing object
		# centroids
        else:
			# grab the set of object IDs and corresponding centroids
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())
			# compute the distance between each pair of object
			# centroids and input centroids, respectively -- our
			# goal will be to match an input centroid to an existing
			# object centroid
            D = dist.cdist(np.array(objectCentroids), inputCentroids)
			# in order to perform this matching we must (1) find the
			# smallest value in each row and then (2) sort the row
			# indexes based on their minimum values so that the row
			# with the smallest value is at the *front* of the index
			# list
            rows = D.min(axis=1).argsort()
			# next, we perform a similar process on the columns by
			# finding the smallest value in each column and then
			# sorting using the previously computed row index list
            cols = D.argmin(axis=1)[rows]
               
			# in order to determine if we need to update, register,
			# or deregister an object we need to keep track of which
			# of the rows and column indexes we have already examined
            usedRows = set()
            usedCols = set()
			# loop over the combination of the (row, column) index
			# tuples
            for (row, col) in zip(rows, cols):
				# if we have already examined either the row or
				# column value before, ignore it
				# val
                if row in usedRows or col in usedCols:
                    continue
				# otherwise, grab the object ID for the current row,
				# set its new centroid, and reset the disappeared
				# counter
                objectID = objectIDs[row]
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0
				# indicate that we have examined each of the row and
				# column indexes, respectively
                usedRows.add(row)
                usedCols.add(col)

			# compute both the row and column index we have NOT yet
			# examined
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

			# in the event that the number of object centroids is
			# equal or greater than the number of input centroids
			# we need to check and see if some of these objects have
			# potentially disappeared
            if D.shape[0] >= D.shape[1]:
				# loop over the unused row indexes
                for row in unusedRows:
					# grab the object ID for the corresponding row
					# index and increment the disappeared counter
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1
					# check to see if the number of consecutive
					# frames the object has been marked "disappeared"
					# for warrants deregistering the object
                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
                              
			# otherwise, if the number of input centroids is greater
			# than the number of existing object centroids we need to
			# register each new input centroid as a trackable object
            else:
                for col in unusedCols:
                    self.register(inputCentroids[col])
		# return the set of trackable objects
        return self.objects

class MyWidget(pg.GraphicsLayoutWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(50) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)

        self.DEBUG = False
        self.colorVar = "v" # "v" for velocity and "z" for elevation z

        self.plotItem = self.addPlot(title="Point Cloud Data")


        self.last_X_data = []
        self.last_Y_data = []
        self.last_z_data = []
        self.last_v_data = []

        self.numPeople = 0

        self.tracker = CentroidTracker()
        self.pastTracks = OrderedDict()
        self.prevKeys = OrderedDict([(0,0)]).keys() 

        self.x_centroids = []
        self.y_centroids = []
        self.trackedTargets = {}

        self.plotItem.setXRange(-5, 5, padding=None, update=True)
        # With the clutterRemoval value at 1, the window goes crazy due to autoscaling issues
        self.plotItem.setYRange(0, 10, padding=None, update=True)

        self.plotDataItem = self.plotItem.plot([], pen=None, 
            symbolBrush=(255,0,0), symbolSize=10, symbolPen=None)
        self.configParameters = {}
        self.scatter = pg.ScatterPlotItem(pxMode = False)

        self.peopleInside = set()
        self.xBounds = 5
        self.yBounds = 2


    def setData(self, x, y, z, v):
        # This is choosing whether we map color from elevation or dopplar velocity
        if self.colorVar == "z" :
            colorMap = z
        elif self.colorVar == "v":
            colorMap = v
        else:
            warnings.warn("Unknown colorVar! Defaulting to constant color.")
            colorMap = np.ones((1, len(x)))*0.5

        # This maps the data from z or v to the color of the dot given in the 'brush' data
        spots = []
        self.scatter.clear()

        displayDetections = False
        displayDB_algo = True

        if displayDetections:
            x = self.displayPersonXData
            y = self.displayPersonYData
            colorMap = np.ones(len(x))*0.5
        if displayDB_algo and len(g_x_data) > 0:
            self.clusters = self._DBSCAN_ALGO() 

            # this data is the same length as the raw arrays, but with elements indicating 
            # which cluster they are in, (-1) would classify the point as noise. 
            # A sample cluster array would look like [0, -1, 1, 1, 1, 2, 2, 0, 1, 2, 0, -1];

            colorMap = self.clusters
            x = np.array(x) # absolute heretics
            y = np.array(y)
            x = x[self.clusters >= 0]
            y = y[self.clusters >= 0]
            x = x.tolist() # god forgive
            y = y.tolist()
            self._clusterCentroid()
            self.trackedTargets = self.tracker.update(np.column_stack((self.x_centroids, self.y_centroids)))
            # print(trackedTargets)
            self.addPastTracks(self.trackedTargets)
            '''
            I want to track the past locations of targets and subsequently graph this on our cute little window
            we could likely do a dictionary, (ordered dictionary???), to get something like pastTracks[1] = [[0, 1], [0, 2], [1, 2] .... ]
            im curious if .append works here. mayhaps we could 
            '''
            #print(self.pastTracks)

        for i in range(len(self.x_centroids)):
            spot_dic = {'pos': (self.x_centroids[i], self.y_centroids[i]), 'size': 0.4, 
                        'pen': {'color': 'black', 'width': 0},
                        'brush': (255, 255, 255),} # making the centroids white fr fr
            spots.append(spot_dic)
        
        for i in self.pastTracks.keys():
            # error about out of ranging values dunno how it could be
            for j in range(len(self.pastTracks[i])):
                spot_dic = {'pos': np.array(self.pastTracks[i][j]), 'size': 0.1, 
                        'pen': {'color': 'black', 'width': 0},
                        'brush': (np.clip((10)*j, 0, 255), 255, np.clip(50*i, 0, 255)),} # making the tracks green fr fr
                spots.append(spot_dic)
        
        for i in self.trackedTargets.keys():
            # print(trackedTargets[i])
            spot_dic = {'pos': (self.trackedTargets[i]), 'size': 0.35, 
                        'pen': {'color': 'black', 'width': 0},
                        'brush': (255, np.clip((75)*(np.abs(i)*(i>=0)), 0, 255), 255),} # making the tracked items pinkish fr fr
            spots.append(spot_dic)

        for i in range(len(x)):
            spot_dic = {'pos': (x[i], y[i]), 'size': 0.2, 
                        'pen': {'color': 'black', 'width': 0},
                        'brush': (150, np.clip((200)*(np.abs(colorMap[i])*(colorMap[i]>=0)), 0, 255), 
                                  np.clip((200)*(np.abs(colorMap[i])*(colorMap[i]<=0)), 0, 255))} # This color mapping function prolly needs help
            spots.append(spot_dic)
        
        self.scatter.addPoints(spots)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.plotItem.addItem(self.scatter)
        # self.plotDataItem.setData(x=x, y=y, symbolBrush=(255, 255, 255))

        # With the clutterRemoval value at 1, the window goes crazy due to autoscaling issues

    def _DBSCAN_ALGO(self):
        
        points = np.column_stack((g_x_data, g_y_data))
        points = np.array(points)

        db = DBSCAN(eps = 0.4, min_samples = 6)

        labels = db.fit_predict(points)

        # Number of clusters in labels, ignoring noise if present.
        # n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        # n_noise_ = list(labels).count(-1)

        # print("Estimated number of clusters: %d" % n_clusters_)
        # print("Estimated number of noise points: %d" % n_noise_)

        return labels
    
    def _clusterCentroid(self):
        """
        We need to find the centroid of the cluster data for further processing.
        first we should find a way to go through all data points and filter for the given cluster.
        Finding the centroid is easy, sum all the x values and divide it by the number of points in the cluster, 
        do something similar for y.
        """
        self.x_centroids = []
        self.y_centroids = []

        for i in range(max(self.clusters) + 1): # [0, 1, ... max(self.clusters)]
            xData = np.array(g_x_data)
            yData = np.array(g_y_data)
            clustersX = xData[self.clusters == i]
            clustersY = yData[self.clusters == i]

            xCentroid = np.sum(clustersX) / len(clustersX)
            yCentroid = np.sum(clustersY) / len(clustersX)

            self.x_centroids.append(xCentroid)
            self.y_centroids.append(yCentroid)
            '''
            print(i)
            print(clustersX)
            print(clustersY)
            print(self.x_centroids)
            print(self.y_centroids)
            print("\n")
            '''
        return 
    
    def addPastTracks(self, trackedTargets):

        for keyInd in self.prevKeys:
            if keyInd not in trackedTargets.keys():
                del self.pastTracks[keyInd]


        for i in trackedTargets.keys():
            self.pastTracks.setdefault(i, [])
            self.pastTracks[i].append(list(trackedTargets[i]))

        self.prevKeys = list(trackedTargets.keys())[:]

        for person_id, (x, y) in trackedTargets.items():
            if y < self.yBounds:
                self.peopleInside.discard(person_id)
            else:
                self.peopleInside.add(person_id)

        self.numPeople = len(self.peopleInside)
        print("people detected: ", self.peopleInside)

    def _framePointDistMatrix(self):
        matrixSize = len(g_x_data)

        pointDistMatrix = np.zeros(matrixSize)
        clumpDetMatrix = np.zeros(matrixSize)
        for i in range(matrixSize):
            for j in range(matrixSize): # hopefully these two match but I wont check this
                pointDistMatrix[i][j] = np.sqrt((g_x_data[i] - self.last_X_Data[j])**2 + 
                                                (g_y_data[i] - self.last_Y_Data[j])**2)
                if pointDistMatrix[i][j] < 0.25: # meters
                    clumpDetMatrix[i][j] = 1
                    # Now where is this clump back in real space?

        ##################################################
        #   S(1,1) S(1,2) ..... S(1,N)
        #   S(2,1) S(2,2)
        #   ......        .....
        #   S(N,1)              S(N,N)
        # Such that N = len(g_x_data) where S(n,n) shows the distance between two points.
        # Note that this SHOULD be reciprocal
        # If we now ask whether that distance is small then theres a clump indicating a person
        # now you should ask, "But what if our target has only one return?" possible, unlikely, and gross cause we need to get clumps 
        
        return pointDistMatrix


    # Funtion to update the data and display in the plot
    def update(self):
        
        dataOk = 0
        global detObj
        global g_x_data 
        global g_y_data 
        global g_z_data
        global g_v_data

        self.last_X_data = g_x_data
        self.last_Y_data = g_y_data
        self.last_z_data = g_z_data
        self.last_v_data = g_v_data
        
        # Read and parse the received data
        dataOk, frameNumber, detObj = readAndParseData14xx(Dataport, self.configParameters, self.DEBUG)
        if dataOk and len(detObj["x"]) > 0:
            #print(detObj)
            g_x_data = detObj["x"]
            g_y_data = detObj["y"]
            g_z_data = detObj["z"]
            g_v_data = detObj["v"]

        return dataOk, g_x_data, g_y_data, g_z_data, g_v_data


    def onNewData(self):
        
        # Update the data and check if the data is okay        
        dataOk,newx,newy,newz,newv = self.update()

        #if dataOk:
            # Store the current frame into frameData
            #frameData[currentIndex] = detObj
            #currentIndex += 1
        
        x = newx
        y = newy
        z = newz
        v = newv
        self.setData(x, y, z, v)
