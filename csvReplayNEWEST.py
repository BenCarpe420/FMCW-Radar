from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import csv
import warnings
import numpy as np

from sklearn import metrics
from sklearn.cluster import DBSCAN
#from dataclasses import dataclass
#from typing import List
import sys

import BlynkLib
BLYNK_AUTH_TOKEN = 'QSrUCSyQ__HzhpFdDcBIXmuCScTkVt9M'
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)

file = "mmw_demo_output_2024_04_11_T_15_24_13.csv"
#file = "mmw_demo_output_2024_03_28_T_09_23_09.csv"

speed = 0.4 # times the normal recorded speed of 20Hz

g_x_data = [] # All the x data from the points in the current frame
g_y_data = [] # All the y data from the points in the current frame
g_z_data = [] # "
g_v_data = [] # "

BLINKEDCOUNTER = 0

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

        # self.deletedPeople = list()
        
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
                    lastLocation = self.objects[objectID]
                    #deletedPerson = PersonIdentifier(LastPersonId=objectID, LastLocation=lastLocation)
                    #print(lastLocation)
                    if lastLocation[1] > 2:
                        global BLINKEDCOUNTER
                        BLINKEDCOUNTER+=1
                        # print(BLINKEDCOUNTER)
                    #self.deletedPeople.append(deletedPerson)
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
            # print(f'Object Centroids: {objectCentroids}')
            # print(f'Input Centroids: {inputCentroids}')
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
        # print(self.objects)
        return self.objects

@dataclass
class PersonIdentifier:
    LastPersonId: int
    LastLocation: List


class MyWidget(pg.GraphicsLayoutWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(round(50/speed))) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)

        self.DEBUG = False
        self.colorVar = "v" # "v" for velocity and "z" for elevation z
        self.currentFrame = 0

        self.last_X_Data = []
        self.last_Y_Data = []
        self.last_z_Data = []
        self.last_v_Data = []

        self.tracker = CentroidTracker()
        self.pastTracks = OrderedDict([(0,0)])
        self.prevKeys = OrderedDict([(0,0)]).keys() 

        self.x_centroids = []
        self.y_centroids = []

        # self.displayPersonXData = []
        # self.displayPersonYData = []

        self.plotItem = self.addPlot(title="Point Cloud Data")
    
        self.plotItem.setXRange(-4, 4, padding=None, update=True)
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
        if displayDB_algo:
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
            trackedTargets = self.tracker.update(np.column_stack((self.x_centroids, self.y_centroids)))
            # print(trackedTargets)
            self.addPastTracks(trackedTargets)
            '''
            I want to track the past locations of targets and subsequently graph this on our cute little window
            we could likely do a dictionary, (ordered dictionary???), to get something like pastTracks[1] = [[0, 1], [0, 2], [1, 2] .... ]
            im curious if .append works here. mayhaps we could 
            '''
            # print(self.pastTracks)

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
        
        for i in trackedTargets.keys():
            # print(trackedTargets[i])
            spot_dic = {'pos': (trackedTargets[i]), 'size': 0.35, 
                        'pen': {'color': 'black', 'width': 0},
                        'brush': (255, np.clip((75)*(np.abs(i)*(i>=0)), 0, 255), 255),} # making the tracked items pinkish fr fr
            spots.append(spot_dic)

        for i in range(len(x)):
            spot_dic = {'pos': (x[i], y[i]), 'size': 0.2, 
                        'pen': {'color': 'black', 'width': 0},
                        'brush': (150, np.clip((200)*(np.abs(colorMap[i])*(colorMap[i]>=0)), 0, 255), 
                                  np.clip((200)*(np.abs(colorMap[i])*(colorMap[i]<=0)), 0, 255))} # making the dots colorful fr fr
            spots.append(spot_dic)

        self.scatter.addPoints(spots)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.plotItem.addItem(self.scatter)


    def _DBSCAN_ALGO(self):
        points = np.column_stack((g_x_data, g_y_data))
        points = np.array(points)

        db = DBSCAN(eps = 0.4, min_samples = 6)

        labels = db.fit_predict(points)

        # Number of clusters in labels, ignoring noise if present.
        #n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        #n_noise_ = list(labels).count(-1)

        #print("Estimated number of clusters: %d" % n_clusters_)
        #print("Estimated number of noise points: %d" % n_noise_)

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
    

#    def personCounter(self, trackedTargets):
#        self.numPeople = 0
#        print(trackedTargets)




    def addPastTracks(self, trackedTargets):

        for keyInd in self.prevKeys:
            if keyInd not in trackedTargets.keys():
                del self.pastTracks[keyInd]


        for i in trackedTargets.keys():
            self.pastTracks.setdefault(i, [])
            self.pastTracks[i].append(list(trackedTargets[i]))

        self.prevKeys = list(trackedTargets.keys())[:]

        
        # print(self.pastTracks)
        for person_id, (x, y) in trackedTargets.items():

            if y > self.yBounds:
                
                if len(self.pastTracks[person_id]) < 1:
                    global BLINKEDCOUNTER
                    BLINKEDCOUNTER = max(0,BLINKEDCOUNTER-1)
                    #print(BLINKEDCOUNTER)
                self.peopleInside.add(person_id)
            else:
                self.peopleInside.discard(person_id)

        # numPeople = len(self.peopleInside) - BLINKEDCOUNTER
        numPeople = min(len(self.peopleInside), len(trackedTargets)) - BLINKEDCOUNTER

        #print(self.peopleInside)
        print(f'Estimated number of people in room: {numPeople}')
        # print(trackedTargets)
        blynk.virtual_write(2, numPeople)



    def _pointDistMatrix(self):
        matrixSizeX = int(len(g_x_data))
        matrixSizeY = int(len(self.last_X_Data))

        # print(str(self.currentFrame) + " " + str(matrixSizeX) + " " + str(matrixSizeY))

        pointDistMatrix = np.zeros((matrixSizeX, matrixSizeY))
        sameTarget = np.zeros((matrixSizeX, matrixSizeY))

        self.displayPersonXData = []
        self.displayPersonYData = []

        for i in range(matrixSizeX):
            for j in range(matrixSizeY): # hopefully these two match but I wont check this
                pointDistMatrix[i][j] = np.sqrt((g_x_data[i] - self.last_X_Data[j])**2 + 
                                                (g_y_data[i] - self.last_Y_Data[j])**2)
                if pointDistMatrix[i][j] < 0.1: # meters
                    self.displayPersonXData.append(g_x_data[i])
                    self.displayPersonYData.append(g_y_data[i])
                    sameTarget[i][j] = 1
                    # Now what

        return sameTarget
                    

    # Funtion to update the data and display in the plot
    def update(self):
        
        dataOk = 0
        global detObj
        global g_x_data 
        global g_y_data 
        global g_z_data
        global g_v_data

        self.last_X_Data = g_x_data
        self.last_Y_Data = g_y_data
        self.last_z_Data = g_z_data
        self.last_v_Data = g_v_data

        if(self.currentFrame < numOfFrames) :
            # Read and parse the received data
            detObj = output[self.currentFrame]
            self.currentFrame = self.currentFrame + 1

        else:
            print("End of File")
            sys.exit()


        if len(detObj["x"]) > 0:
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
        

def readCSVData(csvfile):
    csvReader = csv.reader(csvfile, delimiter=',', quotechar=' ', quoting=csv.QUOTE_NONE)
    frameNumber = 0
    xarray = []
    yarray = []
    zarray = []
    varray = []
    snrarray = []
    noisearray = []
    detobjnumarray = []

    output = []
    
    firstRead = 1
    for row in csvReader:
        # if not cell.isnumeric():
        #     continue
        if firstRead:
            firstRead = 0
            continue
        if row == [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ']:
            # New Frame!
            output.append({"x": xarray, "y": yarray, "z": zarray, "v": varray, "snr": 
                           snrarray, "noise": noisearray})  
            
            detobjnumarray = []
            xarray = []
            yarray = []
            zarray = []
            varray = []
            snrarray = []
            noisearray = []

            frameNumber = frameNumber + 1

        else:
            detobjnumarray.append(float(row[1]))
            xarray.append(float(row[2]))
            yarray.append(float(row[3]))
            zarray.append(float(row[4]))
            varray.append(float(row[5]))
            snrarray.append(float(row[6]))
            noisearray.append(float(row[7]))

        # print(str(row) +  " " + str(frameNumber) + " " + str(type(row)))
    
    # print(output[1])
    # print(output[2])

    return output, frameNumber


output = []
numOfFrames = 0
def main():

    app = QtWidgets.QApplication(sys.argv)

    view = QtWidgets.QGraphicsView()
    view.setRenderHint(QtGui.QPainter.Antialiasing)

    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False) # True seems to work as well
    
    global output
    global numOfFrames
    with open(file, newline='') as f:
        output, numOfFrames = readCSVData(f)

    
    win = MyWidget()
    win.colorVar = "v"
    # win.DEBUG = DEBUG
    # win.configParameters = configParameters # bandaided hehe
    win.show()
    win.resize(800,600) 
    win.raise_()
    app.exec_()



if __name__ == "__main__":
    #exit()
    main()
