from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import os
import serial
import datetime

import BlynkLib
BLYNK_AUTH_TOKEN = 'QSrUCSyQ__HzhpFdDcBIXmuCScTkVt9M'
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)

# let shitter know the piss is there
import IWR6843_Read_Data_Python_MMWAVE_SDK_main.mmw_parse_script as ps
# This mmw_parse_script file has been changed kind of significantly

# WARNING: This takes ALOT of data and slows everything down signifcantly
# Usage: DEBUG[0] is print to terminal, DEBUG[1] prints to the csv
DEBUG = [False, True]
# The CSV system needs work: shits itself if a frame is failed
# Also im not sure if its logging everything or just the current frame??? - Likely why it wasnt used in the first place

if DEBUG[1] is True:
    import csv
    democsvfile = open('mmw_demo_output_' + str(datetime.datetime.now().strftime("%Y_%m_%d_T_%H_%M_%S"))
                        + '.csv', 'w', newline='') 
    demoOutputWriter = csv.writer(democsvfile, delimiter=',',
                                    quotechar=',', quoting=csv.QUOTE_NONE)                                    
    demoOutputWriter.writerow(["frame","DetObj#","x","y","z","v","snr","noise"])   

# TODO: Maybe make this var find the newest cfg file in the dir?
#       DONE!^
# "profile_" + year + month + day + "T" + UTC_HOUR + minute + second + millisecond + ".cfg"
dirList = os.listdir()
lastFile = ['profile', '0', '00', '00', '00', '00', '00', '00', 'cfg']
for i in dirList:
    if i.endswith(".cfg") and i.startswith("profile"):
        # Then this is a cfg file, and we should grab the last 12 chars
        f = (i.replace("T", "_").replace(".", "_")).split("_")

        for j in range(6):
            if int(f[j+1]) > int(lastFile[j+1]):
                configFileName = i
                lastFile = f
        
print(configFileName)

# configFileName = 'profile_2024_03_15T00_14_41_424.cfg' # This can hard override the shitter

# Maybe we should think about a way to detect this?
comSetupPort = 'COM7'
comDataPort = 'COM8'

##############################################
# TODO: Detection Team:
#       Come up with new graphics - its okish now, not my favorite but its ok - I'm happy with it now
#       filter out long lost const returns
#           The module might be able to to this with the remove static clutter real time tuning parameter 
#               - just need to find how to ask it to
#               - I fingered this out, using the clutterRemoval var in the cfg file you can enable it, something like...
#                       "clutterRemoval -1 1" works great but idk what the -1 is for. some frameIDX thing. type help in com port.
#               - I also have it running at 20Hz now, perhaps some thick averaging would be beneficial now.
#       PERSON DETECTION
#           RCS data? SNR of return?
#       GET 3D DATA VISUALIZED - 90% Done - Probably satisfactory, there is a selectable color var for point cloud data.
##############################################

##############################################
# TODO: IOT Team:
#           Send data to blynk???
#               Looked into a bit, their demos dont work for me :(
#           Recover data from blynk??? 
##############################################

def main():        
    # Configurate the serial port
    CLIport, Dataport = ps.serialConfig(configFileName, comSetupPort, comDataPort)
    
    # Get the configuration parameters from the configuration file
    global configParameters 
    configParameters = ps.parseConfigFile(configFileName)
    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False) # True seems to work as well
    
    win = ps.MyWidget()

    blynk.virtual_write(2, win.numPeople)

    win.colorVar = "v"
    win.DEBUG = DEBUG
    win.configParameters = configParameters # bandaided hehe
    win.show()
    win.resize(800*2,600*2) 
    win.raise_()
    app.exec_()
    CLIport.write(('sensorStop\n').encode())
    CLIport.close()
    Dataport.close()



if __name__ == "__main__":
    #exit()
    main()
