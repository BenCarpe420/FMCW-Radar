# FMCW Radar
 This repository houses the python scripts for interfacing with the TI IWR6843ISK for data processing. It works all over USB-Uart, and requires no extra hardware.

# Structure Notes
 FMCW.py in the main directory is the starting point for this program. It includes main and many other essential operations. In main, it opens COM ports, sets radar config parameters, and instantiates the graph widget. You might notice that this function does not loop forever, this is because the class function MyWidget.onNewData() is called every 50ms. From there it is processed and subsequently visualized.
 
 
 A lot has been changed from the original script. Graphics have been the largest thing, and theres a lot more to come. Theres a TODO section in FMCW.py outlining some of the things that need to be done. Person detection will likely be the largest hurdle. Not sure if we want to go with a machine learning system here or try to do conventional methods. 
