#!/usr/bin/env python

from DmxPy import DmxPy

# print "dmxtest started"
global dmx
dmx = DmxPy('/dev/ttyUSB0')


#Main Code
def Main():

    #globals
    global dmx

    # Color Code:
    # 1. Red 
    # 2. Orange
    # 3. Amber
    # 4. Green
    # 5. Cyan
    # 6. Blue
    # 7. Indigo
    # 8. Intensity

    #Channels ending in 8 are intensities
    #Channels ending in 1 are red
    #Channels ending in 6 are blue

    RED = 0
    BLUE = 255

    INTENSITY = 0

    #Right Side
    dmx.setChannel(308,INTENSITY)

    dmx.setChannel(306,BLUE)
    dmx.setChannel(301,RED)


    #Left Side
    dmx.setChannel(318,INTENSITY)

    dmx.setChannel(316,BLUE)
    dmx.setChannel(311,RED)
    
    #Center
    dmx.setChannel(8,INTENSITY)

    dmx.setChannel(006,BLUE) 
    dmx.setChannel(001,RED) 

    dmx.render()

if __name__ == '__main__':
    Main()