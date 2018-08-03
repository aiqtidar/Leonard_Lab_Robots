#!/usr/bin/env python

from DmxPy import DmxPy
import time
import numpy as np
import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray

# print "dmxtest started"
global dmx
dmx = DmxPy('/dev/ttyUSB0')

def callback_light_change(data):
    global light_change_flag
    light_change_flag = data.data

#Main Code
def MAB_Field_Triggered():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    #global variable
    global dmx

    # Color Code:
    # Red 
    # Orange
    # Amber
    # Green
    # Cyan
    # Blue
    # Indigo
    # Intensity




    #dmx.setChannel(318,255)
    
    #Set Colors
    for i in range(0,9):
        dmx.setChannel(310+i, 255)
        print(310+i)
        time.sleep(2)
        dmx.render()
    

if __name__ == '__main__':
    MAB_Field_Triggered()