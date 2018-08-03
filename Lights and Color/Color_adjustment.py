#!/usr/bin/env python

#Pink(103) reads blue
#Orange(105) reads green

import rospy
from std_msgs.msg import String, Bool, Float32

color_103 = 0
color_105 = 0


def callback_blue(data):
    global color_103
    color_103 = data.data


def callback_red(data):
    global color_105
    color_105 = data.data

def read_u():
    rospy.init_node('Color_Adjusting_Block', anonymous= True)

    while not rospy.is_shutdown():
        #Subscribers
        rospy.Subscriber('/105/blue', Float32, callback_blue)
        rospy.Subscriber('/105/red', Float32, callback_red)

        #Control floats
        k_r = 0.00045
        k_b = 0.00073


        U = [k_b*color_103, k_r*color_105]
        print(U)

        pass

if __name__== '__main__':
    read_u()
