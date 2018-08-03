#!/usr/bin/env python

#ROOM IS APPROXIMATELY 7.00 Metres
#Main transforms to read_u()

import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray, Float64
from geometry_msgs.msg import Twist, TransformStamped
from kobuki_msgs.msg import BumperEvent, Led
import numpy as np
import math
import time 

rospy.init_node('Hello_LEDs')
n_robots = len(robot_names)



#Set up publishers
pub_led1 = list()
for i in range(0,n_robots):
    pub_led1.append(rospy.Publisher('/'+robot_names[i]+'/mobile_base/commands/led1', Led, queue_size=10))

pub_led2 = list()
for i in range(0,n_robots):
    pub_led2.append(rospy.Publisher('/'+robot_names[i]+'/mobile_base/commands/led2', Led, queue_size=10))



def main()
    #rostopic pub /mobile_base/commands/led1 kobuki_msgs/Led "value: 1
    while not rospy.is_shutdown():
        #Publisher
        publisher_103 = rospy.Publisher('/105/mobile_base/commands/led1', Led, queue_size=10)
        publisher.publish(3)
        pass

if __name__ == '__main__':
    main()