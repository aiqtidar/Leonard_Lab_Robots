#!/usr/bin/env python

#Dependencies
import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray
from geometry_msgs.msg import Twist, TransformStamped
from kobuki_msgs.msg import BumperEvent
import numpy as np
import math
import time
import csv

beta = [0,0]
color_103 = 0
color_105 = 0

global n_robots
robot_names = ['105','104','103']
n_robots = len(robot_names)

robot_locations = list() #List of robot target positions in twist
for i in range(0,n_robots):
    robot_locations.append(Twist())

#COLOR READING BLOCK--------------------------------------------------------------------
def callback_blue(data):
    global color_103
    color_103 = float(data.data)


def callback_red(data):
    global color_105
    color_105 = float(data.data)

def read_u():
    global beta


    #Subscribers
    rospy.Subscriber('/103/blue', Float32, callback_blue)
    rospy.Subscriber('/105/red', Float32, callback_red)

    #Control floats
    k_r = 0.00045
    k_b = 0.00073

    beta = [color_103*k_b-0.37, 0.20-color_105*k_r]

    #Write to file
    row = [(time.time()-start), (robot_locations[0].linear.y - 3.04) , beta[1]]

    with open('Data_Light_2.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)

    csvFile.close()


    pass
#Define callbacks for all subscribers
def callback_position_105(data):
    #Establsh global variables
    global robot_locations

    #Compute Euler Z angle
    q_r = data.transform.rotation.w
    q_i = data.transform.rotation.x
    q_j = data.transform.rotation.y
    q_k = data.transform.rotation.z
    t3 = +2.0 * (q_r * q_k + q_i * q_j)
    t4 = +1.0 - 2.0 * (q_j**2 + q_k**2)

    #Save data
    robot_locations[robot_names.index('105')].linear.x = data.transform.translation.x
    robot_locations[robot_names.index('105')].linear.y = data.transform.translation.y
    robot_locations[robot_names.index('105')].angular.z = math.atan2(t3, t4)

def callback_position_104(data):
    #Establsh global variables
    global robot_locations

    #Compute Euler Z angle
    q_r = data.transform.rotation.w
    q_i = data.transform.rotation.x
    q_j = data.transform.rotation.y
    q_k = data.transform.rotation.z
    t3 = +2.0 * (q_r * q_k + q_i * q_j)
    t4 = +1.0 - 2.0 * (q_j**2 + q_k**2)

    #Save data
    robot_locations[robot_names.index('104')].linear.x = data.transform.translation.x
    robot_locations[robot_names.index('104')].linear.y = data.transform.translation.y
    robot_locations[robot_names.index('104')].angular.z = math.atan2(t3, t4)

def callback_position_103(data):
    #Establsh global variables
    global robot_locations

    #Compute Euler Z angle
    q_r = data.transform.rotation.w
    q_i = data.transform.rotation.x
    q_j = data.transform.rotation.y
    q_k = data.transform.rotation.z
    t3 = +2.0 * (q_r * q_k + q_i * q_j)
    t4 = +1.0 - 2.0 * (q_j**2 + q_k**2)

    #Save data
    robot_locations[robot_names.index('103')].linear.x = data.transform.translation.x
    robot_locations[robot_names.index('103')].linear.y = data.transform.translation.y
    robot_locations[robot_names.index('103')].angular.z = math.atan2(t3, t4)


#Initialize node
rospy.init_node('move', anonymous = True)

#Subscribers
rospy.Subscriber('/vicon/LIME/LIME', TransformStamped, callback_position_105)

#Publishers
tPublisherG = rospy.Publisher('/105/mobile_base/commands/velocity', Twist, queue_size = 10)

#Define some variables
main_rate = 10

start = time.time()

#Main Code
def Go():

    #What to publish?
    msg = Twist()
    msg.linear.x = 0.3
    tPublisherG.publish(msg)
    
    #Keep Node Running
    rate = rospy.Rate(main_rate)
    
    while not rospy.is_shutdown():
        read_u()
        tPublisherG.publish(msg)


        
#Calls the main function
if __name__ == '__main__':
    try:
        Go()
    except rospy.ROSInterruptException: pass
'''
Results:
An y = x^2 curve which peaks at x = -0.200. Some data points:
(-1.7309, -0.6379)
(1.3551, -0.5538)
(0.2975, -2.0613)
(-0.8320, -1.8039)