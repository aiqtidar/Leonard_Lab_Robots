#!/usr/bin/env python
# Software License Agreement (BSD License)

#YOU NEED TO
#   SET U THRESHOLD IN WAYPOINT COMMANDS()
#   SET k_g gain in read_u()

#Dependencies
import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray
from geometry_msgs.msg import Twist, TransformStamped
from kobuki_msgs.msg import BumperEvent
import numpy as np
import math
import time

#Variables
global n_robots
robot_names = ['104','103']
n_robots = len(robot_names)

robot_target = list() #List of robot target positions in twist
for i in range(0,n_robots):
    robot_target.append(Twist())

robot_locations = list() #List of robot target positions in twist
for i in range(0,n_robots):
    robot_locations.append(Twist())
global main_rate

main_rate = 10
last_velocity_command = list() #List of robot target positions in twist
for i in range(0,n_robots):
    last_velocity_command.append(Twist())

global reorient
color_103 = 0
color_104 = 0
global last_target_center_time
global last_target_circlingcontrol_time
global target_timeout
global beccontrol_default
global L
global s_star
global w
global y_hat
global start_time
global U
global beta

reorient = False

U = [0.5,0.6]
last_target_center_time = 0
last_target_circlingcontrol_time = 0
target_timeout = 1
circlingcontrol_default = 1

w = [[0],[0]]

L = [[1, -1],[-1, 1]]

beta = [0, 0]

#Define callbacks for all subscribers
'''
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
'''
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

#READ LIGHT INTENSITIES AND SET U---------------------------------------------------------------------------------------------------------
def callback_blue(data):
    global color_103
    color_103 = float(data.data)
    pass

def callback_green(data):
    global color_104
    color_104 = float(data.data)
    pass

def read_u():
    while not rospy.is_shutdown():
        global color_104
        global color_103
        global U

        #Subscribers
        rospy.Subscriber('/103/blue', Float32, callback_blue)
        rospy.Subscriber('/104/green', Float32, callback_green)


        #Control gains
        k_g = 0.00017
        k_b = 0.00023

        U = [float(color_103*k_b), float(color_104*k_g)]

    pass

#WAYPOINT COMMANDS GO HERE-----------------------------------------------------------------------------------------------------
def waypoint_commands(current_location_all, last_command_all, robot_i):
    #current_location: Twist
    #target_location: Twist
    #return: Twist giving velocity commands

    #Make local copies of variables
    current_location = current_location_all[robot_i]
    last_command = last_command_all[robot_i]

    velocity_command = Twist()
    global main_rate
    global n_robots
    global beccontrol_default
    global L
    global s_star
    global w
    global y_hat
    global start_time
    global U
    global beta

    

    #Control Law Settings
    v_linear_max = .35 #max linear velocity
    v_angular_max = 3 #max angular velocity
    a_linear_max = 0.25
    a_angular_max = 1.0

    #x-offset (all)
    x_pink = current_location_all[robot_names.index('103')].linear.x - 3.53
    #x_green = current_location_all[robot_names.index('105')].linear.x - 3.53
    x_orange = current_location_all[robot_names.index('104')].linear.x - 3.53
    x_vector = [[x_pink],[x_orange],[x_orange]]

    #Control gains
    kp = 5

    #y-offset (robot in question)
    if robot_i == robot_names.index('103'):
        current_location.linear.y = current_location.linear.y - 1.98
    #elif robot_i == robot_names.index('105'):
    #    current_location.linear.y = current_location.linear.y - 4.34
    else:
        current_location.linear.y = current_location.linear.y - 3.04

    #Consensus (change alpha)
    alpha = 0.5
    w = w + (-alpha*np.sign(np.dot(L,y_hat)))/main_rate
    y_hat = (np.dot(L,w) + x_vector)/main_rate

    #Calculate horizontal velocity
    if time.time() - start_time > s_star:
        epsilon = 0.01
        threshold = 2
        if robot_i == robot_names.index('103'):
            #U[robot_names.index('103')] = U[robot_names.index('103')] + epsilon*(threshold**2 - y_hat[robot_names.index('103')][0]**2)/(main_rate)
            DX = (-2*x_pink + U[robot_names.index('103')]*(math.tanh(x_orange)) + beta[robot_names.index('103')])/(main_rate)
            if abs(DX) > 0.05:
                error = -current_location.linear.y
                DY = (kp*error)/(main_rate)
                velocity_command.angular.z = 0
            else:
                DY = 0
                velocity_command.angular.z = 0
            velocity_command.linear.x = np.sign(DX)*math.sqrt(DX**2+DY**2)
        
        else:
            #U[robot_names.index('104')] = U[robot_names.index('104')] + epsilon*(threshold**2 - y_hat[robot_names.index('104')][0]**2)/(main_rate)
            DX = (-2*x_orange + U[robot_names.index('104')]*(math.tanh(x_pink)) + beta[robot_names.index('104')])/(main_rate)
            if abs(DX) > 0.05:
                error = -current_location.linear.y
                DY = (kp*error)/(main_rate)
                velocity_command.angular.z = 0
            else:
                DY = 0
                velocity_command.angular.z = 0
            velocity_command.linear.x = np.sign(DX)*math.sqrt(DX**2+DY**2)
            
    else:
        velocity_command.linear.x = 0
        velocity_command.angular.z = 0    

    #---------------End Control Law----------------------------

    #Limit Positive Acceleration
    if velocity_command.linear.x > last_command.linear.x + a_linear_max/float(main_rate):
        velocity_command.linear.x = last_command.linear.x + a_linear_max/float(main_rate)

    #Check max velocities
    velocity_command.linear.x = min(abs(velocity_command.linear.x), v_linear_max)*np.sign(velocity_command.linear.x)
    velocity_command.angular.z = min(abs(velocity_command.angular.z), v_angular_max)*np.sign(velocity_command.angular.z)

    return velocity_command



#------------------------------------------------------------------------------------------------------------------------------

#Main Code
def trials():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('trials', anonymous=True)
    #global variables
    global robot_target
    global robot_locations
    global main_rate
    global beccontrol_default
    global L
    global s_star
    global w
    global y_hat
    global start_time
    global U
    global beta
    global pub_u_value
    global color_103
    global color_104
    global n_robots
    global last_velocity_command

    #Subscribe to topics
    #rospy.Subscriber('/vicon/GREEN/GREEN', TransformStamped, callback_position_105)
    rospy.Subscriber('/vicon/ORANGE/ORANGE', TransformStamped, callback_position_104)
    rospy.Subscriber('/vicon/PINK/PINK', TransformStamped, callback_position_103)
    
    #Set up publishers
    pub_velocity_command = list()
    for i in range(0,n_robots):
        pub_velocity_command.append(rospy.Publisher('/'+robot_names[i]+'/mobile_base/commands/velocity', Twist, queue_size=10))

    pub_u_value = rospy.Publisher('/controller/value/u', Twist, queue_size=10)

    start_time = time.time()

    
    #All-to-all
    x_pink = robot_locations[robot_names.index('103')].linear.x
    #x_green = robot_locations[robot_names.index('105')].linear.x
    x_orange = robot_locations[robot_names.index('104')].linear.x
    s_star = (x_pink+x_orange)/(2*2)
    y_hat = [[x_pink],[x_orange],[x_orange]]

    # Keep node going until it is stopped
    rate = rospy.Rate(main_rate) # 10hz

    while not rospy.is_shutdown():

        #Read values of U from light intensity. U is automatically updated
        read_u()

        for i in range(0,n_robots):

            #Depending on u, check if a decision has been made
            if (U[i] > 1.5) or (U[i] < 1):
                #Set speeds to circle
                last_velocity_command[i].linear.x = 0.33
                last_velocity_command[i].angular.z = .2
                pub_velocity_command[i].publish(last_velocity_command[i])

            else:
                #output robot commands for solving equation
                last_velocity_command[i] = waypoint_commands(robot_locations, last_velocity_command, i);
                #Reorientation Code
                if ((robot_locations[i].angular.z > 0.4) and (robot_locations[i].angular.z < (2*math.pi - 0.4))) or ((robot_locations[i].angular.z > (math.pi + 0.4)) and (robot_locations[i].angular.z < (math.pi - 0.4))):
                    last_velocity_command[i].angular.z = float(0.5)*(float(-robot_locations[i].angular.z))
                pub_velocity_command[i].publish(last_velocity_command[i])
                
                U_pubber = Twist()
                U_pubber.linear.x = U[robot_names.index('103')]
                U_pubber.linear.y = U[robot_names.index('104')]
                #U_pubber.linear.z = U[robot_names.index('105')]
                pub_u_value.publish(U_pubber)
                
        rate.sleep()

#--------------------------------------------------------------------------------------------------------------------------------------------------

#Calls the main function
if __name__ == '__main__':
    try:
        trials()
    except rospy.ROSInterruptException: pass