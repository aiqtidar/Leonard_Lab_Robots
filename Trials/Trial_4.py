#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray
from geometry_msgs.msg import Twist, TransformStamped
from kobuki_msgs.msg import BumperEvent
import numpy as np
import math
import time

#DEFINE GLOBALS----------------------------------------------------------------
#Variables
robot_names = ['104','103']
n_robots = len(robot_names)

robot_target = list() #List of robot target positions in twist
for i in range(0,n_robots):
    robot_target.append(Twist())

robot_locations = list() #List of robot target positions in twist
for i in range(0,n_robots):
    robot_locations.append(Twist())

main_rate = 10

last_velocity_command = list() #List of robot target positions in twist
for i in range(0,n_robots):
    last_velocity_command.append(Twist())



#Set up publishers
pub_velocity_command = list()
for i in range(0,n_robots):
    pub_velocity_command.append(rospy.Publisher('/'+robot_names[i]+'/mobile_base/commands/velocity', Twist, queue_size=10))
pub_u_value = rospy.Publisher('/controller/value/u', Twist, queue_size=10)


U = [0.5,0.6]
last_target_center_time = 0
last_target_circlingcontrol_time = 0
target_timeout = 1
circlingcontrol_default = 1

w = [[0],[0]]

L = [[1, -1],[-1, 1]]

beta = [0, 0]
color_103 = 0
color_104 = 0

#COLOR READING BLOCK--------------------------------------------------------------------

def callback_blue(data):
    global color_103
    color_103 = float(data.data)

    pass

def callback_green(data):
    global color_104
    color_104 = float(data.data)

    pass

def read_u():

    global U
    #Subscribers
    rospy.Subscriber('/103/blue', Float32, callback_blue)
    rospy.Subscriber('/104/green', Float32, callback_green)


    #Control float(0.5)s
    k_g = 0.00023
    k_b = 0.00033



    U = [(color_103*k_b), (color_104*k_g)]
    pass

#Main----------------------------------------------------------------------------------
def main():
    rospy.init_node('Node',anonymous= True)
    rate = rospy.Rate(10)


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
    global pub_velocity_command


    #Subscribe to topics
    rospy.Subscriber('/vicon/ORANGE/ORANGE', TransformStamped, callback_position_104)
    rospy.Subscriber('/vicon/PINK/PINK', TransformStamped, callback_position_103)

    move = [False, False]
    decision = False

    start_time = time.time()

    # Keep node going until it is stopped
    rate = rospy.Rate(main_rate) # 10hz

    while not rospy.is_shutdown():
        #All-to-all
        x_pink = robot_locations[robot_names.index('103')].linear.x - 3.53
        x_orange = robot_locations[robot_names.index('104')].linear.x - 3.53
        s_star = (x_pink+x_orange)/(2*2)
        y_hat = [[x_pink],[x_orange],]


        #x-locations after offset
        x_locations = list()
        x_locations = [x_pink, x_orange]

        #Read values of U from light intensity. U is automatically updated
        read_u()
        print(U)
        for i in range(0,n_robots):



            #Depending on u, check if a decision has been made
            if (U[i] < 1):
                print(x_locations[i])

            #Move x-direction
            if move[i] == False:
                move_x(-.33/.5, i)
                move[i] = True

                #Set speeds to circle
                last_velocity_command[i].linear.x = 0.33
                last_velocity_command[i].angular.z = .5
                pub_velocity_command[i].publish(last_velocity_command[i])

            elif (abs(x_locations[i]) > 1.5):
                print(x_locations[i])

                #Move x-direction
                move_x(.33/.5, i)

                #Set speeds to circle
                last_velocity_command[i].linear.x = 0.33
                last_velocity_command[i].angular.z = .5
                pub_velocity_command[i].publish(last_velocity_command[i])

            else:
                print(x_locations[i])
                #output robot commands for solving equation
                last_velocity_command[i] = waypoint_commands(robot_locations, last_velocity_command, i);

                #Reorientation code for setting angular speed
                reorient(i)
                pub_velocity_command[i].publish(last_velocity_command[i])
                
            U_pubber = Twist()
            U_pubber.linear.x = U[robot_names.index('103')]
            U_pubber.linear.y = U[robot_names.index('104')]
            pub_u_value.publish(U_pubber)
        rate.sleep()
pass


#WAYPOINT COMMANDS GO HERE-----------------------------------------------------------------------------------------------------
def waypoint_commands(current_location_all, last_command_all, robot_i):


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
    x_orange = current_location_all[robot_names.index('104')].linear.x - 3.53
    x_vector = [[x_pink],[x_orange],]

    #Control 
    kp = 5

    #y-offset (robot in question)
    if robot_i == robot_names.index('103'):
        current_location.linear.y = current_location.linear.y - 1.98
    else:
        current_location.linear.y = current_location.linear.y - 3.04

    #Consensus (change alpha)
    alpha = 0.5
    w = w + (-alpha*np.sign(np.dot(L,y_hat)))/main_rate
    y_hat = (np.dot(L,w) + x_vector)/main_rate

    #Calculate horizontal velocity
    epsilon = 0.01
    threshold = 2
    if robot_i == robot_names.index('103'):
        velocity_command.linear.x = (-1*x_pink + U[robot_names.index('103')]*(math.tanh(x_orange)) + beta[robot_names.index('103')])/(main_rate)

    else:
        velocity_command.linear.x = (-1*x_orange + U[robot_names.index('104')]*(math.tanh(x_pink)) + beta[robot_names.index('104')])/(main_rate)


    #---------------End Control Law----------------------------

    #Limit Positive Acceleration
    if velocity_command.linear.x > last_command.linear.x + a_linear_max/float(main_rate):
        velocity_command.linear.x = last_command.linear.x + a_linear_max/float(main_rate)

    #Check max velocities
    velocity_command.linear.x = min(abs(velocity_command.linear.x), v_linear_max)*np.sign(velocity_command.linear.x)
    velocity_command.angular.z = min(abs(velocity_command.angular.z), v_angular_max)*np.sign(velocity_command.angular.z)

    #Set appropriate gains
    velocity_command.linear.x = 2*velocity_command.linear.x

    return velocity_command
#Function to move x_value in the x-direction--------------------------------------------------------------------------------------------------------
def move_x(x_value, i):
    cDistance = 0
    fDistance = x_value
    
    global pub_velocity_command
    global last_velocity_command

    while (abs(cDistance) < abs(fDistance)):
        time0 = time.time()    
        last_velocity_command[i].linear.x = -(cDistance - fDistance)
        if (last_velocity_command[i].linear.x < 0.1):
            last_velocity_command[i].linear.x = 0.1
        pub_velocity_command[i].publish(last_velocity_command[i])

        time1 = time.time()
        cDistance = cDistance + last_velocity_command[i].linear.x*(time1-time0)
        print(cDistance)

    pass
#Reorientation code----------------------------------------------------------------------------------------------------------------------------------
def reorient(robot_i):
    global last_velocity_command


    if (robot_locations[robot_i].angular.z < (math.pi/2)): #Its in radians
        last_velocity_command[robot_i].angular.z = -float(0.5)*(float(robot_locations[robot_i].angular.z))
    elif (robot_locations[robot_i].angular.z > (math.pi/2)):
        last_velocity_command[robot_i].angular.z = float(0.5)*(float(robot_locations[robot_i].angular.z))
    elif (robot_locations[robot_i].angular.z > (-math.pi/2)): #Its in radians
        last_velocity_command[robot_i].angular.z = -float(0.5)*(float(robot_locations[robot_i].angular.z))
    elif (robot_locations[robot_i].angular.z < (-math.pi/2)):
        last_velocity_command[robot_i].angular.z = float(0.5)*(float(robot_locations[robot_i].angular.z))

    pass

#Callbacks for location storage-----------------------------------------------------------------------------------------------------------------------

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

if __name__=='__main__':
    main()