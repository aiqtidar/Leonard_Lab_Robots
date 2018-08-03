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
robot_names = ['103','105']
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
color_105 = 0

#COLOR READING BLOCK--------------------------------------------------------------------

def callback_blue(data):
    global color_103
    color_103 = float(data.data)

    pass

def callback_red(data):
    global color_105
    color_105 = float(data.data)

    pass

def read_u():

    global U
    global beta
    #Subscribers
    rospy.Subscriber('/103/blue', Float32, callback_blue)
    rospy.Subscriber('/105/red', Float32, callback_red)


    #Control floats
    k_r = 0.00016
    k_b = 0.00025

    readBeta =True

    if ((color_103 + color_105) > 16000):
        U = [2, 2]
        readBeta = False
    else:
        U = [0.2, 0.2]
    
    if readBeta == True:
        beta = [color_103*k_b, -color_105*k_r]
    #U = [(color_103*k_b), (color_105*k_g)]

    #Changes to Beta Values



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
    global color_105
    global n_robots
    global last_velocity_command
    global pub_velocity_command


    #Subscribe to topics
    rospy.Subscriber('/vicon/LIME/LIME', TransformStamped, callback_position_105)
    rospy.Subscriber('/vicon/BERRY/BERRY', TransformStamped, callback_position_103)
    #All-to-all
    x_pink = robot_locations[robot_names.index('103')].linear.x - 3.53
    x_green = robot_locations[robot_names.index('105')].linear.x - 3.53
    x_locations = list()
    x_locations = [float(x_pink), float(x_green)]

    state = [0,0]

    #Determines if a state change based on location is allowed
    state_change = [True, True]

    # Keep node going until it is stopped
    rate = rospy.Rate(main_rate) # 10hz

    while not rospy.is_shutdown():
        #All-to-all
        x_pink = robot_locations[robot_names.index('103')].linear.x - 3.53
        x_green = robot_locations[robot_names.index('105')].linear.x - 3.53
        s_star = (x_pink+x_green)/(2*2)
        y_hat = [[x_pink],[x_green],]


        #x-locations after offset

        x_locations = [float(x_pink), float(x_green)]

        #Read values of U from light intensity. U is automatically updated
        read_u()
        print(U)
        print(state)
        print((x_locations))
        for i in range(0,n_robots):

            
            #Depending on u, decide which state to be in if this is the start
            
            if ((absol(x_locations[i]) > 1.9) and (U[i]>1)):
                #State to circle after decision is made
                state[i] = 3

            elif ((U[i]>1) and (state_change[i] == True)):
                #State to decide
                state[i] = 2

            elif ((U[i]<1) and (absol(x_locations[i]) > 1.4)):
                state[i] = 2
                state_change[i] = True

            elif ((U[i]<1) and (absol(x_locations[i]) < 0.4)):
                #State to circle at center
                state[i] = 1
                state_change[i] = True

            #Decision has not been made at the start
            if (state[i] == 1):
                #Circle the center

                #Set speeds to circle
                last_velocity_command[i].linear.x = 0.20
                last_velocity_command[i].angular.z = .7
                pub_velocity_command[i].publish(last_velocity_command[i])

            #Decision making
            elif (state[i] == 2):
                #output robot commands for solving equation
                last_velocity_command[i] = waypoint_commands(robot_locations, last_velocity_command, i)

                #Reorientation code for setting angular speed
                reorient(i)

                pub_velocity_command[i].publish(last_velocity_command[i])


            elif (state[i] == 3):
                #Circle the decision made

                #Set speeds to circle
                last_velocity_command[i].linear.x = -0.20
                last_velocity_command[i].angular.z = .7
                pub_velocity_command[i].publish(last_velocity_command[i])

                state_change[i] = False
                
                
            U_pubber = Twist()
            U_pubber.linear.x = U[robot_names.index('103')]
            U_pubber.linear.y = U[robot_names.index('105')]
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
    x_green = current_location_all[robot_names.index('105')].linear.x - 3.53
    x_vector = [[x_pink],[x_green],]

    #Control 
    kp = 5

    #Consensus (change alpha)
    alpha = 0.5
    w = w + (-alpha*np.sign(np.dot(L,y_hat)))/main_rate
    y_hat = (np.dot(L,w) + x_vector)/main_rate

    #Calculate horizontal velocity
    epsilon = 0.01
    threshold = 2
    if robot_i == robot_names.index('103'):
        velocity_command.linear.x = (-1*x_pink + U[robot_names.index('103')]*(math.tanh(x_green)) + beta[robot_names.index('103')])/(main_rate)

    else:
        velocity_command.linear.x = (-1*x_green + U[robot_names.index('105')]*(math.tanh(x_pink)) + beta[robot_names.index('105')])/(main_rate)


    #---------------End Control Law----------------------------

    #Set appropriate gains
    velocity_command.linear.x = 2*velocity_command.linear.x

    #Limit Positive Acceleration
    if velocity_command.linear.x > last_command.linear.x + a_linear_max/float(main_rate):
        velocity_command.linear.x = last_command.linear.x + a_linear_max/float(main_rate)

    #Check max velocities
    velocity_command.linear.x = min(absol(velocity_command.linear.x), v_linear_max)*np.sign(velocity_command.linear.x)
    velocity_command.angular.z = min(absol(velocity_command.angular.z), v_angular_max)*np.sign(velocity_command.angular.z)

    #Take into consideration which direction the robot is facing
    #if ((((current_location_all[robot_i].angular.z) > 0.8) and (velocity_command.linear.x>0)) or (((current_location_all[robot_i].angular.z) < 0.80) and (velocity_command.linear.x<0))):
    #    velocity_command.linear.x = -velocity_command.linear.x

    
    if (abs(velocity_command.linear.x) < 0.01):
        velocity_command.linear.x = 10*velocity_command.linear.x


    print("From waypoint_commands")
    print(velocity_command.linear.x)
    print(abs(current_location_all[robot_i].angular.z))
    print("-----")
    
    return velocity_command
#Function to move x_value in the x-direction--------------------------------------------------------------------------------------------------------
def move_x(x_value, i):
    cDistance = 0
    fDistance = x_value
    
    global pub_velocity_command
    global last_velocity_command

    while (absol(cDistance) < absol(fDistance)):
        time0 = time.time()    
        last_velocity_command[i].linear.x = -(cDistance - fDistance)
        if (last_velocity_command[i].linear.x < 0.1):
            last_velocity_command[i].linear.x = 0.1
        pub_velocity_command[i].publish(last_velocity_command[i])

        time1 = time.time()
        cDistance = cDistance + last_velocity_command[i].linear.x*(time1-time0)
        print(cDistance)

    pass
#Absolute function
def absol(x):
    return (np.sign(x)*x)
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

    last_velocity_command[robot_i].angular.z = 3*abs(last_velocity_command[robot_i].linear.x)*last_velocity_command[robot_i].angular.z

    pass

def reorient_new(robot_i):
    global last_velocity_command


    if (robot_locations[robot_i].angular.z < 0.01) and (robot_locations[robot_i].angular.z > -0.01): #Its in radians
        last_velocity_command[robot_i].angular.z = -0.5
    
    last_velocity_command[robot_i].angular.z = 3*abs(last_velocity_command[robot_i].linear.x)*last_velocity_command[robot_i].angular.z

    pass
#Callbacks for location storage-----------------------------------------------------------------------------------------------------------------------

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