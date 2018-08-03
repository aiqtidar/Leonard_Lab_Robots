#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray, Float64
from geometry_msgs.msg import Twist, TransformStamped
from kobuki_msgs.msg import BumperEvent, Led
import numpy as np
import math
import time
import csv

#Global Variables
rospy.init_node('Coolest_Node_Ever',anonymous= True)

robot_names = ['103','105']
n_robots = len(robot_names)

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

pub_led1 = list()
for i in range(0,n_robots):
    pub_led1.append(rospy.Publisher('/'+robot_names[i]+'/mobile_base/commands/led1', Led, queue_size=10))

pub_led2 = list()
for i in range(0,n_robots):
    pub_led2.append(rospy.Publisher('/'+robot_names[i]+'/mobile_base/commands/led2', Led, queue_size=10))


pub_u_value = rospy.Publisher('/controller/value/u', Float64, queue_size=10)

U_location = Twist()
U = [0.5,0.6]

w = [[0],[0]]

L = [[1, -1],[-1, 1]]

beta = [0, 0]
color_103 = 0
color_105 = 0

beta_change = False
beta_color = 0

#COLOR READING BLOCK--------------------------------------------------------------------
def callback_blue(data):
    global color_103
    color_103 = float(data.data)


def callback_red(data):
    global color_105
    color_105 = float(data.data)

def callback_U(data):
    #Establsh global variables
    global U_location

    #Compute Euler Z angle
    q_r = data.transform.rotation.w
    q_i = data.transform.rotation.x
    q_j = data.transform.rotation.y
    q_k = data.transform.rotation.z
    t3 = +2.0 * (q_r * q_k + q_i * q_j)
    t4 = +1.0 - 2.0 * (q_j**2 + q_k**2)

    #Save data
    U_location.linear.x = data.transform.translation.x
    U_location.linear.y = data.transform.translation.y
    U_location.angular.z = math.atan2(t3, t4)


def read_u():

    global beta_color
    global beta
    global U

    #Subscribers
    rospy.Subscriber('/103/blue', Float32, callback_blue)
    rospy.Subscriber('/105/red', Float32, callback_red)
    rospy.Subscriber('/vicon/HandLeft/HandLeft', TransformStamped, callback_U)

    u_value = (U_location.linear.x/7)*2.75

    pub_u_value.publish(u_value)

    U = [u_value, u_value]

    #Control floats
    k_r = 0.00045
    k_b = 0.00073

    beta = [color_103*k_b-0.37, 0.20-color_105*k_r]

    if (float(beta[0])+ float(beta[1])) > 1:
        beta_color = 6
    elif (float(beta[0])+ float(beta[1])) < (-1):
        beta_color = 1
    else: beta_color = 0

    print("Beta =" + str(beta))

    pass

#Main----------------------------------------------------------------------------------
def main():

    rate = rospy.Rate(10)

    #global variables
    global robot_locations
    global main_rate
    global L
    global w
    global start_time
    global U
    global beta
    global pub_u_value
    global color_103
    global color_105
    global n_robots
    global last_velocity_command
    global pub_velocity_command
    global beta_color

    last_beta_color = 0

    #Subscribe to topics
    rospy.Subscriber('/vicon/LIME/LIME', TransformStamped, callback_position_105)
    rospy.Subscriber('/vicon/BERRY/BERRY', TransformStamped, callback_position_103)
    #All-to-all
    x_pink = robot_locations[robot_names.index('103')].linear.x - 3.53
    x_green = robot_locations[robot_names.index('105')].linear.x - 3.53
    x_locations = list()
    x_locations = [float(x_pink), float(x_green)]

    state = [2,2]

    #Determines if a state change based on location is allowed
    state_change = [True, True]

    # Keep node going until it is stopped
    rate = rospy.Rate(main_rate) # 10hz

    beta_change = False

    fake_robot_locations = list() #List of robot target positions in twist
    for i in range(0,n_robots):
        robot_locations.append(Twist())

    with open('Data.csv', 'wb') as csvfile:
        filewriter = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)

        filewriter.writerow(['Time', 'U', 'x_pink','x_green','Beta_Blue', 'Beta_Red'])

    start = time.time()

    while not rospy.is_shutdown():
        
        #All-to-all
        x_pink = robot_locations[robot_names.index('103')].linear.x - 3.53
        x_green = robot_locations[robot_names.index('105')].linear.x - 3.53
        s_star = (x_pink+x_green)/(2*2)
        y_hat = [[x_pink],[x_green]]


        fake_robot_locations = robot_locations

        #x-locations after offset
        x_locations = [float(x_pink), float(x_green)]

        last_beta_color = beta_color

        #Read values of U from light intensity. U is automatically updated
        read_u()



        if (U != [0,0]):

            print("-------------------------------------------------------------------------")
            print("U = " + str(U))
            print("States = " + str(state))
            print("Locations = " + str(x_locations))
            print("Last velocity command = " + "[" + str(last_velocity_command[0].linear.x) + "," + str(last_velocity_command[1].linear.x) + "]")
            print("Beta_change = " + str(beta_change))
            print("State Change = " + str(state_change))

            #Check for massive changes in Beta
            if (last_beta_color != beta_color):
                beta_change = True
            
            for i in range(0,n_robots):
                
                #Depending on u, decide which state to be in if this is the start
                if ((absol(x_locations[i]) > 1.9) and (U[i]>1)) and (beta_change == False):
                    #State to circle after decision is made
                    state[i] = 3
                    pub_led2[i].publish(1)

                elif ((U[i]>1) and (state_change[i] == True)) or ((beta_change == True)):
                    #State to decide
                    state[i] = 2
                    pub_led2[i].publish(1)

                elif ((U[i]<1) and (absol(x_locations[i]) > 1.4)):
                    state[i] = 2
                    state_change[i] = True
                    pub_led2[i].publish(3)

                elif ((U[i]<1) and (absol(x_locations[i]) < 0.4)):
                    #State to circle at center
                    state[i] = 1
                    state_change[i] = True
                    pub_led2[i].publish(3)

                #Cute block of code which takes care of osciallations    
                if state[i] == 2:
                    if (state[i] != state[abs(1-i)]):
                        if (state[abs(1-i)]) == 3:
                            fake_robot_locations[abs(1-i)].linear.x = np.sign(x_locations[i])*2 + 3.53
                        elif (state[abs(1-i)]) == 1:
                            fake_robot_locations[abs(1-i)].linear.x = 3.53

                #Reset Beta change with the right condition
                if (abs(x_locations[i]) < 0.2 ):
                    beta_change = False
                    #state_change[i] = False

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
                    last_velocity_command[i] = waypoint_commands(fake_robot_locations, last_velocity_command, i)

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
        
        #Write to file
        row = [(time.time()-start), U[0], (fake_robot_locations[0].linear.x - 3.53) , (fake_robot_locations[1].linear.x - 3.53), beta[0], beta[1]]

        with open('Data.csv', 'a') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(row)

        csvFile.close()

        #print(time.time()-start)
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
    global L
    global s_star
    global w
    global y_hat
    global start_time
    global U
    global beta


    #x-offset (all)
    x_pink = current_location_all[robot_names.index('103')].linear.x - 3.53
    x_green = current_location_all[robot_names.index('105')].linear.x - 3.53
    x_vector = [[x_pink],[x_green]]

    #Calculate horizontal velocity

    if robot_i == robot_names.index('103'):
        velocity_command.linear.x = (-1*x_pink + U[robot_names.index('103')]*(math.tanh(x_green)) + beta[robot_names.index('103')])/(main_rate)

    else:
        velocity_command.linear.x = (-1*x_green + U[robot_names.index('105')]*(math.tanh(x_pink)) + beta[robot_names.index('105')])/(main_rate)


    #---------------End Control Law----------------------------


    #------------------------Velocity gains-----------------------------
    
    '''
    #Smoothened Gains Block
    if (abs(velocity_command.linear.x) < 0.1):
        velocity_command.linear.x = 10*velocity_command.linear.x
    elif (abs(velocity_command.linear.x) < 0.5):
        velocity_command.linear.x = (np.log(velocity_command.linear.x))/5
    '''

    #Manual Gains Block
    if (abs(velocity_command.linear.x) < 0.01):
        velocity_command.linear.x = 10*velocity_command.linear.x
    elif (abs(velocity_command.linear.x) < 0.1):
        velocity_command.linear.x = 5*velocity_command.linear.x
    elif (abs(velocity_command.linear.x) < 0.5):
        velocity_command.linear.x = 1.5*velocity_command.linear.x
    #'''
    #---------------End Gains------------------------------------------

    return velocity_command


#Absolute function-----------------------------------------------------------------------------------------------------------------------------------
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

#------------------------------------------------------------------------------------------------------------------------------------------------------


if __name__=='__main__':
    main()
