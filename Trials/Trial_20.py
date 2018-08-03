#!/usr/bin/env python

#NO BETA CHANGE IN THIS VERSION

import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray, Float64
from geometry_msgs.msg import Twist, TransformStamped
from kobuki_msgs.msg import BumperEvent, Led
import numpy as np
from scipy.stats import hypsecant
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

pub_beta = list()
pub_beta.append(rospy.Publisher('/controller/value/red_beta', Float64, queue_size=10))
pub_beta.append(rospy.Publisher('/controller/value/blue_beta', Float64, queue_size=10))

pub_u_value = rospy.Publisher('/controller/value/u', Float64, queue_size=10)

U_location = Twist()
U = [0,0]

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

    beta_init = [color_103*k_b-0.37, 0.20-color_105*k_r]

    #Beta correction block
    r_off = np.sqrt((robot_locations[1].linear.x-3.53)*(robot_locations[1].linear.x-3.53) + (robot_locations[1].linear.y-3.04)*(robot_locations[1].linear.y-3.04))

    #print("from beta:" + str(r_off))

    red_beta_control = -(0.6-2.9*math.pi*hypsecant.pdf(r_off+0.15))
    blue_beta_control = (0.6-2.9*math.pi*hypsecant.pdf(r_off+0.15))

    beta = [blue_beta_control*beta_init[0], red_beta_control/beta_init[1]]
    
    '''
    Beta = 0.28 at Maximum Red
    Beta = 0.83 at 100 Red
    Beta = -1.89 at 0 Red

    '''
    
    #Set Beta values for Red
    if (beta[1]>0) and (beta[1]<0.4):
        beta[1] = -0.5
    elif (beta[1] > 0.4):
        beta[1] = 0
    else: beta[1] = 0.5
    

    '''
    Beta = 1.14 for max blue
    Beta = 0.41 for 100 blue
    Beta = -0.08 for zero blue
    '''

    #Set Beta Values
    if beta[0]>0.7:
        beta[0] = 0.5
    elif beta[0]>0.1:
        beta[0] = 0
    else: beta[0] = -0.5
    


    if (float(beta[0])+ float(beta[1])) > 1:
        beta_color = 6
    elif (float(beta[0])+ float(beta[1])) < (-1):
        beta_color = 1
    else: beta_color = 0

    #beta = [1, 1]
    print("Beta =" + str(beta))

    #Publish Betas
    pub_beta[0].publish(beta[1])
    pub_beta[1].publish(beta[0])

    pass

#Main----------------------------------------------------------------------------------
def main():

    rate = rospy.Rate(10)

    #global variables
    global robot_locations
    global main_rate
    global L
    global w
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
    angular_sign = [1,1]
    circle_direction = True

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
    rad_start = time.time()

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

        #U = [0,0]

        if (U != [0,0]):

            print("-------------------------------------------------------------------------")
            print("U = " + str(U))
            print("States = " + str(state))
            print("Locations = " + str(x_locations))
            print("Last velocity command = " + "[" + str(last_velocity_command[0].linear.x) + "," + str(last_velocity_command[1].linear.x) + "]")
            print("State Change = " + str(state_change))


            
            for i in range(0,n_robots):

                #Sets the direction to circle depending on the robot's y-location
                if circle_direction == True:
                    if ((robot_locations[i].linear.y < 3.5) and (x_locations[i] < 0)) or ((robot_locations[i].linear.y > 3.5) and (x_locations[i] > 0)):
                        angular_sign[i] = -1
                    else:
                        angular_sign[i] = 1
                        
                #Depending on u, decide which state to be in if this is the start
                if ((absol(x_locations[i]) > 1.9) and (U[i]>1)):
                    #State to circle after decision is made
                    state[i] = 3
                    pub_led2[i].publish(1)

                elif ((U[i]>1) and (state_change[i] == True)):
                    #State to decide
                    state[i] = 2
                    pub_led2[i].publish(1)

                elif ((U[i]<1) and (absol(x_locations[i]) > 1.4)):
                    state[i] = 2
                    state_change[i] = True
                    pub_led2[i].publish(3)

                elif ((U[i]<1) and (absol(x_locations[i]) < 0.31)):
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


                #Decision has not been made at the start
                if (state[i] == 1):
                    circle_direction = False
                    #Circle the center
                    if (abs(rad_start - time.time()) < 3):
                        last_velocity_command[i].linear.x = 0.25
                        last_velocity_command[i].angular.z = np.sign(angular_sign[i])*.6

                    else: 
                        #Set speeds to circle
                        last_velocity_command[i].linear.x = 0.25
                        last_velocity_command[i].angular.z = np.sign(angular_sign[i])*.6

                    pub_velocity_command[i].publish(last_velocity_command[i])

                #Decision making
                elif (state[i] == 2):
                    circle_direction = True
                    #output robot commands for solving equation
                    last_velocity_command[i] = waypoint_commands(fake_robot_locations, last_velocity_command, i)

                    if (state[abs(1-i)] == 1) and (U[i] < 1) :
                        last_velocity_command[i].linear.x = -x_locations[i]

                    if last_velocity_command[i].linear.x > 0.8:
                        last_velocity_command[i].linear.x = 0.7

                    #Reorientation code for setting angular speed
                    reorient(i)

                    pub_velocity_command[i].publish(last_velocity_command[i])


                elif (state[i] == 3):
                    circle_direction = False
                    #Circle the decision made

                    #Set speeds to circle
                    last_velocity_command[i].linear.x = 0.20
                    last_velocity_command[i].angular.z = np.sign(angular_sign[i])*.7
                    pub_velocity_command[i].publish(last_velocity_command[i])

                    state_change[i] = False

                if (state[i] != 1):
                    rad_start = time.time()
        
        #Write to file
        row = [(time.time()-start), U[0], (fake_robot_locations[0].linear.x - 3.53) , (fake_robot_locations[1].linear.x - 3.53), beta[0], beta[1]]

        with open('Data_Run_18.csv', 'a') as csvFile:
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
        velocity_command.linear.x = (np.log(velocity_command.linear.x))/15
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

    last_velocity_command[robot_i].angular.z = 5*abs(last_velocity_command[robot_i].linear.x)*last_velocity_command[robot_i].angular.z

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
