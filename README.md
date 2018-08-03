# Leonard_Lab_Robots

## A Brief Overview
There are two separate hardware systems at play, connected via a ROS platform:
 - Vicon
 - Kobukis
 
The Vicon system is basically a set of cameras which records the positions and orientation of robots with respect to a well defined coordinate system. The Kobukis are vacuum-cleaner-like robots; they can move with a certain linear and angular speed. The Robot Operating System (ROS) binds the Vicon and the Kobukis together; The ROS-core reads Vicon data, and sends (publishes) commands to the kobukis.

## Understanding ROS
Understanding how ROS works is essential to understanding the system. I would recommend going through all the ROS tutorials thoroughly: https://docs.google.com/document/d/1FFweM-iAUHEGsGNKYE8pN4EZRdZCeedRtgar93bCYYc/edit?usp=sharing

## Starting Up the Vicon
The Vicon system broadcasts data in form of Twist messages to the ROS-core. A ROS package called "Vicon_bridge" handles communication with the vicon system. The "vicon_LL.launch" node publishes the vicon data to the "/vicon/" topics, from which the data can be accessed. 
  - Turn on Vicon cameras and fire up Vicon Nexus. You may need to calibrate the cameras if the calibration is off.
  - Select only the object you wish to use.
  - Launch the vicon node ``` roslaunch vicon_bridge vicon_LL.launch ```
  
  

