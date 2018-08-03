# Leonard_Lab_Robots

## A Brief Overview
There are two separate hardware systems at play, connected via a ROS platform:
 - Vicon
 - Kobukis
 
The Vicon system is basically a set of cameras which records the positions and orientation of robots with respect to a well defined coordinate system. The Kobukis are vacuum-cleaner-like robots; they can move with a certain linear and angular speed. The Robot Operating System (ROS) binds the Vicon and the Kobukis togethe. The ROS-core reads Vicon data, and sends (publishes) commands to the kobukis.

## Understanding ROS
Understanding how ROS works is essential to understanding the system. I would recommend going through all the ROS tutorials thoroughly: https://docs.google.com/document/d/1FFweM-iAUHEGsGNKYE8pN4EZRdZCeedRtgar93bCYYc/edit?usp=sharing

## Starting Up the Vicon
The Vicon system broadcasts data in form of Twist messages to the ROS-core. A ROS package called "Vicon_bridge" handles communication with the vicon system. The "vicon_LL.launch" node publishes the vicon data to the "/vicon/" topics, from which the data can be accessed. 
  - Turn on Vicon cameras and fire up Vicon Nexus. You may need to calibrate the cameras if the calibration is off.
  - Select only the object you wish to use.
  - Launch the vicon node ``` roslaunch vicon_bridge vicon_LL.launch ```

Remember to ```catkin_make``` and ```bash devel/setup.bash```
  
## Setting up the Lighting
The three spotlights are connected to the Roscore computer via DMX. Each spotlight has a set of channels which accept data values from 0 to 255 corresponding to the light intensity of that channel. Th spotlights and their channels (as viewed from the side of the Roscore computer) are as follows.

- Center Spotlight: Channels starting from 001
- Right Spotlight: Channels starting from 301
- Left Spotlight: Channels starting from 311

The spotlights are connected in series, and can be lighted using python code. It is run via the commandline; try ```python /Lights_and_Color/Light_test3.py``` The file ```Light_test3.py``` is also a good example of how to operate the lights.

## Setting up the Neopixels via Arduino
Download the Adafruit_Neopixel library onto your Arduino platform. Go to ``` Example >> Adafruit NeoPixel >> simple ``` This should suffice for firing up the NeoPixels to a constant lighting. There are some useful notes:
- The number of pixels on each strip is 150
- The NexPixels can be connected in series.
- The Arduino is only required for powering the neopixels up.
- The data is read from pin ~6.
- Each NeoPixel requires its own power supply (of course).

## Running the Code for Kobukis
There were a total of 22 Trials. Each trial consists of python code, and Trials 21 and 22 are both final versions.




