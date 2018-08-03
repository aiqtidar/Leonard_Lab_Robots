/* 
 * Open up a terminal and run:
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 */

#include <ros.h>
#include <std_msgs/Float64.h>


ros::NodeHandle nh;

float x; 

void messageCb( const std_msgs::Float64& msg){
  x = msg.data;
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

std_msgs::Float64 test;
ros::Subscriber<std_msgs::Float64> s("/controller/value/blue_beta", &messageCb);
ros::Publisher p("/arduino/blue", &test);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  test.data = x;
  p.publish( &test );
  nh.spinOnce();
  delay(10);
}

