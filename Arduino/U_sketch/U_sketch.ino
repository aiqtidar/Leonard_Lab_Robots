/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */

#include <ros.h>
#include <std_msgs/Float64.h>


ros::NodeHandle nh;

float x; 

void messageCb( const std_msgs::Float64& msg){
  x = msg.data;
}

std_msgs::Float64 test;
ros::Subscriber<std_msgs::Float64> 
s("/controller/value/u", &messageCb);

void setup()
{
  nh.initNode();
  nh.subscribe(s);
}

void loop()
{
  Serial.print(x);
  nh.spinOnce();
  delay(10);
}

