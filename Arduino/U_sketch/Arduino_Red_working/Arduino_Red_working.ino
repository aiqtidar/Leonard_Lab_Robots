/* 
 * Open up a terminal and run:
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 */

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float64.h>


// Which pin on the Arduino is connected to the NeoPixels?
int PIN = 6;

// How many NeoPixels are attached to the Arduino?
int NUMPIXELS = 150;

ros::NodeHandle nh;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int color = 0;
int i;
float brightness = 0;
float x; 



void messageCb( const std_msgs::Float64& msg){
  x = msg.data;
    
  brightness = x;

    
  if (brightness==-1){color = 255;}
  if (brightness== 0){color = 120;}
  if (brightness== 1){color = 40;}

  /*
  for(int i = 0; i<NUMPIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(color,0,0));
    pixels.show();
  }
  */
  
  if (color == 0){
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  } else { digitalWrite(13, LOW-digitalRead(13));}   // blink the led
  
  pixels.setPixelColor(i, pixels.Color(color,0,0));
  pixels.show();
  if(i<151){i = i + 1;}
  
  
  //test.data = color;
  //p.publish( &test );
}

std_msgs::Float64 test;
ros::Subscriber<std_msgs::Float64> s("/controller/value/red_beta", &messageCb);
//ros::Publisher p("/arduino/red", &test);


void setup()
{
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  pixels.begin(); 
  pixels.show();
  nh.initNode();
  //nh.advertise(p);
  nh.subscribe(s);
  i = 0;
}

void loop()
{

  nh.spinOnce();
  delay(1);
  
}

