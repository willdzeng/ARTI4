#include <SabertoothSimplified.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

SabertoothSimplified ST;

void cmd_callback( const geometry_msgs::Twist& msg){
  int vx = msg.linear.x * 127;
  ST.motor(1, vx);
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_callback);

void setup()
{
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}