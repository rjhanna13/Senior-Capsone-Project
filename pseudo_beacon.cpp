// vision.cpp
// RJ Hanna
// System States:
// 0 - Beacon not found (searching)
// 1 - Beacon found, rotate slight left
// 2 - Beacon found, proceed forward
// 3 - Beacon found, rotate slight right
// 4 - Objective reached, stop
//
/////////////////////////////////////////////////
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <cmath>
using namespace ros;
using namespace std;
using namespace cv;

std_msgs::String ready_check;
/////////////////////////////////////////////////
void chatterEcho(const std_msgs::String::ConstPtr& state)
  {
  ROS_INFO("\nMotion state: [%s]\n", state->data.c_str());
  ready_check = *state;
  }
////////////////////////////////////////////////
int main(int argc, char **argv)
  {

  std_msgs::Int8 msg;
  msg.data = 0;


  init(argc, argv, "vision");
  NodeHandle n;
  Publisher chatter_pub = n.advertise<std_msgs::Int8>("chatter", 1000);
  Rate loop_rate(10);
  Subscriber motion_state = n.subscribe("arduino", 1000, chatterEcho);

  int count_a = 10;
  int count_b = 5;
  int count_c = 30;

  while(ok())
    {
    if(!ready_check.data.compare("Not Ready To Receive"))
      {continue;}
    if (count_a)
      {msg.data = 0; count_a--;}
    else
      {
      if (count_b)
        {msg.data = 1; count_b--;}
      else
        {
        if (count_c)
          {msg.data = 2; count_c--;}
        else
          {msg.data = 4; break;}
        }
      }

    ROS_INFO("%d", msg.data);
    chatter_pub.publish(msg);
    spinOnce();
    loop_rate.sleep();
    }

  return(0);
  }
    
