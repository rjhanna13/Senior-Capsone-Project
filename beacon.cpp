// vision.cpp
// RJ Hanna
// System States:
// 0 - Beacon not found (searching)
// 1 - Beacon found, rotate slight left
// 2 - Beacon found, proceed forward
// 3 - Beacon found, rotate slight right
// 4 - Objective reached, stop
//
//
/////////////////////////////////////////////////
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Int8.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <unistd.h>
#include <cmath>
using namespace ros;
using namespace std;
using namespace cv;

cv::VideoCapture cap(0);
/////////////////////////////////////////////////
float getCenter(vector<cv::KeyPoint> keypoints)
  {
  float x_avg;
  x_avg = (keypoints[0].pt.x + keypoints[1].pt.x)/2;
  return(x_avg);
  }
/////////////////////////////////////////////////
float getDistance(vector<cv::KeyPoint> kps)
  {
  float dist = sqrt((kps[1].pt.x - kps[0].pt.x)*(kps[1].pt.x - kps[0].pt.x) +
                    (kps[1].pt.y - kps[0].pt.y)*(kps[1].pt.y - kps[0].pt.y));
  return dist;
  }

/////////////////////////////////////////////////
class beaconFind
  {
  protected:
    std_msgs::Int8 state;
    Mat tic, toc, diff;
    unsigned int delay, cols;
    float init_distance;
    bool first_sight;
  public:
    beaconFind() : delay(400000), cols(1), init_distance(100000000), first_sight(true)
      { state.data = 0; }
    std_msgs::Int8 get_state()
      { return(state); }
    void set_delay()
      {
      unsigned int temp;
      cout << "\nEnter delay time (ms): "; cin >> temp; 
      delay = temp;
      } 
    Mat tic_toc()
      {
      cap >> tic;
      usleep(delay);
      cap >> toc;
      cols = toc.cols;
      absdiff(tic, toc, diff);
      cvtColor(diff, diff, CV_BGR2GRAY);
      threshold(diff, diff, 170, 255, cv::THRESH_BINARY);
      return diff;
      }
    std_msgs::Int8 determine_state(vector<cv::KeyPoint> kps)
      {
      float center, distance;
      distance = sqrt((kps[1].pt.x - kps[0].pt.x)*(kps[1].pt.x - kps[0].pt.x) +
                    (kps[1].pt.y - kps[0].pt.y)*(kps[1].pt.y - kps[0].pt.y));
      center = (kps[0].pt.x + kps[1].pt.x)/2;

      if(kps.size() != 2)
        { state.data = 0; }
      if(kps.size() == 2)
        {
        if(first_sight == true)
          {init_distance = distance; first_sight = false;}
        if(center < (3/5)*cols)
          { state.data = 1; }
        if(center > (4/5)*cols)
          { state.data = 3; }
        else
          { state.data = 2; }
         }
      if(distance <= (init_distance/2))
        { state.data = 4; }
      return state;
      } 
  };
////////////////////////////////////////////////
int main(int argc, char **argv)
  {

  std_msgs::Int8 msg;
  msg.data = 0;
  cv::SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 50.0f; 
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;
  params.minArea = 100.0f;
  params.maxArea = 50000.0f;

  cv::SimpleBlobDetector blob_detector(params);
  vector<cv::KeyPoint> keypoints;

  beaconFind bf;

  init(argc, argv, "vision");
  NodeHandle n;
  Publisher chatter_pub = n.advertise<std_msgs::Int8>("chatter", 1000);
  Rate loop_rate(10);

  if ( !cap.isOpened() )  // if not success, exit program
    {
    cout << "Cannot open the web cam" << endl;
    return -1;
    }

  namedWindow("test", CV_WINDOW_AUTOSIZE);
  bf.set_delay();


  while(ok())
    {
    msg = bf.get_state();
    if(msg.data == 4)
      {break;}
    msg.data = 0;
    blob_detector.detect(bf.tic_toc(), keypoints); 
    imshow("test",bf.tic_toc());
    if(keypoints.size() != 0)
      {msg = bf.determine_state(keypoints);}
    cout << msg.data;
    ROS_INFO("%d", msg.data);
    chatter_pub.publish(msg);
    spinOnce();
    loop_rate.sleep();
    }

  return(0);
  }
    
