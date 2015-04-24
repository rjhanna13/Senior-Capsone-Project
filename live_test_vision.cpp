// vision.cpp
// RJ Hanna
// f_4 with 500000
// System States:
// 0 - Beacon not found (searching)
// 1 - Beacon found, rotate slight left
// 2 - Beacon found, proceed forward
// 3 - Beacon found, rotate slight right
// 4 - Objective reached, stop
//
//////////////.. HEADER FILES ../////////////////////
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cmath>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
using namespace ros;
using namespace std;
using namespace cv;

std_msgs::Bool ready;
std_msgs::Float64 msg;
//////////////////////////////////////////////////////
////////////.. FUNCTION DECLARATIONS.. ///////////////
//////////////////////////////////////////////////////

//....................................................
Mat tic_toc(int delay, VideoCapture& cap, int& dilFac, struct HSVparam);
//....................................................
void set_delay(int& delay);
//....................................................
void chatterbeacon_request(const std_msgs::Bool::ConstPtr& beacon_request);

///////////////////////////////////////////////////////
//////////////.. CLASS DECLARATIONS ../////////////////
///////////////////////////////////////////////////////
struct HSVparam
  {
  int Hlow,Hhigh,Slow,Shigh,Vlow,Vhigh;
  };
///////////////////////////////////////////////////////
class beaconFind
  {
  protected:
    std_msgs::Float64 state;
    unsigned int cols;
    float init_separation;
    bool first_sight;
  public:
    beaconFind() : cols(1), init_separation(100000000), first_sight(true)
      { state.data = 0; }
    std_msgs::Float64 get_state()
      { return(state); }
    double determine_state(vector<cv::KeyPoint> kps)
      {
      if(kps.size() != 2)
        { state.data = 0; }
      // If there are 2 keypoints, calculate center and separation
      if(kps.size() == 2)
        {
        float separation = sqrt((kps[1].pt.x - kps[0].pt.x)*(kps[1].pt.x - kps[0].pt.x) +
                    (kps[1].pt.y - kps[0].pt.y)*(kps[1].pt.y - kps[0].pt.y));
        float center = (kps[0].pt.x + kps[1].pt.x)/2; 
        //cout << endl << separation << "  " << center << " " << cols;

        /*if(first_sight == true)
          {init_separation = separation; first_sight = false;}
        if(center < static_cast<float>((2.0/5)*cols))
          { state.data = 1; }
        if(center > static_cast<float>((3.0/5)*cols))
          { state.data = 3; }
        else
          { state.data = 2; }
         
        if(separation <= (init_separation/2))
          { state.data = 4; }*/
      
      state.data = (center/640)*60 - 30.0;
        }
      return state.data;
      } 
  };
//////////////////////////////////////////////////////
////////////////////.. MAIN BODY ..///////////////////
//////////////////////////////////////////////////////
int main(int argc, char **argv)
  {
  //Initialize camera
  VideoCapture cap(2);
  if ( !cap.isOpened() )  // if not success, exit program
    {
    cout << "Cannot open the web cam" << endl;
    return -1;
    }

  HSVparam p = {50,100,0,100,213,255};

  //define msg for passing data to motion node
  msg.data = 0;
  ready.data = 1;

  //set up blobdetector
  cv::SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 75.0f; 
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;
  params.minArea = 50.0f;
  params.maxArea = 10000.0f;
  cv::SimpleBlobDetector blob_detector(params);
  
  //define keypoint vector
  vector<cv::KeyPoint> keypoints;

  //define beaconfind class
  beaconFind bf;

  //set up publisher "beacon_data"
  init(argc, argv, "vision");
  NodeHandle n; 
  Publisher beacon_data = n.advertise<std_msgs::Float64>("beacon_data", 1);
  Rate loop_rate(10);

  //set up subscriber "beacon_request"
  Subscriber beacon_request = n.subscribe("beacon_request", 1000, chatterbeacon_request);

  namedWindow("output", CV_WINDOW_AUTOSIZE);
  //namedWindow("feed", CV_WINDOW_AUTOSIZE);
  
  //setup delay between frames
  int delay;
  set_delay(delay);

  //make sliders for fine tuning
  //createTrackbar("delay", "output", &delay, 2000000);
  int dilationParam = 10;
  //createTrackbar("dilation", "output", &dilationParam, 100);
  spinOnce();
  double storage[2];

  while(ok())
    {
    //check if masternode is ready to receive data
    spinOnce();
    if(!ready.data)
      continue;
    
    msg = bf.get_state();

    msg.data = 100;
 
    for(int i = 0 ; i < 1 ; i++)
      {
      Mat grab = tic_toc(delay, cap, dilationParam, p);
      blob_detector.detect(grab, keypoints);
      imshow("output", grab);
      if(keypoints.size() != 0 && keypoints.size() == 2)
        {storage[i] = bf.determine_state(keypoints); cout << storage[i] << endl;}
        
      }

    if(waitKey(30) == 27)
      {
      cout << "\nesc key pressed by user\n";
      break;
      }
    cout << endl << keypoints.size();
    if (abs(storage[1] - storage[0]) < 5)
      {msg.data = storage[0];}
    beacon_data.publish(msg);
    ROS_INFO("%f", msg.data);
    
    //wait for beacon request before running again
    ready.data = 0;
     
    spinOnce();
    loop_rate.sleep();
  /*Mat imgOriginal;
    cap.read(imgOriginal);
    imshow("test", imgOriginal);*/

    }

  return(0);
  }
//.............................................
Mat tic_toc(int delay, VideoCapture& cap, int& dilFac, HSVparam p)
  { 
  Mat tic, toc, diff;
  cap >> tic;
  usleep(delay);
  cap >> toc;
  int cols = toc.cols;
  absdiff(tic, toc, diff);
  cvtColor(diff, diff, CV_BGR2GRAY);
  threshold(diff, diff, 119, 255, cv::THRESH_BINARY);
  dilate(diff, diff, getStructuringElement(MORPH_ELLIPSE, Size(dilFac, dilFac)) ); 
  return diff;
  }
//..............................................
void set_delay(int& delay)
  {
  unsigned int temp;
  cout << "\nEnter delay time (ms): "; cin >> temp; 
  delay = temp;
  } 
//................................................
void chatterbeacon_request(const std_msgs::Bool::ConstPtr& beacon_request)
  {
  ROS_INFO("\nBeacon Request: [%s]\n", beacon_request->data ? "True" : "False");
  ready.data = true;
  }
