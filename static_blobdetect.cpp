// blobdetect.cpp
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <unistd.h>
#include <string>
using namespace std;
using namespace cv;

////////////////////////////////////////////
struct hsvParams
  {
  int hL, sL, vL, hH, sH, vH;
  };
////////////////////////////////////////////
void removenoise(Mat&);
/////////////////////////////////////////////

int main()
  {
  hsvParams hsv = {0,0,180,255,10,255};

  //Set up blob detection parameters
  SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 50.0f; 
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;
  params.minArea = 1000.0f;
  params.maxArea = 500000.0f;

  SimpleBlobDetector blob_detector(params);
  vector<KeyPoint> keypoints;

  const string filename("samplepics/15.JPG");
  string text("Object not found");

  Mat img, imgHSV, imgTHRESH, out;
  img = imread(filename, CV_LOAD_IMAGE_COLOR);
  
  //convert color to HSV, threshold and remove noise
  cvtColor(img, imgHSV, COLOR_BGR2HSV);
  inRange(imgHSV, Scalar(hsv.hL, hsv.sL, hsv.vL),
         Scalar(hsv.hH, hsv.sH, hsv.vH), imgTHRESH);
  removenoise(imgTHRESH);

  namedWindow("Input", WINDOW_NORMAL);
  namedWindow("Detection", WINDOW_NORMAL);
  
  //Initialize blobdetector with predefine parameters
  SimpleBlobDetector blobDetect(params);
  //Detect blobs
  blobDetect.detect(imgTHRESH, keypoints);
  drawKeypoints(imgTHRESH, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);
  //Circle blobs
  for(int i = 0; i < keypoints.size(); i++)
    {
    circle(out, keypoints[i].pt, 1.5*keypoints[i].size, CV_RGB(0,255,0), 20, 8);
    }

  if(keypoints.size() == 1)
    text = "Object Found";
  if(keypoints.size() > 1)
    text = "Error";

  putText(out, text, Point(100,200), FONT_HERSHEY_PLAIN, 20, Scalar(0, 0, 255), 20);

  //equalizeHist(img, out);


  for(;;)
    {
    imshow("Input", img);
    imshow("Detection", out);
    if(waitKey(30) >= 0 ) break; 
    }

  return 0;
  }
//.....................................................
//removenoise()
void removenoise(Mat& image)
  {
  //Morphologial opening
  erode(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  dilate(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  //Morphological closing
  dilate(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  erode(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  }
//.................................................................................
