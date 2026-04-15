#ifndef IMAGELOADER_H
#define IMAGELOADER_H

// ROS lib (roscpp)
#include <ros/ros.h>

// ROS Img transport plugin
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// OpenCV for image processing, loading and display
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Boos filesystem to search files in a folder
#include <boost/filesystem.hpp>

// C++ Standard libraries (STL)
#include <vector>

// Num of particles
#include <std_msgs/Int32.h>
#include <quad_amcl/StampedInteger.h>


// Name spaces
using namespace boost::filesystem;
using namespace cv;
using namespace std;

class ImageLoader
{
private:
  // ROS Stuffs
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  image_transport::ImageTransport it;

  image_transport::Publisher pubSatImgs;
  image_transport::Publisher pubSonImg;
  ros::Publisher pubNumParticles;

  ros::Rate r;

  // Methods
  bool setup();
  
public:

  path imgFolder;

  ImageLoader();

  bool start();

  void publishMsgs();

};

#endif // IMAGELOADER_H
