#ifndef PARTICLE_FILTER_NODE_H
#define PARTICLE_FILTER_NODE_H

// ROS lib (roscpp)
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

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

#include <SatelliteManager.h>

#include "SonImg.h"
#include "SonImgQueue.h"
#include "SonMatch.h"
#include "SonMap.h"

// Name spaces
using namespace boost::filesystem;
using namespace cv;
using namespace std;

class SonEnhancement
{
private:
  // ROS Stuffs
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Rate r;

  // Pose
  ros::Subscriber subPose;
  bool hasFirstPose=false;

  // Img Rank
  image_transport::Subscriber subSonImg;
  bool hasFirstSon=false;

  // Image subscribers...
  image_transport::ImageTransport it;

  image_transport::Publisher pubSatImgs;
  image_transport::Publisher pubSonImg;

  // Visualisation
  image_transport::Publisher pubMeanSon;
  image_transport::Publisher pubMeanSonNoCompensation;

  image_transport::Publisher pubMapImg;
  image_transport::Publisher pubParticlesView;

  // ===== Class control ======
  Mat lastSon;
  Mat sonMask;
  double lastTime, lastX, lastY, lastYaw;
  double vx=0.0,vy=0.0,vYaw=0.0;
  double sonarRange=50.0, sonarFoV=130.0;
  Mat colors; // Color map (Used by draw functions)

  bool doTest=false;
  bool disableOrientation=false;

  // ===== Satellite image stuff =====
  SatelliteManager sat;
  path mapPath;

  // ==== Son image stuff ====
  SonImgQueue sonImgs;

  SonMatch sonMatch,sonMatch2;
  SonMap sonMap, sonMap2;

  // Methods
  bool setup();
  void sonCallback(const sensor_msgs::ImageConstPtr& msg);

  void poseCallback(const geometry_msgs::PoseStamped &msg);

  void checkSonEvalTimming();

  void getTransformedSonImg(Size imgSize, Mat sonImg,
                            Point p, double deg, Mat &sonImgResult, Mat &sonMaskResult);

  void getTransformedSonImg(Size imgSize, Mat sonImg,
                            const Mat &transform,
                            Mat &sonImgResult, Mat &sonMaskResult);


  // Visualisation stuff...
  void updateMapView();
  void tryEnhancement();
  void tryNoCompensationEnhancement();

//  void add(Mat &src, Mat & dst);
//  void toFinalImg(Mat &src, Mat & dst, int n);

//  void copy(Mat &src, Mat & dst);

  void drawSonImg(Mat &img,
            const SonImg &p,
            const string &txt,
            const Scalar &color,
            int thickness);

  bool showImgMap=false;
  bool showParticlesView=false;

public:
  path imgFolder;

  SonEnhancement();

  bool start();

};

#endif // PARTICLE_FILTER_NODE_H
