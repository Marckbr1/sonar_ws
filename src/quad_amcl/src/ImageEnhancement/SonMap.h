#ifndef SON_MAP_H
#define SON_MAP_H

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <vector>

#include "SonImgQueue.h"

#include <pcl/registration/icp.h>
#include <pcl/pcl_config.h>

// ROS lib (roscpp)
#include <ros/ros.h>

// ROS Img transport plugin
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

typedef pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ICP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

class SonMap
{
  Mat mapImg, mapCount,sonMask;
  Mat mapTransform;
  Size mapSz, imgSz;

  Point2f imgCenter;
  Point2f mapCenter;
  Point2f offset;

  string name="son_map";

  // ROS Stuff
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Publisher pubSonMap;


public:
  void setupROS();

  void reset();

  void setName(string name);

  SonMap();

  void newImg(Mat &img,
             Mat &transform);

  void newImg(const Mat &img,
             const Point2f &p, double theta);
};

#endif // SON_MAP_H
