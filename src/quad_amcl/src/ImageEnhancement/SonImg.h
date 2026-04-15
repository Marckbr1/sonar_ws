#ifndef SON_IMG_H
#define SON_IMG_H

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <pcl/registration/icp.h>

using namespace cv;

class SonImg
{
public:
  double heading,x,y;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  Mat img;
  Mat m;

  SonImg();

};

#endif // SON_IMG_H
