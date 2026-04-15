#ifndef SON_MATCH_H
#define SON_MATCH_H

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
//#include <opencv2/features2d.hpp>

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

#include "PointCloudMatch/PointCloudMatch.h"


class SonMatchListner
{
public:
  SonMatchListner(){}
  virtual void reset() = 0;
};

class SonMatch
{
  Mat sonMask, sonMaskHalf, sonPattern;
  double patternMin, patternMax;
  SonImgQueue sons;
  Size imgSize;
  string name="son_match";

  // ROS stuff
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Publisher pubBinImg;
  image_transport::Publisher pubMagImg;
  image_transport::Publisher pubSonFeatures;
  image_transport::Publisher pubSonPts;
  image_transport::Publisher pubSonAligment;

  // Prv_pts
  Mat prv_img;
  PointCloudPtr prv_cloud;
  vector<Point2f> prv_pts;

  // Point Cloud Match Stuff
  PointCloudMatch::Ptr pcMatch;

  // Current pose
  double s2M=1.0;// Scale from pix to meters

  bool showImgs=true;

  bool drawPoints(const Size &imgSz, Mat &dstImg, Mat M,
                  PointCloudPtr cloud_A,
                  PointCloudPtr cloud_B);

  bool drawAligment(Mat &imgA, Mat &imgB,
                    Mat &M, Mat &dstImg);

  bool match(PointCloudPtr cloud_A,
             PointCloudPtr cloud_B,
             MatchTransform *pTransform=nullptr,
             Mat *pCvM=nullptr);

  bool match2(PointCloudPtr cloud_A,
             PointCloudPtr cloud_B,
             MatchTransform *pTransform=nullptr,
             Mat *pCvM=nullptr);

  void setupICP();

  bool describe(Mat &sonImg,
                PointCloudPtr &p,
                Mat *pBinImg=nullptr,
                Mat *pMagImg=nullptr,
                Mat *pAngImg=nullptr,
                Mat *pSonFeatures=nullptr);

  Mat get2DTransform(double x, double y, double theta);

  bool reset();


public:
  SonMatch();

  void setupROS();

  bool newImg(Mat &img,
            Mat &transform);

  void setName(string name);

  void setPointCloudMatch(PointCloudMatch::Ptr pcM);
};

#endif // SON_MATCH_H
