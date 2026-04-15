#ifndef SON_MATCH_H
#define SON_MATCH_H

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

#include <pcl/registration/icp.h>
#include <pcl/pcl_config.h>


using namespace cv;
using namespace std;

typedef pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ICP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

class SonMatch
{
  SimpleBlobDetector::Params p;
  Mat sonMask, sonMaskHalf, sonPattern;
  double patternMin, patternMax;

  int minA, maxA;
  int minC, maxC;
  int minConv, maxConv;
  int minInn, maxInn;
  int contrast;

  // Sparse optical flow
  vector<Point2f> ps;
  vector<vector<Point2f>> m;
  vector<vector<char>> mStatus;
  Mat prv_img;

  int nGrid;
  double gW,
         gH;

  // LK Algorithm Variables
  TermCriteria criteria;
  Size lkWinSz;
  int lkMaxLvl;

  // Prv_pts
  vector<Point2f> prv_pts;

  // ICP paramaters
  double maxCorrespondenceDistance=20.0,
         transformationEpsilon=0.05,
         euclideanFitnessEpsilon=0.03;
  int maximumIterations=30;

  ICP icp;

  // Son pose
  double x=0.0, y=0.0, theta=0.0;


  double match(PointCloudPtr cloud_A,
               PointCloudPtr cloud_B);

  void setupICP();


  bool describe(Mat &sonImg,
                PointCloudPtr p,
                Mat *pBinImg=nullptr,
                Mat *pMagImg=nullptr,
                Mat *pAngImg=nullptr,
                Mat *pSonFeatures=nullptr);

public:

  SonMatch();

  void newImg1(Mat &img);
  void newImg2(Mat &img);
  void newImg3(Mat &trackImg);
  void newImg4(Mat &img);
  void newImg(Mat &img);



};

#endif // SON_MATCH_H
