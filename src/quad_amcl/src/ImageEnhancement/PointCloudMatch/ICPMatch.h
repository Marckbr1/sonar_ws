#ifndef ICPMATCH_H
#define ICPMATCH_H

#include <opencv2/core.hpp>
#include <pcl/registration/icp.h>
#include <pcl/pcl_config.h>

#include <vector>

#include "PointCloudMatch.h"

using namespace cv;

typedef pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ICP;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ICPMatch: public PointCloudMatch
{
  ICP icp;
  // ICP parameters
  double maxCorrespondenceDistance=70.0,
         transformationEpsilon=1e-8,
         euclideanFitnessEpsilon=1e-1;
  int maximumIterations=100;

  void setupICP();
public:
  ICPMatch();
  virtual ~ICPMatch(){}

  bool match(PointCloudPtr cloud_A,
             PointCloudPtr cloud_B,
             MatchTransform *pTransform=nullptr,
             Mat *pCvM=nullptr);
};

#endif // ICPMATCH_H
