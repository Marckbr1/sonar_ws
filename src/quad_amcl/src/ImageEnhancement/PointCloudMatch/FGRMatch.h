#ifndef FGRMATCH_H
#define FGRMATCH_H

#include <opencv2/core.hpp>
#include <pcl/registration/icp.h>
#include <pcl/pcl_config.h>

#include <vector>

#include "PointCloudMatch.h"

using namespace cv;

typedef pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ICP;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class FGRMatch: public PointCloudMatch
{
public:
  FGRMatch(){}
  virtual ~FGRMatch(){}

  bool match(PointCloudPtr cloud_A,
             PointCloudPtr cloud_B,
             MatchTransform *pTransform=nullptr,
             Mat *pCvM=nullptr);
};

#endif // FGRMATCH_H
