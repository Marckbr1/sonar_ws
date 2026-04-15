#ifndef POINTCLOUDMATCH_H
#define POINTCLOUDMATCH_H

#include <opencv2/core.hpp>

#include <vector>

using namespace cv;
using namespace std;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef struct
{
  double score;
  double tx,ty;
  double dTheta; // Radians
  double sx,sy;
}MatchTransform;

class PointCloudMatch
{
public:
  using Ptr = shared_ptr<PointCloudMatch>;

  PointCloudMatch(){}
  virtual ~PointCloudMatch(){}

  virtual bool match(PointCloudPtr cloud_A,
             PointCloudPtr cloud_B,
             MatchTransform *pTransform=nullptr,
             Mat *pCvM=nullptr) = 0;
};

Mat transform2CVTransform(const Eigen::Matrix<float,4,4> &m);

#endif // POINTCLOUDMATCH_H
