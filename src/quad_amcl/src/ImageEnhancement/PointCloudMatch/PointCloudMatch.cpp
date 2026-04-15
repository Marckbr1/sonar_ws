#include "PointCloudMatch.h"


Mat transform2CVTransform(const Eigen::Matrix<float,4,4> &m)
{
  return (Mat_<float>(3,3)
        << m(0,0) , m(0,1), m(0,3),
           m(1,0) , m(1,1), m(1,3),
             0.f  ,   0.f ,   1.f);
}
