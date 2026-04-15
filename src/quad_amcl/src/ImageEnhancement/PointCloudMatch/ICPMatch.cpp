#include "ICPMatch.h"

#include <pcl/features/fpfh_omp.h>
#include <iostream>
#include <app.h>
using namespace std;


void ICPMatch::setupICP()
{
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);

  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (maximumIterations);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (transformationEpsilon);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (euclideanFitnessEpsilon);
}

ICPMatch::ICPMatch()
{
  // PCL ICP parameters
  setupICP();
}

bool ICPMatch::match(PointCloudPtr cloud_A, PointCloudPtr cloud_B, MatchTransform *pTransform, Mat *pCvM)
{
  if(cloud_A->size() >= 10 && cloud_B->size() >= 10)
  {
    icp.setInputSource(cloud_A);
    icp.setInputTarget(cloud_B);

    PointCloud Final;
    icp.align(Final);

    double score = icp.getFitnessScore()/ min(cloud_A->size(),cloud_B->size());

    if(!icp.hasConverged() || score > 200.0)
    {
      cout << "Not converged!! Score " << score << endl;
      return false;
    }

    Eigen::Matrix<float,4,4> m = icp.getFinalTransformation();

    if(pCvM!= nullptr)
    {
      *pCvM = transform2CVTransform(m);
    }

    if(pTransform!= nullptr)
    {
      // Image origin
      Point2f imgO(776.8,855.4);

      Eigen::Matrix<float,4,4> t, mt, nM;

      t = t.Identity(); mt = mt.Identity();
      t(0,3) = imgO.x; mt(0,3) = -imgO.x;
      t(1,3) = imgO.y; mt(1,3) = -imgO.y;

      // Convert m to sonar img origin (middle bottom position of the image)
      nM = mt * m * t;

//      cout << "mT: " << endl << mt << endl
//           << "t: " << endl << t << endl
//           << "m: " << endl << m << endl
//           << "nM: " << endl << nM << endl << endl;

      pTransform->score = score;
      pTransform->tx = nM(0,3);  pTransform->ty = nM(1,3);
      pTransform->sx = sqrt(nM(0,0)*nM(0,0) + nM(0,1)*nM(0,1));
      pTransform->sy = sqrt(nM(1,0)*nM(1,0) + nM(1,1)*nM(1,1));

      if(nM(0,0)<0.0) pTransform->sx= -pTransform->sx;
      if(nM(1,1)<0.0) pTransform->sy =-pTransform->sy;

      pTransform->dTheta = -atan2(nM(1,0),nM(1,1));
    }

    return true;
  }else
  {
    cout << "Not enough points on the clouds!!" << endl
         << "A: " << cloud_A->size() << endl
         << "B: " << cloud_B->size() << endl;
  }
  return false;
}
