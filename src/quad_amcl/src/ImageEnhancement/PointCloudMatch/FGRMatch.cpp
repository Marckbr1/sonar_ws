#include "FGRMatch.h"

#include <pcl/features/fpfh_omp.h>
#include <iostream>
#include <app.h>
using namespace std;


pcl::PointCloud<pcl::PointNormal>::Ptr toPointNormalPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr c)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr nCloud(new pcl::PointCloud<pcl::PointNormal>());

  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > & pts = c->points;

  for (uint i = 0; i < pts.size(); ++i)
  {
    pcl::PointNormal p;
    p.x = pts[i].x;
    p.y = pts[i].y;
    p.z = pts[i].z;

    p.normal_x = 0.0;
    p.normal_y = 0.0;
    p.normal_z = 1.0;

    nCloud->push_back(p);
  }

  return nCloud;
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr describePoints(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
  pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());

  fest.setRadiusSearch(100);
  fest.setInputCloud(cloud);
  fest.setInputNormals(cloud);
  fest.compute(*features);

  return features;
}

void features2Vec(pcl::PointCloud<pcl::FPFHSignature33>::Ptr feat,
                  vector<Eigen::VectorXf> &vecFeat)
{
  vector<pcl::FPFHSignature33, Eigen::aligned_allocator<pcl::FPFHSignature33> >
        &features = feat->points;

  vecFeat.resize(features.size(),Eigen::VectorXf(33));

  for(uint i = 0; i < features.size(); i++)
  {
    memcpy(&vecFeat[i](0),
            features[i].histogram,
            33*sizeof(float));
  }
}

void cloud2Vec(pcl::PointCloud<pcl::PointXYZ>::Ptr c,
                  vector<Eigen::Vector3f> &vecFeat)
{
  vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >
        &pts = c->points;

  vecFeat.resize(pts.size());

  for(uint i = 0; i < vecFeat.size(); i++)
  {
    memcpy(&vecFeat[i](0),
           pts[i].data,
           3*sizeof(float));
  }
}

bool FGRMatch::match(PointCloudPtr cloud_A, PointCloudPtr cloud_B, MatchTransform *pTransform, Mat *pCvM)
{
  if(cloud_A->size() >= 10 && cloud_B->size() >= 10)
  {
    vector<Eigen::VectorXf> feat_A, feat_B;
    vector<Eigen::Vector3f> pts_A, pts_B;

    // Point cloud description and type conversions
    {
      pcl::PointCloud<pcl::PointNormal>::Ptr
          cloudNormA= toPointNormalPlane(cloud_A),
          cloudNormB= toPointNormalPlane(cloud_B);

      pcl::PointCloud<pcl::FPFHSignature33>::Ptr
          featuresA = describePoints(cloudNormA),
          featuresB = describePoints(cloudNormB);

      features2Vec(featuresA,feat_A);
      features2Vec(featuresB,feat_B);

      cloud2Vec(cloud_A,pts_A);
      cloud2Vec(cloud_B,pts_B);
    }

    fgr::CApp app;

    app.LoadFeature(pts_A,feat_A);
    app.LoadFeature(pts_B,feat_B);

    app.NormalizePoints();

    app.AdvancedMatching();
    app.OptimizePairwise(true);

    Eigen::Matrix<float,4,4> m = app.GetOutputTrans();

    cout << "Transform: " << endl << m << endl << endl;
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

      pTransform->score = 0.0;
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
