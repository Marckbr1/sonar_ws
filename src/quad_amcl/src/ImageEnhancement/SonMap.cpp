#include "SonMap.h"

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <iostream>

// FGR
#include <app.h>

using namespace std;
using namespace cv;


Mat scale(double s)
{
  Mat m = (Mat_<float>(3,3)
    <<  s , 0.f, 0.f,
       0.f,  s , 0.f,
       0.f, 0.f, 1.f);

  return m;
}

Mat rot(double theta)
{
  double st = sin(theta), ct=cos(theta);

  Mat m = (Mat_<float>(3,3)
    << ct , -st, 0.f,
       st ,  ct, 0.f,
       0.f, 0.f, 1.f);

  return m;
}

Mat translation(double x,double y)
{
  Mat m = (Mat_<float>(3,3)
    << 1.f , 0.f, x  ,
       0.f , 1.f, y  ,
       0.f , 0.f, 1.f);

  return m;
}

Point2f transform(const Point2f &p, const Mat &m)
{
  Mat mp = (Mat_<float>(3,1)
    << p.x , p.y, 1.f),
      mr;
  mr = m * mp;
  return Point2f(mr.at<float>(0,0), mr.at<float>(0,1));
}


void SonMap::setupROS()
{
  pubSonMap = it.advertise("/"+name,1);
}

void SonMap::reset()
{
  mapImg.release();
  mapCount.release();
  mapTransform = Mat::eye(3,3,CV_32FC1);
}

void SonMap::setName(string name)
{
  this->name = name;
}

SonMap::SonMap():
  it(nh)
{
  sonMask = imread("/home/auros/ros/netuno_ws/SonEnhancementResults/son_mask.png",
                   IMREAD_GRAYSCALE);
//  pyrDown(sonMask,sonMask);
//  pyrDown(sonMask,sonMask);

  mapTransform = Mat::eye(3,3,CV_32FC1);

}

void SonMap::newImg(Mat &img, Mat &transform)
{
  Mat workImg;

  Mat mTransform(transform.clone());

//  mTransform.at<float>(0,2) *= 0.5;
//  mTransform.at<float>(1,2) *= 0.5;

  img.copyTo(workImg);
//  pyrDown(workImg,workImg);
//  pyrDown(workImg,workImg);

  if(mapImg.empty())
  {
    imgSz = Size(workImg.cols, workImg.rows);
    mapSz = Size(workImg.cols*2, workImg.rows*2);

    mapImg = Mat(mapSz.height,mapSz.width,CV_32S,Scalar(0));
    mapCount = Mat(mapSz.height,mapSz.width,CV_16S,Scalar(0));

    mapTransform *= translation(mapSz.width/2.0-imgSz.width/2.0,
                               mapSz.height/2.0-imgSz.height);
  }

  mapTransform = mapTransform * mTransform.inv();

  Mat sonTransformed, maskTransformed;

  // Copy transform
  mTransform = mapTransform.clone();

  // Warp son image
  warpAffine(workImg,sonTransformed,
             mapTransform(Rect(0,0,3,2)),
             mapSz);

  // Warp son mask
  warpAffine(sonMask,maskTransformed,
             mapTransform(Rect(0,0,3,2)),
             mapSz);

  // Add image to the map
  cv::add(mapImg,sonTransformed,
          mapImg,maskTransformed,
          CV_32S);

//  double minV,maxV;
//  minMaxLoc(mapImg,&minV,&maxV);
//  cout << "MapImg minV: " << minV << " maxV: " << maxV << endl;

  // Add pixel count
  cv::add(mapCount,Scalar(1),
          mapCount,maskTransformed,
          CV_16S);
//  minMaxLoc(mapCount,&minV,&maxV);
//  cout << "MapCount minV: " << minV << " maxV: " << maxV << endl;

  Mat mMap;
  cv::divide(mapImg,mapCount,
             mMap,1,CV_8UC1);

  // Pub son_map img
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", mMap).toImageMsg();
  pubSonMap.publish(msg);

}

void SonMap::newImg(const Mat &img,
                    const Point2f &p,
                    double theta)
{

  if(mapImg.empty())
  {
    imgSz = Size(img.cols, img.rows);
    mapSz = Size(img.cols*2, img.rows*2);

    imgCenter.x = imgSz.width/2.f;
    imgCenter.y = imgSz.height;

    mapCenter.x = mapSz.width/2.f;
    mapCenter.y = mapSz.height/2.f;

    // Initialize map
    mapImg = Mat(mapSz.height,mapSz.width,CV_32S,Scalar(0));
    mapCount = Mat(mapSz.height,mapSz.width,CV_16S,Scalar(0));

    offset = p;

//    mapTransform *= translation(mapSz.width/2.0-imgSz.width/2.0 - x,
//                               mapSz.height/2.0-imgSz.height - y);

  }

//  mapTransform =
//      // Translate
//        translation(x - offset.x,
//                    y - offset.y);

  Mat sonTransformed, maskTransformed;

  // Warp son image
  warpAffine(img,sonTransformed,
             mapTransform(Rect(0,0,3,2)),
             mapSz);

  // Warp son mask
  warpAffine(img,maskTransformed,
             mapTransform(Rect(0,0,3,2)),
             mapSz);

  // Add image to the map
  cv::add(mapImg,sonTransformed,
          mapImg,maskTransformed,
          CV_32S);

//  double minV,maxV;
//  minMaxLoc(mapImg,&minV,&maxV);
//  cout << "MapImg minV: " << minV << " maxV: " << maxV << endl;

  // Add pixel count
  cv::add(mapCount,Scalar(1),
          mapCount,maskTransformed,
          CV_16S);
//  minMaxLoc(mapCount,&minV,&maxV);
//  cout << "MapCount minV: " << minV << " maxV: " << maxV << endl;

  Mat mMap;
  cv::divide(mapImg,mapCount,
             mMap,1,CV_8UC1);

  imshow("Son Map", mMap);

  // Pub son_map img
//  sensor_msgs::ImagePtr msg =
//      cv_bridge::CvImage(std_msgs::Header(), "mono8", mMap).toImageMsg();
//  pubSonMap.publish(msg);
}
