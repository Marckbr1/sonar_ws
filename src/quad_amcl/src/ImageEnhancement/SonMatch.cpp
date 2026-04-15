#include "SonMatch.h"

#include "PointCloudMatch/ICPMatch.h"

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <iostream>

#include <cv_bridge/cv_bridge.h>


#include <pcl/features/fpfh_omp.h>
#include <pcl/impl/point_types.hpp>

#include <Eigen/Core>

using namespace std;
using namespace cv;

typedef pair<int,Point2f > PairFeature;

bool compPF(const PairFeature& a,const PairFeature& b)
{
  return a.first > b.first;
}

void operator << (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const vector<Point2f> &pts)
{
  // Fill in the CloudIn data
  cloud->width    = pts.size();
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < pts.size(); ++i)
  {
    cloud->points[i].x = pts[i].x;
    cloud->points[i].y = pts[i].y;
    cloud->points[i].z = 0;
  }
}

void operator >> (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<Point2f> &pts)
{
  pts.resize(cloud->points.size());
  for (size_t i = 0; i < pts.size(); ++i)
  {
    pts[i].x = cloud->points[i].x;
    pts[i].y = cloud->points[i].y;
    cloud->points[i].z = 0;
  }
}

ostream & operator << (ostream &os, MatchTransform &t)
{
  os << "score: " << t.score << endl
     << "sx: " << t.sx << " sy: " << t.sy
     << endl
     << "tx: " << t.tx << " ty: " << t.ty
     << endl
     << "psi: " << t.dTheta << endl;
  return os;
}

Point center(const vector<Point> &pts)
{
  Point c(0.f,0.f);
  for(uint i =0; i < pts.size(); i++)
  {
    c+= pts[i];
  }
  if(pts.size() >0)
  {
    c.x = round(c.x / float(pts.size()));
    c.y = round(c.y / float(pts.size()));
  }
  return c;
}

void addPoint(const vector<Point2f> &pts, const Point2f &p,
              vector<Point2f> &dst_pts)
{
  dst_pts.resize(pts.size());
  for(uint i = 0; i < pts.size();i++)
    dst_pts[i] = pts[i] + p;
}

Mat translateTransform(const Mat &M,const Point2f &p)
{
  Mat hM = Mat::eye(3, 3, CV_32F),
      hT = Mat::eye(3, 3, CV_32F),
      mhT = Mat::eye(3, 3, CV_32F),
      newM;

  M.copyTo(hM(Rect(0,0,3,2)));

  hT.at<float>(0,2) = p.x;
  hT.at<float>(1,2) = p.y;
  mhT.at<float>(0,2) = -p.x;
  mhT.at<float>(1,2) = -p.y;

  newM  = hT * hM * mhT;

//  cout << "hM: " << endl << hM << endl
//       << "hT: " << endl << hT << endl
//       << "r: " << endl << newM << endl << endl;

  return newM(Rect(0,0,3,2));
}

bool SonMatch::drawPoints(const Size &imgSz, Mat &dstImg,
                          Mat M,
                          PointCloudPtr cloud_A,
                          PointCloudPtr cloud_B)
{
  if(M.rows < 2 || M.rows > 3 ||
     M.cols != 3)
  {
    cout << "Wrong transform matrix!" << endl
         << "rows: " << M.rows << " cols: " << M.cols << endl;
    return false;
  }

  vector<Point2f> pa, pta,pb;
  cloud_A >> pa;
  cloud_B >> pb;
  transform(pa,pta,M(Rect(0,0,3,2)));

  dstImg = Mat(imgSz.height,imgSz.width,
               CV_8UC3,cv::Scalar(0,0,0));

  for(unsigned i = 0 ; i < pa.size(); i++)
  {
//    circle(dstImg,pa[i],3,
//           Scalar(255,0,0),-1);
    circle(dstImg,pta[i],3,
           Scalar(0,255,0),-1);
  }

  for(unsigned i = 0 ; i < pb.size(); i++)
  {
    circle(dstImg,pb[i],3,
           Scalar(0,0,255),-1);
  }
  return true;
}

bool SonMatch::drawAligment(Mat &imgA, Mat &imgB,
                            Mat &M, Mat &dstImg)
{
  if(M.rows < 2 || M.rows > 3 ||
     M.cols != 3)
  {
    cout << "Wrong transform matrix!" << endl
         << "rows: " << M.rows << " cols: " << M.cols << endl;
    return false;
  }

  Mat sbgr[3];

  sbgr[0] = Mat(imgB.rows,imgB.cols,CV_8UC1,Scalar(0));

  // Green is the aligned previous image
  warpAffine(imgA,sbgr[1],M(Rect(0,0,3,2)),
             Size(imgA.cols,imgA.rows));


  // Red is the current image
  sbgr[2] = imgB;


  merge(sbgr,3,dstImg);
}

bool SonMatch::describe(Mat &sonImg, PointCloudPtr &p,
                  Mat *pBinImg, Mat *pMagImg,
                  Mat *pAngImg, Mat *pSonFeatures)
{
  Mat fimg;
  sonImg.convertTo(fimg,CV_32F,1/255.0);

  // First order gradient derivatives on x (gx) and on y(gy)
  Mat gx, gy;

  Sobel(fimg,gx,CV_32F,1,0,1);
  Sobel(fimg,gy,CV_32F,0,1,1);
//    Scharr(fimg,gx,CV_32F,1,0);
//    Scharr(fimg,gy,CV_32F,0,1);

  // Compute magnitude and angle
  Mat magImg, angImg;
  cartToPolar(gx,gy,magImg,angImg,true);

//  double maxV, minV;
//  minMaxLoc(magImg,&minV, &maxV,0,0,sonMask);
//  cout << "minV: " << minV << " maxV: " << maxV << endl;

  Mat binImg;
  threshold(magImg,binImg,
            0.3,255,THRESH_BINARY);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  binImg.convertTo(binImg,CV_8UC1);

//  Mat kernel=
//     getStructuringElement(MORPH_CROSS,Size(3,3));
//  morphologyEx(binImg,binImg,MORPH_OPEN, kernel,Point(-1,-1),1);
//  morphologyEx(binImg,binImg,MORPH_OPEN, kernel,Point(-1,-1),1);
//  morphologyEx(binImg,binImg,MORPH_DILATE, kernel,Point(-1,-1),1);

//  imshow("binMorph",binImg);

  findContours(binImg, contours,
               hierarchy, RETR_EXTERNAL,
               CHAIN_APPROX_NONE);

  vector<PairFeature> fs(contours.size()); // Contour size, Contour Centroid

  for(uint i = 0; i < contours.size();i++)
  {
//      const Moments &m = moments(contours[i]);
//      fs[i].second = Point2f( static_cast<float>(m.m10 / (m.m00 + 1e-5)),
//                       static_cast<float>(m.m01 / (m.m00 + 1e-5)) );

      fs[i].first = contours[i].size();
      fs[i].second = center(contours[i]);
  }

  sort(fs.begin(),fs.end(),compPF);

  vector<Point2f> pts;
  pts.reserve(fs.size());

  for(uint i = 0; i < fs.size();i++)
  {
    if(fs[i].first < 20) break;

    pts.push_back(fs[i].second);
  }

  if(pBinImg != nullptr)
    *pBinImg = binImg;

  if(pMagImg != nullptr)
    *pMagImg = magImg;

  if(pAngImg != nullptr)
    *pAngImg = angImg;

  if(pSonFeatures!= nullptr)
  {
    cvtColor(sonImg,*pSonFeatures,COLOR_GRAY2BGR);

    for(uint i = 0; i < pts.size();i++)
    {
      circle(*pSonFeatures,pts[i],5,
             Scalar(0,255,0),2);
    }
  }

  if(pts.size() < 10)
    return false;

  p.reset(new PointCloud());

//  addPoint(pts, Point2f(-sonImg.cols/2.0, -sonImg.rows),
//           pts);
  p << pts;
  return true;
}

Mat SonMatch::get2DTransform(double x, double y, double theta)
{
  Mat M = getRotationMatrix2D(Point2f(0.0,0.0),
                                 theta,1.0);

  M.at<double>(0,2) = x;
  M.at<double>(1,2) = y;
  return M;
}

bool SonMatch::reset()
{
  //Reset
  sons.clear();
}

void SonMatch::setupROS()
{
  pubBinImg = it.advertise("/"+name+"_bin",1);
  pubMagImg = it.advertise("/"+name+"_mag",1);
  pubSonFeatures = it.advertise("/"+name+"_features",1);
  pubSonPts = it.advertise("/"+name+"_pts",1);
  pubSonAligment = it.advertise("/"+name+"_aligment",1);
}

SonMatch::SonMatch():
  sons(6), // Looking past 3 son imgs to find the best match
  it(nh)
{
  sonMask = imread("/home/auros/ros/netuno_ws/SonEnhancementResults/son_mask.png",IMREAD_GRAYSCALE);
  sonPattern = imread("/home/auros/ros/netuno_ws/SonEnhancementResults/mean_img.png",IMREAD_GRAYSCALE);

  pyrDown(sonMask,sonMaskHalf);

  minMaxLoc(sonPattern,&patternMin,&patternMax,
            0,0,sonMask);

  pcMatch.reset(new ICPMatch());
}

bool SonMatch::newImg(Mat &img,
                   Mat &transform)
{
  double x, y, theta;

  transform = Mat::eye(3,3,CV_32FC1);

//  static int jump =0;
//  jump++;
//  if(jump < 10)
//    return false;
//  jump=0;

  x = y = theta = 0.0;
  if(img.empty())
  {
    cout << "Empty son image!!" << endl;
    return false;
  }

  Mat tmpImg;
//  pyrDown(img,tmpImg);
  tmpImg = img;

  if(sons.empty())
  {
    // Compute image scale
    s2M = 50.0/tmpImg.rows; // Scale from pixel to meters

    // Get img Size
    imgSize = Size(tmpImg.cols,tmpImg.rows);
  }

  Mat binImg,
      magImg,
      angImg,
      sonFeatures;

  PointCloudPtr cloud_A,
                cloud_B (new PointCloud);

  bool imgDescription=false, imgMatch=false;

  if( describe(tmpImg,cloud_B,
           &binImg, &magImg,
           &angImg, &sonFeatures) == true )
  {
    imgDescription=true;
  }

  if(showImgs)
  {
    // Pub bin image
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", binImg).toImageMsg();
    pubBinImg.publish(msg);

    // Pub MagImg
    normalize(magImg,magImg,
              0,255,
              NORM_MINMAX,CV_8UC1);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", magImg).toImageMsg();
    pubMagImg.publish(msg);

    // Pub son features
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sonFeatures).toImageMsg();
    pubSonFeatures.publish(msg);
  }

  if(!sons.empty() && imgDescription==true)
  {
    Mat m,bestM;
    MatchTransform t,bestT;
    double bestScore = 99999.9;
    int bestSon=-1;

    // Find best match
    for(uint i =0 ; i < sons.size(); i++)
    {
      cloud_A = sons[i].cloud;

      if(pcMatch->match(cloud_A, cloud_B,
               &t,&m))
      {
        if(t.score < bestScore)
        {
          bestSon=i;
          bestScore = t.score;
          bestT=t;
          bestM=m;
        }
      }
    }

    // Register img
    if(bestSon>=0)
    {
      x = sons[bestSon].x + bestT.tx;
      y = sons[bestSon].y + bestT.ty;
      theta = sons[bestSon].heading + bestT.dTheta;

      m.copyTo(transform);


      if(showImgs)
      {
        Mat screen;
        drawPoints(Size(tmpImg.cols,tmpImg.rows),
                 screen,bestM,
                 cloud_A,cloud_B);

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();
        pubSonPts.publish(msg);

        drawAligment(sons[bestSon].img,tmpImg,
                     bestM, screen);

        putText(screen,
            format("Best: %d Score: %g",bestSon, t.score),
            Point(20,40),FONT_HERSHEY_PLAIN,3.0,
            Scalar(255,0,255),2);

//        putText(screen,
//            format("%.2f %.2f %.1f",x*s2M,y*s2M,theta*180.0/M_PI),
//            Point(20,80),FONT_HERSHEY_PLAIN,3.0,
//            Scalar(255,0,255),2);

        putText(screen,
            format("%.2f %.2f %.1f",bestT.tx,bestT.ty,theta*180.0/M_PI),
            Point(20,80),FONT_HERSHEY_PLAIN,3.0,
            Scalar(255,0,255),2);

        // Pub son alignment
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();
        pubSonAligment.publish(msg);

      }

      // Add image
      SonImg sonImg;
      sonImg.cloud = cloud_B;
      sonImg.x = x;
      sonImg.y = y;
      sonImg.heading = theta;
      tmpImg.copyTo(sonImg.img);
      sons.addImg(sonImg);
      imgMatch=true;
    }

  }else if(sons.empty() && imgDescription==true)
  {
    // Add image
    SonImg sonImg;
    sonImg.cloud = cloud_B;
    sonImg.x = x;
    sonImg.y = y;
    sonImg.heading = theta;
    tmpImg.copyTo(sonImg.img);
    sons.addImg(sonImg);
    imgMatch=true;
  }

  if(imgDescription==false)
    reset();
  cout << "x: " << x << " y: " << y << " theta: " << theta << endl;
//  x=y= 0.0;
//  waitKey(1);

  return imgDescription && imgMatch;
}

void SonMatch::setName(string name)
{
  this->name = name;
}

void SonMatch::setPointCloudMatch(PointCloudMatch::Ptr pcM)
{
  pcMatch=pcM;
}
