#include "SonMatch.h"

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <iostream>


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

double SonMatch::match(PointCloudPtr cloud_A,
                       PointCloudPtr cloud_B)
{

}

void SonMatch::setupICP()
{
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);

  // Set the maximum number of iterations (criterion 1)
//  icp.setMaximumIterations (maximumIterations);
  // Set the transformation epsilon (criterion 2)
//  icp.setTransformationEpsilon (transformationEpsilon);
  // Set the euclidean distance difference epsilon (criterion 3)
  //  icp.setEuclideanFitnessEpsilon (euclideanFitnessEpsilon);
}

bool SonMatch::describe(Mat &sonImg, PointCloudPtr p,
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
            0.5,255,THRESH_BINARY);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  binImg.convertTo(binImg,CV_8UC1);

  imshow("binSon",binImg);

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
    if(fs[i].first < 10) break;

    pts.push_back(fs[i].second);
  }

  if(pSonFeatures!= nullptr)
  {
    Mat sonFeatures;
    cvtColor(sonImg,sonFeatures,COLOR_GRAY2BGR);

    for(uint i = 0; i < pts.size();i++)
    {
      circle(sonFeatures,pts[i],5,
             Scalar(255,0,255),2);
    }

    imshow("Features", sonFeatures);
  }

  if(pts.size() < 10)
    return false;

  p = new PointCloud();
  p << pts;
  return true;
}

SonMatch::SonMatch()
{
  string nw("BlobSetting");

  namedWindow(nw);

  cv::createTrackbar("minArea",nw,
      &minA,1000);

  cv::createTrackbar("maxArea",nw,
      &maxA,1000);

  cv::createTrackbar("minC",nw,
      &minC,100);

  cv::createTrackbar("maxC",nw,
      &maxC,100);

  cv::createTrackbar("minConv",nw,
      &minConv,100);

  cv::createTrackbar("maxConv",nw,
      &maxConv,100);

  cv::createTrackbar("minInn",nw,
      &minInn,100);

  cv::createTrackbar("maxInn",nw,
      &maxInn,100);

  cv::createTrackbar("Contrast",nw,
      &contrast,200);

  sonMask = imread("/home/auros/ros/netuno_ws/SonEnhancementResults/son_mask.png",IMREAD_GRAYSCALE);
  sonPattern = imread("/home/auros/ros/netuno_ws/SonEnhancementResults/mean_img.png",IMREAD_GRAYSCALE);

  pyrDown(sonMask,sonMaskHalf);
  minMaxLoc(sonPattern,&patternMin,&patternMax,
            0,0,sonMask);

  criteria = cv::TermCriteria((cv::TermCriteria::COUNT)
                              + (cv::TermCriteria::EPS),
                              10, 0.03);
  lkWinSz = cv::Size(15,15);
  lkMaxLvl = 8;

  // PCL ICP parameters
  setupICP();

}

void SonMatch::newImg4(Mat &img)
{
  Mat binImg,tmpImg;

//  subtract(img,sonPattern,img,sonMask,CV_8UC1);

//  normalize(img,img,0,255,NORM_MINMAX);

  blur(img,tmpImg,Size(7,7));

  threshold(tmpImg,binImg,200,255,THRESH_BINARY);

  imshow("bin", binImg);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  findContours(binImg, contours,
               hierarchy, RETR_EXTERNAL,
               CHAIN_APPROX_NONE);

  // draw contours on the original image
  Mat screen;
  cvtColor(img,screen,COLOR_GRAY2BGR);

  vector<Point2f> mc( contours.size() );

  for(uint i = 0; i < contours.size();i++)
  {
      const Moments &m = moments(contours[i]);

      mc[i] = Point2f( static_cast<float>(m.m10 / (m.m00 + 1e-5)),
                       static_cast<float>(m.m01 / (m.m00 + 1e-5)) );

      if(contourArea(contours[i])> 53)
        circle(screen,mc[i],5,Scalar(0,255,0),-1);
  }


//  drawContours(screen, contours,
//               -1, Scalar(0, 255, 0),
//               2);

  imshow("SonContours", screen);
  waitKey(1);

}

void SonMatch::newImg(Mat &img)
{
//  static int jump=0;
//  if(jump < 10)
//  {
//    jump++;
//    return;
//  }
//  jump =0;

  Mat fimg,tmpImg;
  pyrDown(img,tmpImg);

  tmpImg.convertTo(fimg,CV_32F,1/255.0);

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
            0.5,255,THRESH_BINARY);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  binImg.convertTo(binImg,CV_8UC1);

  imshow("binSon",binImg);

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

  Mat screen;
  cvtColor(tmpImg,screen,COLOR_GRAY2BGR);

  vector<Point2f> pts;
  pts.reserve(fs.size());
  for(uint i = 0; i < fs.size();i++)
  {
    if(fs[i].first < 10) break;

    pts.push_back(fs[i].second);
    circle(screen,fs[i].second,5,Scalar(255,0,255),2);
  }

//  drawContours(screen, contours,
//               -1, Scalar(0, 255, 0),
//               2);

  imshow("Features", screen);

  if(prv_pts.size() > 10 && pts.size() > 10)
  {
    // Match points
    PointCloudPtr cloud_A (new PointCloud);
    PointCloudPtr cloud_B (new PointCloud);

    vector<cv::Point2f> tPts;

//    addPoint(prv_pts,Point2f(-tmpImg.cols/2,-tmpImg.rows),
//             aPts);
//    addPoint(pts,Point2f(-tmpImg.cols/2,-tmpImg.rows),
//             bPts);
//    addPoint(prv_pts,Point2f(0.0,0.0),
//             aPts);
//    addPoint(pts,Point2f(0.0,0.0),
//             bPts);

    cloud_A << prv_pts;
    cloud_B << pts;

    icp.setInputSource(cloud_A);
    icp.setInputTarget(cloud_B);

    PointCloud Final;
    icp.align(Final);

//    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//    icp.getFitnessScore() << std::endl;

    Eigen::Matrix<float,4,4> m = icp.getFinalTransformation();
    std::cout << m << std::endl;

    cv::Mat mCV(2,3,CV_32F);
    mCV.at<float>(0,0) = m(0,0);
    mCV.at<float>(0,1) = m(0,1);
    mCV.at<float>(0,2) = m(0,3);
    mCV.at<float>(1,0) = m(1,0);
    mCV.at<float>(1,1) = m(1,1);
    mCV.at<float>(1,2) = m(1,3);

    double sx,sy,psi,tx,ty;
    tx = m(0,3);  ty = m(1,3);
    sx = sqrt(m(0,0)*m(0,0) + m(0,1)*m(0,1));
    sy = sqrt(m(1,0)*m(1,0) + m(1,1)*m(1,1));

    if(m(0,0)<0.0) sx =-sx;
    if(m(1,1)<0.0) sy =-sy;

    psi = -atan2(m(1,0),m(1,1));

    cout << "sx: " << sx << " sy: " << sy
         << endl
         << "tx: " << tx << " ty: " << ty
         << endl
         << "psi: " << psi*180.0/M_PI << endl;

    x += tx; y+= ty; theta += psi*180.0/M_PI;

//    cout << mCV << endl;
    transform(prv_pts,tPts,mCV);

//    addPoint(tPts,Point2f(tmpImg.cols/2,tmpImg.rows),
//             tPts);

    Mat imT(tmpImg.rows,tmpImg.cols,CV_8UC3,
            cv::Scalar(0,0,0));

    for(unsigned i = 0 ; i < tPts.size(); i++)
    {
//        circle(imT,prv_pts[i],3,Scalar(0,255,0),-1);
      circle(imT,tPts[i],3,Scalar(255,0,255),-1);
    }

    for(unsigned i = 0 ; i < pts.size(); i++)
    {
      circle(imT,pts[i],3,Scalar(255,0,0),-1);
    }

    imshow("Result",imT);

    Mat m2CV = getRotationMatrix2D(Point2f(0.0,0.0),psi*180.0/M_PI,1.0);

    m2CV.at<double>(0,2) = tx;
    m2CV.at<double>(1,2) = ty;

    cout << mCV << endl
         << m2CV << endl;

    Mat sbgr[3];

//    sbgr[0] = Mat(tmpImg.rows,tmpImg.cols,CV_8UC1,Scalar(0));

    warpAffine(prv_img,sbgr[0],m2CV,
               Size(prv_img.cols,prv_img.rows));

    sbgr[1] = tmpImg;

    warpAffine(prv_img,screen,mCV,
               Size(prv_img.cols,prv_img.rows));

    sbgr[2] = screen;

    merge(sbgr,3,screen);

    putText(screen,
        format("%g",icp.getFitnessScore()/min(tPts.size(),pts.size())),
        Point(20,20),FONT_HERSHEY_PLAIN,1.0,Scalar(255,0,255));

    putText(screen,
        format("%.2f %.2f %.1f",x,y,theta),
        Point(20,40),FONT_HERSHEY_PLAIN,1.0,Scalar(255,0,255));

    imshow("TransformSon",screen);

  }
  waitKey(1);

  prv_pts = pts;
  tmpImg.copyTo(prv_img);
}

void SonMatch::newImg3(Mat &img)
{

  if(img.empty()) return;

  if(prv_img.empty())
  {
    img.copyTo(prv_img);
    return;
  }

  Mat diff;
  blur(img,diff,Size(7,7));

//  cv::absdiff(img,prv_img,diff);

//  normalize(diff,diff,0,255,NORM_MINMAX);
//  double minV,maxV;
//  minMaxLoc(diff,&minV,&maxV,0,0,sonMask);

//  subtract(maxV,diff,diff,sonMask,CV_8UC1);

//  diff.convertTo(diff,CV_16FC1,1.0/maxV,0.0);

//  multiply(img,diff,diff,1.0,0.0);

  imshow("Diff", diff);
  waitKey(1);

  img.copyTo(prv_img);
}

void SonMatch::newImg2(Mat &trackImg)
{
  if(trackImg.empty()) return ;

//  double s= 1080.0/img.cols,
//        is= 1.0/s;
//  resize(img,trackImg,Size(),s,s);
  double s =1.0,is=1.0;

  if(ps.size() == 0)
  { // Initialize grid flow

    nGrid=30,
    gW=trackImg.cols/(nGrid+2),
    gH=trackImg.rows/(nGrid+2);

    for(int i = 0; i < nGrid;i++)
    {
      for(int j = 0; j < nGrid;j++)
      {
        ps.push_back(Point2f((j+1)*gW,(i+2)*gH));
      }
    }
    m.resize(nGrid,vector<Point2f>(nGrid));
    mStatus.resize(nGrid,vector<char>(nGrid,0));
  }

  if(!prv_img.empty())
  { // Compute optical flow
    vector<uchar> status; // Define if the feature has or not has match
    vector<float> err; // Match error

    Mat screen;
    cvtColor(trackImg,screen,COLOR_GRAY2BGR);

    vector<Point2f> &p0=ps,
                    p1;

    // Calculate optical flow
    calcOpticalFlowPyrLK(prv_img, trackImg,
                         p0, p1,
                         status,err,
                         lkWinSz, lkMaxLvl,
                         criteria);

    double min=999999, max=0;
    for(uint i = 0; i < p0.size();i++)
    {
      int gi=i/nGrid,
          gj=i%nGrid;

      if(status[i] && err[i] < 2.5)
      {
        if(err[i] > max) max = err[i];
        if(err[i] < min) min = err[i];

        line(screen,p0[i]*is,p1[i]*is,
             Scalar(0,0,255),4);
        circle(screen,p1[i]*is,5,Scalar(255,255,0),-1);

        m[gi][gj] = (p1[i]-p0[i])*is;
        mStatus[gi][gj] = true;
      }else
      {
        m[gi][gj] = Point2f(0,0);
        mStatus[gi][gj] = false;
      }
    }

    // Draw min max flow error
//    minMaxIdx(err,&min,&max);
    string minMaxStr = format("%.1f, %.1f",min,max);

    putText(screen,minMaxStr,Point(290, 50),
            FONT_HERSHEY_COMPLEX,
            2,Scalar(255, 0, 255), 2);

    imshow("Tracking", screen);
    waitKey(1);
  }

  // Save for next iteration
  trackImg.copyTo(prv_img);
}

void SonMatch::newImg1(Mat &img)
{

  p.filterByColor=false;

  p.filterByArea=true;
  p.minArea = minA;
  p.maxArea = maxA;

  p.filterByCircularity=false;
  p.minCircularity = minC/100.0;
  p.maxCircularity = maxC/100.0;

  p.filterByConvexity=false;
  p.minConvexity = minConv/100.0;
  p.maxCircularity = maxConv/100.0;

  p.filterByInertia=false;
  p.minInertiaRatio = minInn/100.0;
  p.maxInertiaRatio = maxInn/100.0;

  Ptr<SimpleBlobDetector> d = SimpleBlobDetector::create(p);

  // Detect blobs.
  std::vector<KeyPoint> keypoints;
  d->detect( img, keypoints);

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size
  // of the circle corresponds to the size of blob
  Mat im_with_keypoints;
  drawKeypoints( img, keypoints,
                 im_with_keypoints,
                 Scalar(0,0,255),
                 DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  // Show blobs
  imshow("Blobs", im_with_keypoints );
  waitKey(1);

}
