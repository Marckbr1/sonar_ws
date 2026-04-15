#include "SonEnhancementNode.h"
//#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "PointCloudMatch/FGRMatch.h"

#include <random>
//#include <omp.h>

bool SonEnhancement::setup()
{
  // Set 2 thread on opem MP
//  omp_set_thread_num(2);

  sonMap.setName("icp_map");
  sonMap.setupROS();

  sonMatch.setName("icp_match");
  sonMatch.setupROS();

//  sonMap2.setName("fgr_map");
//  sonMap2.setupROS();

  sonMatch2.setName("fgr_match");
  sonMatch2.setupROS();

  sonMatch2.setPointCloudMatch(make_shared<FGRMatch>());

  // Load son mask image
  sonMask = imread("/home/auros/ros/netuno_ws/SonEnhancementResults/son_mask.png",IMREAD_GRAYSCALE);

  // Parameters
  string map_path;

  if(!pnh.getParam("map_path",map_path))
  {
    ROS_ERROR("Parameter ~map_path not defined!");
    return false;
  }
  mapPath = map_path;

  pnh.param("disable_orientation",disableOrientation,false);

  if(!sat.loadMap(mapPath.string()))
  {
    ROS_ERROR_STREAM("Could not load map " << map_path);
    return false;
  }
//  sat.setMapWhite();

  // Topic Subscription
  subSonImg = it.subscribe("/son",1,
        &SonEnhancement::sonCallback,this);

  subPose = nh.subscribe("/pose",1,
        &SonEnhancement::poseCallback,this);

//  subRank = nh.subscribe("/rank",1,
//        &SonEnhancement::rankCallback,this);

  // Topic advertisement
//  pubSatImgs = it.advertise("/sat_crop",300); // Particles view
//  pubSonImg = it.advertise("/son_small",3); // Sonar image

  pubMeanSon = it.advertise("/mean_son",3); // Enhanced son image
  pubMeanSonNoCompensation = it.advertise("/mean_son_no_compensation",3); // Enhanced son image without motion compesation

//  pubPose = nh.advertise<geometry_msgs::PoseStamped>("/pf_pose",1);

  pubMapImg = it.advertise("/map_img",1);
//  pubParticlesView = it.advertise("/particles_view",1);

  return true;
}


void SonEnhancement::sonCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if(!hasFirstPose) return;

  try
  {
    lastSon = cv_bridge::toCvShare(msg, "mono8")->image;
//    lastSon = cv_bridge::toCvShare(msg, "mono16")->image;
//    son = cv_bridge::toCvShare(msg, "bgr8")->image;
//    cv::imshow("View son", lastSonSmall);
//    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }

//  Mat son8bit;
//  lastSon.convertTo(son8bit,CV_8UC1);

//  imshow("Son8bit",son8bit);
//  waitKey(10);

//  lastSon = son8bit;

  // Son img size is 1553 x 848
  SonImg son;

  // Get pose from tracking
  if(sonMatch.newImg(lastSon,
         son.m) == false)
  {
    ROS_INFO("Reset sonMap from ICP!");
    sonMap.reset();
  }else
  {
    sonMap.newImg(lastSon,
           son.m);
  }

  // Get pose from tracking
//  if(sonMatch2.newImg(lastSon,
//         son.m) == false)
//  {
//    sonMap2.reset();
//  }else
//  {
//    sonMap2.newImg(lastSon,
//           son.m);
//  }

  // Last Ground Truth pose
  son.x = lastX;
  son.y = lastY;
  son.heading = lastYaw;

  lastSon.copyTo(son.img);
  sonImgs.addImg(son);

//  #pragma omp parallel
//  {
//    if(omp_get_thread_num()==0)
//    {
      tryEnhancement();
//    }else if(omp_get_thread_num()==1)
//    {
      tryNoCompensationEnhancement();
//    }
//  }
    updateMapView();
//  if(sonImgs.size()>3)
//  {
//    SonImg son2= sonImgs[2];
//    double minv,maxv;
//    minMaxLoc(son2.img,&minv,&maxv);
//    cout << "test min " << minv << " max " << maxv << endl;
//  }

//  ROS_INFO("Soncalback");
}

void SonEnhancement::poseCallback(const geometry_msgs::PoseStamped &msg)
{

  // We have to transform pose in velocity
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.orientation,q);

  tf::Matrix3x3 mat(q);
  double yaw, pitch, roll;
  mat.getEulerYPR(pitch,yaw, roll,0);

//  cout << "Orientation solution 1: " << endl
//       << "roll: " << roll*180.0/M_PI << endl
//       << "pitch: " << pitch*180.0/M_PI << endl
//       << "yaw: " << yaw*180.0/M_PI << endl;

  if(abs(roll) > 1.0)
    mat.getEulerYPR(pitch,yaw, roll,1);

//  cout << "Orientation solution 2: " << endl
//       << "roll: " << roll*180.0/M_PI << endl
//       << "pitch: " << pitch*180.0/M_PI << endl
//       << "yaw: " << yaw*180.0/M_PI << endl;

//  mat.getRPY(roll,pitch,yaw);
//  cout << "Orientation 2way " << yaw*180.0/M_PI << endl;

  double time = msg.header.stamp.toSec(),
      dt = time-lastTime,
      px = msg.pose.position.x,
      py = msg.pose.position.y,
      dx = px-lastX,
      dy = py-lastY,
      dYaw = yaw-lastYaw;
  // Getn smaller ang diff
  if(dYaw > M_PI)
    dYaw-= 2*M_PI;
  else if(dYaw < -M_PI)
    dYaw+= 2*M_PI;

  lastX = px;
  lastY = py;
  lastYaw = yaw;
  lastTime = time;
  hasFirstPose = true;

  //  ROS_INFO("PoseCalback");
}

void SonEnhancement::getTransformedSonImg(Size imgSize, Mat sonImg,
                                         Point p, double deg,
                                         Mat &sonImgResult, Mat &sonMaskResult)
{
  // SonImg must be one channel grey sacele image

  // Get rotation matrix of the son image
  Mat R = getRotationMatrix2D(
           Point(sonImg.cols/2,sonImg.rows),
           deg,1.0);
  // R type is CV_64F

  // Translate to the center of the son image
  R.at<double>(0,2) -= sonImg.cols/2;
  R.at<double>(1,2) -= sonImg.rows;

  // Apply image Translation
  R.at<double>(0,2) += p.x;
  R.at<double>(1,2) += p.y;

//  cout << "Rot type: " << R.type() << " - " << CV_64F << endl;

  // Warp son image
  warpAffine(sonImg,sonImgResult,R,
             imgSize);

  // Warp son mask
  warpAffine(sonMask,sonMaskResult,R,
             imgSize);
}

void SonEnhancement::getTransformedSonImg(Size imgSize, Mat sonImg,
                                          const Mat &transform,
                                          Mat &sonImgResult, Mat &sonMaskResult)
{
  // Warp son image
  warpAffine(sonImg,sonImgResult,
             transform(Rect(0,0,3,2)),
             imgSize);

  // Warp son mask
  warpAffine(sonMask,sonMaskResult,
             transform(Rect(0,0,3,2)),
             imgSize);
}

void SonEnhancement::updateMapView()
{
  Mat screen = sat.getMapImg();

//  double scale = min(sat.UTM2Img_.x,sat.UTM2Img_.y);

  string txt;

  // Draw particles
  for(uint i = 0; i < sonImgs.size();i++)
  {
    const SonImg &p = sonImgs[i];

    double normW=1.0;
    normW=double(i)/double(sonImgs.size());
    const Vec3b& cPix = colors.at<Vec3b>(0,normW*255);


    drawSonImg(screen,
               p,txt,
               Scalar(cPix[0],cPix[1],cPix[2]),2);

    // Particles Color version
//    double normW=1.0;
//    if(maxW != minW)
//      normW=(p.weight-minW)/(maxW-minW);
//    const Vec3b& cPix = colors.at<Vec3b>(0,normW*255);

//    drawParticle(screen,
//                 p,txt,
//                 Scalar(cPix[0],cPix[1],cPix[2]),2);
  }

  screen = screen(Rect(2060,100,1768,1236));
//  cout << screen.size();
  // Resize img
  double s = 900.0/screen.cols;
  resize(screen,screen,Size(),s,s);

  // Show img on screen
  if(showImgMap)
  {
    imshow("Test", screen);
    waitKey(10);
  }

  // Publish msg
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();
//  msg->header.stamp = time;
  pubMapImg.publish(msg);
}

void SonEnhancement::tryEnhancement()
{
  if(sonImgs.size() == 0) return;

  double scale = 848.0/50.0; // Img height / 50 meters
  //  meters * scale = pixels

  SonImg m = sonImgs.last();

  int rows=m.img.rows,
      cols=m.img.cols;

  Mat accSon(rows,cols,
             CV_64FC1,
             Scalar(0.0));

//  cout << "Channels: " << m.img.channels()
//       << " against : " << accSon.channels()
//       << endl;

//  cout << "Type 1: " << accSon.type()
//       << " Type 2: " << m.img.type()
//       << " CV_64FC1: " << CV_64FC1
//       << " CV_8UC1: " << CV_8UC1
//       << endl;

//  cout << "Size1: " << accSon.size()
//       << " Size2: " << m.img.size()
//       << endl;

//  cout << "OpenCV version : " << CV_VERSION << endl;
//  cout << "Major version : " << CV_MAJOR_VERSION << endl;
//  cout << "Minor version : " << CV_MINOR_VERSION << endl;
//  cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;

  add(accSon,m.img,accSon,sonMask,CV_64FC1);

  Size globalImSz(m.img.cols,m.img.rows);
  Point globalOrigin(globalImSz.width/2.0,globalImSz.height);
  Mat tResult, tMask;

  for(uint i = 0; i < sonImgs.size()-1;i++)
  {
    SonImg &p = sonImgs[i];
    double dh = m.heading - p.heading,
           dx = m.x - p.x,
           dy = m.y - p.y,
           dpx = dx*scale,
           dpy = dy*scale;

//    getTransformedSonImg(globalImSz,
//                         p.img,
//                         globalOrigin-Point(dpx,dpy),
//                         dh*180.0/M_PI,
//                         tResult,tMask);

    getTransformedSonImg(globalImSz,
                         p.img,
                         p.m,
                         tResult,tMask);

//    imshow("tResult",tResult);
//    imshow("tMask",tMask);
//    waitKey(0);

//    cout << "Channels 2: " << p.img.channels() << endl;
//    accSon += p.img;
    add(accSon,tResult,accSon,tMask,CV_64FC1);

//    Mat mean = accSon/ (i+2);
//    mean.convertTo(mean,CV_8UC1);
//    imshow("Mean_Son",mean);
//    waitKey(0);
  }

  Mat mean = accSon/ sonImgs.size();
  mean.convertTo(mean,CV_8UC1);
//  imshow("Mean_Son",mean);
//  waitKey(2);

  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", mean).toImageMsg();

  pubMeanSon.publish(msg);
}

void SonEnhancement::tryNoCompensationEnhancement()
{
  if(sonImgs.size() == 0) return;

  double scale = 848.0/50.0; // Img height / 50 meters
  //  meters * scale = pixels

  SonImg m = sonImgs.last();

  int rows=m.img.rows,
      cols=m.img.cols;

  Mat accSon(rows,cols,
             CV_64FC1,
             Scalar(0.0));

  add(accSon,m.img,accSon,sonMask,CV_64FC1);

  Size globalImSz(m.img.cols,m.img.rows);
  Point globalOrigin(globalImSz.width/2.0,globalImSz.height);
  Mat tResult, tMask;

  for(uint i = 0; i < sonImgs.size()-1;i++)
  {
    SonImg &p = sonImgs[i];

    add(accSon,p.img,accSon,tMask,CV_64FC1);
  }

  Mat mean = accSon/ sonImgs.size();
  mean.convertTo(mean,CV_8UC1);

  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", mean).toImageMsg();

  pubMeanSonNoCompensation.publish(msg);
}

//void SonEnhancement::add(Mat &src, Mat &dst)
//{
//  for(uint i =0; i < src.rows; i++)
//    for(uint j =0; j < src.cols; j++)
//    {
////      if(i%100==0) cout << dst.at<float>(i,j) << endl;
//      dst.at<float>(i,j) += src.at<uchar>(i,j);
////      if(i%100==0) cout << dst.at<float>(i,j) << endl;
//    }
////  double minv,maxv;
////  minMaxLoc(src,&minv,&maxv);
////  cout << "test min " << minv << " max " << maxv << endl;
////  minMaxLoc(src,&minv,&maxv);
////  cout << "min " << minv << " max " << maxv << endl;
//}

//void SonEnhancement::toFinalImg(Mat &src, Mat &dst, int n)
//{
//  // Compute average
//  for(uint i =0; i < src.rows; i++)
//    for(uint j =0; j < src.cols; j++)
//    {
////      if(i%100==0) cout << src.at<float>(i,j) << endl;
//      src.at<float>(i,j) /= float(n);
////      if(i%100==0) cout << src.at<float>(i,j) << endl;
//    }

//  dst = Mat(src.rows,src.cols,CV_8UC1);

//  // Convert to uint8
//  src.convertTo(dst,CV_8UC1);
//}

//void SonEnhancement::copy(Mat &src, Mat &dst)
//{
//  double minv,maxv;
//  minMaxLoc(src,&minv,&maxv);
//  cout << "test min " << minv << " max " << maxv << endl;
//  minMaxLoc(dst,&minv,&maxv);
//  cout << "test min " << minv << " max " << maxv << endl;

//  for(uint i =0; i < src.rows; i++)
//    for(uint j =0; j < src.cols; j++)
//    {
//      uint v = src.at<uchar>(i,j);
////      if(i%100==0) cout << src.at<float>(i,j) << endl;
//      dst.at<float>(i,j) = v;
////      if(i%100==0) cout << src.at<float>(i,j) << endl;
//    }
//}

void SonEnhancement::drawSonImg(Mat &img, const SonImg &p,
                                const string &txt, const Scalar &color,
                                int thickness)
{
  // Get the sonar field of view polygon on satellite image
  int nPts = 15;
  Point pts[nPts];

  sat.getSonarPolyOnImg(Point2d(p.x,p.y),p.heading*180.0/M_PI,
                        pts,nPts,sonarRange,sonarFoV);

  int npts[] = {nPts};
  const Point* ppt[1] = { pts };

  polylines(img,
            ppt,npts,
            1,true,
            color,thickness);

  if(!txt.empty())
    putText(img,txt,
            pts[0],FONT_HERSHEY_DUPLEX,
            3.0,color,9);
}


SonEnhancement::SonEnhancement():
    pnh("~"),
    r(4.0),
    it(nh),
    colors(1,256,CV_8UC1)
{
  for(uint i =0 ;i < 256; i++)
    colors.at<uchar>(0,i) = i;

  applyColorMap(colors,colors,COLORMAP_JET);
}

bool SonEnhancement::start()
{
  // Setup ROS stuff
  // Load maps
  if(!setup()) return false;
//  ROS_INFO_STREAM("Setup completed!");

  ros::spin();

//  while(ros::ok())
//  {
//    if(sonImgs.size()>3)
//    {
//      SonImg son2= sonImgs.last();
//      double minv,maxv;
//      minMaxLoc(son2.img,&minv,&maxv);
//      cout << "test min " << minv << " max " << maxv << endl;
//    }

//    updateMapView();
//    tryEnhancement();
//    waitKey(200);
//    ros::spinOnce();
//  }

  return true;
}
