
// ROS lib (roscpp)
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// ROS Img transport plugin
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

// C++ Limits
#include <limits>

// std iostream
#include <iostream>
using namespace std;

class Son2Range
{
  // ROS stuff
  ros::NodeHandle n;
  image_transport::ImageTransport it;

  ros::Publisher range_pub;
  image_transport::Subscriber son_sub;

  // Sonar stuff
  #define N_BEAMS 256 // 256 beams
  Point2d beam[N_BEAMS];
  double binIni=100;
  double sonMaxRange=50.0; // 50 meters
  int valueThreshold=254;
  double bearing=130; // 130 degrees


  void initBeamVecs()
  {
    if(N_BEAMS<=1) return;

    double halfBearing=bearing/2.0,
        rate;

    for(int i = 0; i < N_BEAMS; i++)
    {
      if(i==0) rate = 0;
      else rate = (i/ (N_BEAMS-1.0));

      double bDeg = rate*bearing - halfBearing,
             bRad = bDeg*M_PI/180.0;

      beam[i] = Point2d( -sin(bRad), -cos(bRad));
    }
  }

  void sonCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    Mat sonImg;
    try
    {
      sonImg = cv_bridge::toCvShare(msg, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }

//    Mat sonBlur;
//    blur(sonImg,sonBlur,Size(3,3));

    // Find range measurements
    uint nBins = sonImg.rows;
    Point2d center(sonImg.cols/2.0,sonImg.rows);

    sensor_msgs::LaserScan lmsg;

    lmsg.header = msg->header;
    lmsg.header.frame_id = "son";

    double bearingRad=(bearing*M_PI/180.0);
    lmsg.angle_max = bearingRad/2.0;
    lmsg.angle_min = -lmsg.angle_max;
    lmsg.angle_increment = bearingRad/(N_BEAMS-1);

    lmsg.time_increment = 0.0;

    lmsg.scan_time = 0.0;

    lmsg.range_min = 0.01;
    lmsg.range_max = sonMaxRange;

    lmsg.ranges.resize(N_BEAMS);
    lmsg.intensities.resize(N_BEAMS);

    Point2d lastP, p;

    // For each BEAM i
    for(uint i = 0; i < N_BEAMS;i++)
    {
      lmsg.ranges[i] = numeric_limits<double>::infinity();
      lmsg.intensities[i] = 0.0;

      // For each bin j
      for(uint j = binIni+1; j < nBins;j+=4)
      {
        if(j == 1)
        {
          lastP = center;
        }
        else
        {
          lastP = center + double(j-1.0) * beam[i];
        }
        p = center + double(j) * beam[i];

        uchar value = sonImg.at<uchar>(round(p.y),round(p.x));
          //   lastValue = sonImg.at<uchar>(round(lastP.y),round(lastP.x));


//        if(abs(lastValue-value) >= valueThreshold)
        if(value >= valueThreshold)
        {
          if(j==0)
            lmsg.ranges[i] = lmsg.range_min;
          else
            lmsg.ranges[i] = (double(j)/double(nBins-1)) * lmsg.range_max;

          lmsg.intensities[i] = value;
          break;
        }

      }
    }

    range_pub.publish(lmsg);

  }

  void initROS()
  {
    range_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

    son_sub = it.subscribe("/son",1,
                           &Son2Range::sonCallback,this);

  }
public:
  Son2Range():
    it(n)
  {
    initBeamVecs();
    initROS();

  }

  void start()
  {
    ros::spin();
  }
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "son_to_range");
  Son2Range s2r;
  s2r.start();

}
