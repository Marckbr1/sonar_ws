#ifndef DATA2BAG_H
#define DATA2BAG_H

// ROS Img transport plugin
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// OpenCV for image processing, loading and display
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Boos filesystem to search files in a folder
#include <boost/filesystem.hpp>

// C++ Standard libraries (STL)
#include <vector>

// ROS BAG
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseStamped.h>

#include "../libs/SatelliteManager/SatelliteManager.h"
#include "../libs/DatasetReader/DatasetReader.h"

// Name spaces
using namespace boost::filesystem;
using namespace cv;
using namespace std;

class Data2Bag
{
private:

  // Parameters
  bool stop;
  bool loadVideos;

  SatelliteManager sat;
  double sonarRange;

  // ROSBAG
  rosbag::Bag outBag_;

  // Methods  
  bool loadSonCorrection(unsigned dataFixerId,
            DatasetFrame &fr);

  Mat loadSatImg(DatasetFrame &fr,
                 int rows, int cols);

  Mat loadSonImg(int frameId);

  bool verifyPaths();

  // ROS BAG
  void writeCompressedImg(const string &name,
                          std_msgs::Header &h,
                          cv::Mat &m);

  void writeImg(std::string name,
                std_msgs::Header &h,
                cv::Mat &m);

public:
    Data2Bag();

    path fixerFilesPath;
    path sourceData;
    path mapPath;
    path outBagFileName;

    void start();
    bool loadDataset(vector<DatasetFrame> &frames);
};

#endif // DATA2BAG_H
