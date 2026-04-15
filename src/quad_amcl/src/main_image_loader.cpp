
// ROS lib (roscpp)
#include <ros/ros.h>
#include "ImageLoader.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "img_loader");

    ImageLoader im;
    im.start();

}
