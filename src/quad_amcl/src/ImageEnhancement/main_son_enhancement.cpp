
// ROS lib (roscpp)
#include <ros/ros.h>
#include "SonEnhancementNode.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "son_enhancement_filter");

    SonEnhancement pfn;
    pfn.start();

}
