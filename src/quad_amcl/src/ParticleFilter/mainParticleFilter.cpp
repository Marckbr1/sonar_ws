
// ROS lib (roscpp)
#include <ros/ros.h>
#include "ParticleFilterNode.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "quad_particle_filter");

    ParticleFilterNode pfn;
    pfn.start();

}
