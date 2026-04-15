#ifndef PARTICLE_FILTER_TESTER
#define PARTICLE_FILTER_TESTER

// ROS lib (roscpp)
#include <ros/ros.h>

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

// Num of particles
#include <std_msgs/Int32.h>
#include <quad_amcl/StampedInteger.h>

// Random numbers
#include <random>

// Satellite images
#include <SatelliteManager.h>

// Name spaces
using namespace boost::filesystem;
using namespace cv;
using namespace std;

#include "ParticleFilter.h"

class ParticleFilterTester: public ParticleFilter
{
private:
  // Number of particles to draw
  uint num_particles=0;

  // Flag, if filter is initialized
  bool is_initialized=false;

  bool testOrientation=false; // Way it will create the particle, test orientation or translation

  // Vector of weights of all particles
  vector<double> weights;

  // Global velocity
  double vx,vy,vYaw;

  // Initialized pose
  double iniX, iniY, iniTheta;

  // Current time
  double currentTime=0.0; // The time is set on initialization

  // Field of interest defined from
  // sat image coverage
  double minX,minY,maxX,maxY;
  double sonaRange=50.0;
  double sonarBearing=130.0*M_PI/180.0;

  int resampleKWorst=3;

  bool inMap(double x, double y);
  bool inMap(Particle &p);
  bool inWater(double x, double y);
  bool inWater(Particle &p);

  const SatelliteManager *map;

public:
  // Set of current particles
  vector<Particle> particles;

  ParticleFilterTester(bool testOrientation=false);

  void createParticleLineFromPose(double x, double y, double theta,
                               vector<Particle> &ps);
  void createRotParticlesOnPosiotion(double x, double y, double theta,
                               vector<Particle> &ps);

  void setMap(const SatelliteManager *map);

  bool init(double x, double y, double theta, double std[],
            double time,
            double minX=0.0,double minY=0.0,
            double maxX=1000.0, double maxY=1000.0);

  bool resampleOutOfBoundParticles(double std[]);
  bool resampleOutOfWaterParticles(double std[]);

  void prediction(double time, double std_pos[]);

  void getMinMaxWieght(double &minW, double &maxW);

  void updateWeights(vector<float> weights);

  void updateVelocity(double time, double vx, double vy, double vYaw);
  void updatePose(double time, double x, double y, double yaw);

  /*
   * Resample particles with replacement with probability proportional to weight
   */
  void resample(double std[]);

  void normalizeWeights();

  void getBestParticle(Particle &p);
  int getBestParticleId();
  const vector<Particle> & getParticles();

  /*
   * Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const
  {
    return is_initialized;
  }

private:

};

#endif // PARTICLE_FILTER_TESTER
