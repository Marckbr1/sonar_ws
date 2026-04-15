#ifndef SIMPLE_PF
#define SIMPLE_PF

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

#include "ParticleFilter.h"

// Name spaces
using namespace boost::filesystem;
using namespace cv;
using namespace std;

class SimplePF: public ParticleFilter
{
private:
  // Number of particles to draw
  uint num_particles=0;

  // Pseudo random numbers generators
  mt19937 genMt19937;

  // Object of random number engine class that generate pseudo-random numbers
  default_random_engine gen;

  // Flag, if filter is initialized
  bool is_initialized=false;

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

  // Settings
  bool particlesOnWaterOnly=true; // Map need to be segmented (Water is blue 255,0,0)
  bool particleThaSeeSomethingOnly=true;

  const SatelliteManager *map;

public:
  // Set of current particles
  vector<Particle> particles;

  SimplePF();

  void setMap(const SatelliteManager *map);

  bool init(double x, double y, double theta, double std[],
            double time,
            double minX=0.0,double minY=0.0,
            double maxX=1000.0, double maxY=1000.0);

  bool isParticleOk(Particle &p);

  bool resampleNotOkParticles(double std[]);

  void prediction(double time, double std_pos[]);

  void getMinMaxWieght(double &minW, double &maxW);

  void updateWeights(vector<float> weights);

  void updateVelocity(double time, double vx, double vy, double vYaw);

  void forceOrientation(double yaw);

  /*
   * Resample particles with replacement with probability proportional to weight
   */
  void resample(double std[]);

  /*
   * Resample only the K worse particles
   */
  void resampleV2(double std[]);

  void normalizeWeights();

  Particle getNewParticle(double std[]);
  void newParticleFromBegin(Particle &p,double std[]);
  void newParticleFromRandomMap(Particle &p);
  bool newParticleFromBestEstimation(Particle &p,double std[]);

  bool newParticleFromPose(Particle &p,
                           double x, double y, double theta,
                           double std[]);

  void getBestParticle(Particle &p);
  int getBestParticleId();
  int getGoodParticleIdToClone();

  void getBestAverageParticle(int n,Particle &p);

  const vector<Particle> & getParticles();

  /*
   * Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const
  {
    return is_initialized;
  }

private:
  /*
   * Convert the passed in vehicle co-ordinates into map co-ordinates from
   * the perspective of the particle in question
   */
   LandmarkObs convertVehicleToMapCoords(LandmarkObs observationToConvert,
                                         Particle particle);
   /*
   * Finds which observations correspond to which landmark
   * (likely by using a nearest-neighbors data association).
   * @param landmarks: List of landmarks
   * @param observation: Current list of converted observation
   */
  vector<LandmarkObs> dataAssociation(/*vector<Map::single_landmark_s> landmarks,*/
                                      vector<LandmarkObs> observations);

};

#endif // SIMPLE_PF
