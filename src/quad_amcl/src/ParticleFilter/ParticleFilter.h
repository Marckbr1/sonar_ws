#ifndef PARTICLE_FILTER
#define PARTICLE_FILTER

#include <opencv2/core.hpp>

// Satellite images
#include <SatelliteManager.h>

// Name spaces
using namespace cv;

struct Particle
{
  uint id;
  double x;
  double y;
  double theta;
  double weight;
};

bool ascendingParticles(const Particle& i,const Particle& j);
bool descendingParticles(const Particle& i,const Particle& j);

// == Aux Map <--> Particle functions ====
bool inWater(const Particle &p ,const SatelliteManager &map);
bool inMap(const Particle &p, const SatelliteManager &map,
           double sonBearing=2.26893 /*130deg*/,
           double sonRange=50.0);
bool seeSomething(const Particle &p, const SatelliteManager &map,
                double sonBearing=2.26893 /*130deg*/,
                double sonRange=50.0);

// Struct representing one position/control measurement.
struct control_s
{
  // Velocity [m/s]
  double velocity;
  // Yaw rate [rad/s]
  double yawrate;
};

// Struct representing one ground truth position.
struct ground_truth
{
  // Global vehicle x position [m]
  double x;
  // Global vehicle y position
  double y;
  // Global vehicle yaw [rad]
  double theta;
};

// Struct representing one landmark observation measurement.
struct LandmarkObs
{
  // Id of matching landmark in the map.
  int id;
  // Local (vehicle coordinates) x position of landmark observation [m]
  double x;
  // Local (vehicle coordinates) y position of landmark observation [m]
  double y;
};

class ParticleFilter
{
public:
  ParticleFilter();
  virtual ~ParticleFilter(){}

  virtual void setMap(const SatelliteManager *map);

  virtual bool init(double x, double y, double theta, double std[],
            double time,
            double minX=0.0,double minY=0.0,
            double maxX=1000.0, double maxY=1000.0);
  
            // virtual bool init(double x, double y, double theta, double std[],
            // double time,
            // double minX=0.0,double minY=-800.0,
            // double maxX=1000.0, double maxY=1000.0);

  virtual void prediction(double time, double std_pos[]);

  virtual void getMinMaxWieght(double &minW, double &maxW);

  virtual void updateWeights(vector<float> weights);

  virtual void updateVelocity(double time, double vx, double vy, double vYaw);
  virtual void updatePose(double time, double x, double y, double yaw);
  virtual void forceOrientation(double yaw);

  virtual void resample(double std[]);

  virtual void getBestParticle(Particle &p);
  virtual int getBestParticleId();

  virtual void getBestAverageParticle(int n,Particle &p);


  virtual const vector<Particle> & getParticles();

  virtual const bool initialized() const
  {
    return false;
  }
};

#endif // PARTICLE_FILTER
