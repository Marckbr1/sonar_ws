#ifndef ODOMETRY
#define ODOMETRY

#include <opencv2/core.hpp>

// Name spaces
using namespace cv;

class Odometry
{
  Point2d p; // Current position
  double theta;
  Point2d v; // Current velocity
  double wTheta;

  double currentTime=0;
  bool isIntialized=false;
public:
  Odometry();

  void setInitialPose(double time, double px, double py, double theta);

  void updateVelocity(double time,
                      double vx, double vy,
                      double wTheta);

  bool predict(double time,
               double &x, double &y,
               double &theta);

  void forceOrientation(double theta);

  bool initialized();
};

#endif // ODOMETRY
