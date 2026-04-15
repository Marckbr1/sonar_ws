#include "Odometry.h"

#include <iostream>
using namespace std;

Odometry::Odometry()
{
}

void Odometry::setInitialPose(double time,
                              double px, double py,
                              double theta)
{
  p = Point2d(px,py);
  this->theta = theta;
  currentTime = time;
  isIntialized=true;
}

void Odometry::updateVelocity(double time,
                              double vx, double vy,
                              double wTheta)
{
  double dt = time - currentTime;
  if(dt < 0.0)
  { // Invalid time
    cout << "Invalid time!" << endl;
    return;
  }

  double dx = v.x*dt,
         dy = v.y*dt; // Displacement on body frame

  double dYaw = this->wTheta*dt;

  // Convert body velocity to global velocity
  double co = cos(theta), so=sin(theta);
  p.x +=  dx*so -dy*co;
  p.y +=  dx*co +dy*so;
  this->theta += dYaw;

  // Set velocity on current time
  v.x = vx; v.y = vy;
  this->wTheta = wTheta;
  currentTime = time;
}

bool Odometry::predict(double time, double &x, double &y, double &theta)
{
  x = y = theta = 0.0;
  
  double dt = time - currentTime;
  if(dt < 0.0)
  { // Invalid time
    cout << "Invalid time!" << endl;
    return false;
  }

  if(dt == 0.0)
  {
    x=p.x;y=p.y;theta=this->theta;
    return true;
  }

  double dx = v.x*dt,
         dy = v.y*dt; // Displacement on body frame

  double dYaw = this->wTheta*dt;

  // Convert body velocity to global velocity
  double co = cos(theta), so=sin(theta);
  x = p.x + dx*so -dy*co;
  y = p.y + dx*co +dy*so;
  theta = this->theta + dYaw;
  return true;
}

void Odometry::forceOrientation(double theta)
{
  this->theta = theta;
  wTheta=0.0;
}

bool Odometry::initialized()
{
  return isIntialized;
}
