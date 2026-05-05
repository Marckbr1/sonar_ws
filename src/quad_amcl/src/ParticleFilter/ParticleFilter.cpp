#include "ParticleFilter.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>


bool ascendingParticles(const Particle &i, const Particle &j)
{
  return (i.weight<j.weight);
}

bool descendingParticles(const Particle &i, const Particle &j)
{
  return (i.weight>j.weight);
}

ParticleFilter::ParticleFilter()
{
}

void ParticleFilter::setMap(const SatelliteManager *map)
{
  cout << "Error, setMap() not implemented!" << endl;
}

bool ParticleFilter::init(double x, double y, double theta, double std[], double time,
                          double minX, double minY, double maxX, double maxY)
{
  cout << "Error, init() not implemented!" << endl;
  return false;
}

void ParticleFilter::prediction(double time,
                                double std_pos[])
{
  cout << "Error, prediction not implemented!" << endl;
}

void ParticleFilter::getMinMaxWieght(double &minW, double &maxW)
{
  minW = maxW = 0.0;
  cout << "Error getMinMaxWieght not implemented" << endl;
}

// Update all the weights of the particles in the particle filter
void ParticleFilter::updateWeights(vector<float> weights)
{
  cout << "Error updateWeights not implemented" << endl;
}

void ParticleFilter::updateVelocity(double time, double vx, double vy, double vYaw)
{
  cout << "Error updateVelocity not implemented" << endl;
}

void ParticleFilter::updatePose(double time, double x, double y, double yaw)
{
  cout << "Error updatePose not implemented" << endl;
}

void ParticleFilter::forceOrientation(double yaw)
{
  cout << "Error forceOrientation() not implemented" << endl;
}

void ParticleFilter::resample(double std[])
{
  cout << "Error resample not implemented" << endl;
}

void ParticleFilter::getBestParticle(Particle &p)
{
  cout << "Error getBestParticle not implemented" << endl;
}

int ParticleFilter::getBestParticleId()
{
  cout << "Error getBestParticleId not implemented" << endl;
  return 0;
}

void ParticleFilter::getBestAverageParticle(int n, Particle &p)
{
  cout << "Error getBestAverageParticle not implemented" << endl;
}

const vector<Particle> &ParticleFilter::getParticles()
{
  static vector<Particle> nullresult;
  cout << "Error getParticles() not implemented" << endl;
  return nullresult;
}

bool inWater(const Particle &p, const SatelliteManager &map)
{
  Vec3b pix;
  if(!map.getPixel(Point2d(p.x,p.y),pix))
  {
    cout << "Error particle out of the map!!" << endl;
    return false; // Its is out of the map (Why??)
  }

  if(pix[0] < pix[1] || pix[0] < pix[2])
  {
//    cout << "Particle out of water!" << endl;
    return false; // It is not in the water
  }

  // All good, particle is in the water
  return true;
}

bool inMap(const Particle &p, const SatelliteManager &map,
           double sonBearing, double sonRange)
{
  double radMid=p.theta,
         radBegin=radMid-(sonBearing/2.0),
         radEnd=radMid+(sonBearing/2.0);

  Point2d p1(p.x,p.y),
          p2(p1 + sonRange*Point2d(sin(radBegin),cos(radBegin))),
          p3(p1 + sonRange*Point2d(sin(radMid),cos(radMid))),
          p4(p1 + sonRange*Point2d(sin(radEnd),cos(radEnd)));

  Vec3b pix;

  return map.getPixel(p1,pix) &&
         map.getPixel(p2,pix) &&
         map.getPixel(p3,pix) &&
         map.getPixel(p4,pix);
}

// bool seeSomething(const Particle &p,
//                 const SatelliteManager &map,
//                 double sonBearing, double sonRange)
// {
//   Mat img = map.cropSonarFoV(Point2d(p.x,p.y),
//                          p.theta*180/M_PI,
//                          sonRange,
//                          sonBearing);
//   Mat bgr[3];
//   split(img,bgr);

//   int count = countNonZero(bgr[1]);

//   return count > 20;
// }

bool seeSomething(const Particle &p,
                const SatelliteManager &map,
                double sonBearing, double sonRange)
{
  // Mat img = map.cropSonarFoV(Point2d(p.x,p.y),
  //                        p.theta*180/M_PI,
  //                        sonRange,
  //                        sonBearing);
  // MODIFIQUE AQUI
  Mat img = map.cropSonarFoV(Point2d(p.x,p.y),
                         90.0 - p.theta*180/M_PI, 
                         sonRange,
                         sonBearing);



  // *** PROTEÇÃO contra imagem vazia ***
  if(img.empty() || img.cols == 0 || img.rows == 0)
    return false;

  Mat bgr[3];
  split(img, bgr);

  if(bgr[1].empty())
    return false;

  int count = countNonZero(bgr[1]);
  return count > 20;
}