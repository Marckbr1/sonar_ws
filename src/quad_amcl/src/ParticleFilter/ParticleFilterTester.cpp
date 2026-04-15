#include "ParticleFilterTester.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>


bool ParticleFilterTester::inMap(double x, double y)
{
  if(x >= minX && x <= maxX &&
     y >= minY && y <= maxY)
    return true;
  return false;
}

bool ParticleFilterTester::inMap(Particle &p)
{
  double x1=p.x,y1=p.y,
         radMid=p.theta,
         radBegin=radMid-sonarBearing/2.0,
         radEnd=radMid+sonarBearing/2.0,
         x2=x1+sin(radBegin)*sonaRange,
         y2=y1+cos(radBegin)*sonaRange,
         x3=x1+sin(radMid)*sonaRange,
         y3=y1+cos(radMid)*sonaRange,
         x4=x1+sin(radEnd)*sonaRange,
         y4=y1+cos(radEnd)*sonaRange;

  return inMap(x1,y1) &&
         inMap(x2,y2) &&
         inMap(x3,y3) &&
      inMap(x4,y4);
}

bool ParticleFilterTester::inWater(double x, double y)
{
  Vec3b pix;
  if(!map->getPixel(Point2d(x,y),pix))
    return false; // Its is out of the map (Why??)

  if(pix[0] != 255 && pix[1] != 0 && pix[2] != 0)
    return false; // It is not in the water

  // All good, particle is in the water
  return true;
}

bool ParticleFilterTester::inWater(Particle &p)
{
  return inWater(p.x,p.y);
}

ParticleFilterTester::ParticleFilterTester(bool testOrientation):
  testOrientation(testOrientation)
{
}

void ParticleFilterTester::createParticleLineFromPose(double x, double y, double theta,
                                                   vector<Particle> &ps)
{
  double ortoAng = theta + M_PI/2,
         lineOfParticlesLength=50; // 10m
  int nParticlesOnTheLine=10;
  double spaceBetweenParticles=lineOfParticlesLength/nParticlesOnTheLine;

  ps.resize(nParticlesOnTheLine);

  Point2d ortoU(sin(ortoAng), cos(ortoAng)),
      middlePoint(x,y),
      iniPoint = middlePoint - (lineOfParticlesLength/2.0)*ortoU,
      step = ortoU*spaceBetweenParticles;

  for(uint i=0; i < nParticlesOnTheLine; i++)
  {
    Point2d p= iniPoint + double(i) * step;
    ps[i].x =p.x;
    ps[i].y =p.y;
    ps[i].theta = theta;
    // Keep the weight the same from previous iteration
  }
}

void ParticleFilterTester::createRotParticlesOnPosiotion(double x, double y, double theta,
                                                         vector<Particle> &ps)
{
  int nParticles=10;
  double
      bearing = M_PI,
      iniTheta = theta - bearing/2,
      angInc = bearing/(nParticles+1);

  ps.resize(nParticles);
  for(uint i=0; i < nParticles; i++)
  {
    ps[i].x=x;
    ps[i].y=y;
    ps[i].theta=iniTheta + angInc*i;
    // We keep the weights as it is
  }
}

void ParticleFilterTester::setMap(const SatelliteManager *map)
{
  this->map = map;
}

// Initializes particle filter by initializing particles to
// Gaussian distribution around first position and all the weights set to 1.
bool ParticleFilterTester::init(double x, double y, double theta, double std[], double time,
                          double minX, double minY, double maxX, double maxY)
{
  if(x < minX || x > maxX || y < minY || y > maxY)
  {
    cerr << "Initial pose out of the map" << endl;
    return false;
  }

  this->minX = minX;
  this->minY = minY;
  this->maxX = maxX;
  this->maxY = maxY;
  currentTime = time;


  // Save initial pose
  iniX = x; iniY = y; iniTheta = theta;

  // NOTE: The number of particles needs to be tuned
  num_particles = 20;

  if(testOrientation)
    createRotParticlesOnPosiotion(x,y,theta,particles);
  else
    createParticleLineFromPose(x,y,theta,particles);

  // Since this function is called only once(first measurement), set to True
  is_initialized = true;

  return true;
}

bool ParticleFilterTester::resampleOutOfBoundParticles(double std[])
{
  bool didSomething=false;
  for(uint i = 0 ; i < particles.size(); i++)
  {
    if(!inMap(particles[i]))
    {
//      newParticleFromBestEstimation(particles[i],std);
      didSomething=true;
    }
  }
  return didSomething;
}

bool ParticleFilterTester::resampleOutOfWaterParticles(double std[])
{
  bool didSomething=false;
  for(uint i = 0 ; i < particles.size(); i++)
  {
    if(!inMap(particles[i]))
    {
//      newParticleFromBestEstimation(particles[i],std);
      didSomething=true;
    }
  }
  return didSomething;
}

// Predicts the state(set of particles) for the next time step
// using the process model.
void ParticleFilterTester::prediction(double time,
                                double std_pos[])
{
  double dt = time - currentTime;
  if(dt > 3.0 || dt <= 0.0)
  { // Invalid time
    cout << "Invalid time!" << endl;
    return;
  }
  currentTime = time;

  double dx = vx*dt,
         dy = vy*dt,
         dYaw = vYaw*dt;

  // Prediction for position x,y and angle theta for each of the particles
  for(size_t par_index = 0; par_index < particles.size(); par_index++)
  {
    Particle &p = particles[par_index];

    // Update the position x, y and angle theta of the particle
    // considering body local velocity and global absolute position
    double co = cos(p.theta), so=sin(p.theta);
    p.x +=  dx*so +dy*co;
    p.y +=  dx*co -dy*so;
    p.theta += dYaw;
  }
}

void ParticleFilterTester::getMinMaxWieght(double &minW, double &maxW)
{
  if(particles.size() == 0)
  {
    minW = maxW = 0.0;
    return;
  }

  minW = maxW = particles[0].weight;

  for(uint i = 0; i < particles.size(); i++)
  {
    const Particle &p = particles[i];

    if(minW > p.weight)
      minW = p.weight;
    if(maxW < p.weight)
      maxW = p.weight;
  }
}

// Update all the weights of the particles in the particle filter
void ParticleFilterTester::updateWeights(vector<float> weights)
{
  if(weights.size() == 0) return;

  // quad network give us distance, so we have to
  // normalize and invert the weight
  float minW,maxW;
  minW = maxW = weights[0];

  for(uint i = 1 ; i < weights.size(); i++)
  {
    if(minW > weights[i]) minW = weights[i];
    if(maxW < weights[i]) maxW = weights[i];
  }

  cout << endl << "Weight update:" << endl;
  for(uint i = 0 ; i < weights.size(); i++)
  {
//    particles[i].weight = weights[i];
//    cout << weights[i] << " -> ";
    particles[i].weight = maxW - weights[i] + minW;
//    cout << particles[i].weight << endl;
  }
  normalizeWeights();
}

void ParticleFilterTester::updateVelocity(double time, double vx, double vy, double vYaw)
{
  this->vx = vx;
  this->vy = vy;
  this->vYaw = vYaw;
  currentTime = time;
}

void ParticleFilterTester::updatePose(double time, double x, double y, double yaw)
{
  if(testOrientation)
    createRotParticlesOnPosiotion(x,y,yaw,particles);
  else
    createParticleLineFromPose(x,y,yaw,particles);
}

// Resample particles with replacement with probability proportional to weight.
void ParticleFilterTester::resample(double std[])
{
}

void ParticleFilterTester::normalizeWeights()
{
  if(particles.size() == 0) return;

  double minW,maxW,wRange;
  getMinMaxWieght(minW,maxW);
  wRange = maxW-minW;

  if(wRange<1e-4)
  {
    cout << "Weight normalization aborted. All weights are symetrical." << endl;
    return ;
  }

  for(uint i = 0; i < particles.size(); i++)
  {
    Particle &p= particles[i];
    p.weight= (p.weight-minW)/wRange;
  }
}

void ParticleFilterTester::getBestParticle(Particle &p)
{
  int bestPId = getBestParticleId();
  p = particles[bestPId];
}

int ParticleFilterTester::getBestParticleId()
{
  if(particles.size()==0) return -1;

  double highest_weight = particles[0].weight;
  int bestPId=0;

  for (uint i = 1; i < num_particles; ++i)
  {
    if (particles[i].weight > highest_weight)
    {
      highest_weight = particles[i].weight;
      bestPId = i;
    }
  }
  return bestPId;
}

const vector<Particle> &ParticleFilterTester::getParticles()
{
  return particles;
}
