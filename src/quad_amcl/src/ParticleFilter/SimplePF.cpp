#include "SimplePF.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

SimplePF::SimplePF():
  genMt19937(std::random_device{}()),
  gen(std::random_device{}())
{
}

void SimplePF::setMap(const SatelliteManager *map)
{
  this->map = map;
}

// Initializes particle filter by initializing particles to
// Gaussian distribution around first position and all the weights set to 1.


bool SimplePF::init(double x, double y, double theta, double std[], double time,
                          double minX, double minY, double maxX, double maxY)
{
  // En tu código antes de llamar a init(), verifica los límites
  ROS_INFO("Map bounds: minX=%.2f, minY=%.2f, maxX=%.2f, maxY=%.2f", 
         minX, minY, maxX, maxY);
  ROS_INFO("Initial pose: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);

  
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
  num_particles = 120;
  particles.resize(num_particles);

  // Add random Gaussian noise to each particle.
  for (uint par_index = 0; par_index < num_particles; ++par_index)
  {
    Particle &new_particle = particles[par_index];
    new_particle.weight = 1.0/num_particles;
    if(!newParticleFromPose(new_particle,x,y,theta,std))
      newParticleFromRandomMap(new_particle);
    new_particle.id = par_index;

    ROS_INFO("Particle %u initialized!", par_index);
    // Set the id of the particle to be the same as the current index
  }

  // Since this function is called only once(first measurement), set to True
  is_initialized = true;

  return true;
}

bool SimplePF::isParticleOk(Particle &p)
{
  bool good=false;
  good = inMap(p,*map,sonarBearing,sonaRange);

  if(good && particlesOnWaterOnly)
    good = inWater(p,*map);

  if(good && particleThaSeeSomethingOnly)
    good = seeSomething(p,*map,sonarBearing,sonaRange);

  if(p.weight <= 0.0)
    cout << "Particle weight problem!" << endl;

  return good;
}

bool SimplePF::resampleNotOkParticles(double std[])
{
  double wide_std[3]={15.0,15.0,std[2]};

  bool didSomething=false;
  for(uint i = 0 ; i < particles.size(); i++)
  {
    if(!isParticleOk(particles[i]))
    {
      if(!newParticleFromBestEstimation(particles[i],std))
        newParticleFromPose(particles[i],
                            particles[i].x,
                            particles[i].y,
                            particles[i].theta,
                            wide_std);

//        newParticleFromRandomMap(particles[i]);
      didSomething=true;
    }
  }
  return didSomething;
}


// Predicts the state(set of particles) for the next time step
// using the process model.
void SimplePF::prediction(double time,
                          double std_pos[])
{
  double dt = time - currentTime;
  if(dt == 0.0) return;
  if(dt > 3.0)
  { // Invalid time
    cout << "Invalid time!" << endl;
    return;
  }

  currentTime = time;

  // Create a normal (Gaussian) distribution for noise along position x.
  normal_distribution<double> noise_dist_x(0, std_pos[0]);
  // Create a normal (Gaussian) distribution for noise along position y.
  normal_distribution<double> noise_dist_y(0, std_pos[1]);
  // Create a normal (Gaussian) distribution for noise of direction theta.
  normal_distribution<double> noise_dist_theta(0, std_pos[2]);

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
    p.x +=  dx*so -dy*co;
    p.y +=  dx*co +dy*so;
    p.theta += dYaw;

    // Add random gaussian noise for each of the above updated measurements
    p.x += noise_dist_x(gen);
    p.y += noise_dist_y(gen);
    p.theta += noise_dist_theta(gen);

//    ROS_INFO("p %lu x %g y %g y %g w %g",
//             par_index,p.x,p.y,p.theta,p.weight);
  }

  // Do not resample particles here because
  // they may be under evaluation of the quad network
//  resampleNotOkParticles(std_pos);
}

void SimplePF::getMinMaxWieght(double &minW, double &maxW)
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
void SimplePF::updateWeights(vector<float> weights)
{
  if(weights.size() == 0) return;
  double wide_std[3]={15.0,15.0,0.001};

  // quad network give us distance, so we have to
  // normalize and invert the weight
  float minW,maxW;
  minW = maxW = weights[0];

  double accW =0.0;
  for(uint i = 0 ; i < weights.size(); i++)
  {
    if(weights[i] > 1.1)
      newParticleFromPose(particles[i],
                          particles[i].x,
                          particles[i].y,
                          particles[i].theta,
                          wide_std);

    if(minW > weights[i]) minW = weights[i];
    if(maxW < weights[i]) maxW = weights[i];
    accW+=weights[i];
  }

//  cout << endl << "Weight update:" << endl;
  for(uint i = 0 ; i < weights.size(); i++)
  {
//    particles[i].weight = weights[i];
//    cout << weights[i] << " -> ";
    particles[i].weight = (maxW - weights[i]) + minW;
//    cout << particles[i].weight << " -> ";
    particles[i].weight /= accW;
//    cout << particles[i].weight << endl;

    // Invert weights, normalize and multiply by last weight
//    if(particles[i].weight > 0.0)
//      particles[i].weight *= (maxW - weights[i] + minW)/accW;
//    else
//      particles[i].weight = (maxW - weights[i] + minW)/accW;
  }
  // Normalize weights after mutiplication
  normalizeWeights();
}

void SimplePF::updateVelocity(double time, double vx, double vy, double vYaw)
{
  this->vx = vx;
  this->vy = vy;
  this->vYaw = vYaw;
  currentTime = time;
}

void SimplePF::forceOrientation(double yaw)
{
  for(uint i = 0; i < particles.size();i++)
    particles[i].theta = yaw;
  this->vYaw = 0.0;
}

// Resample particles with replacement with probability proportional to weight.
void SimplePF::resample(double std[])
{// Roulette wheel resampling
  // Copy of the particles vector list
  vector<Particle> particlesCopy = particles;

  // Clear existing particle list
  particles.clear();

  // Vector of weights of the particles
  vector<double> weights;
  for(size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
  {
    weights.push_back(particlesCopy[par_index].weight*100.0);
  }

  // Object for generating discrete distribution based on the weights vector
  discrete_distribution<int> weights_dist(weights.begin(), weights.end());

  // Create a normal (Gaussian) distribution for noise along position x.
  normal_distribution<double> noise_dist_x(0, std[0]);
  // Create a normal (Gaussian) distribution for noise along position y.
  normal_distribution<double> noise_dist_y(0, std[1]);
  // Create a normal (Gaussian) distribution for noise of direction theta.
  normal_distribution<double> noise_dist_theta(0, std[2]);

  // With the discrete distribution pick out particles according to their
  // weights. The higher the weight of the particle, the higher are the chances
  // of the particle being included multiple times.
  // Discrete_distribution is used here to pick particles with the appropriate
  // weights(i.e. which meet a threshold)
  // http://www.cplusplus.com/reference/random/discrete_distribution/
  // NOTE: Here is an example which helps with the understanding
  //       http://coliru.stacked-crooked.com/a/3c9005a4cc0ed9d6
  for(size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
  {
    // Append the particle to the new list
    // NOTE: Calling weights_dist with the generator returns the index of one
    //       of weights in the vector which was used to generate the distribution.
    particles.push_back(particlesCopy[weights_dist(genMt19937)]);
    // Object of random number engine class that generate pseudo-random numbers
    // NOTE: http://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine

    Particle &p = particles.back();
    double nx = noise_dist_x(gen),
           ny = noise_dist_y(gen);

//    cout << "P" << par_index
//         << " nx " << nx
//         << " ny " << ny << endl;

    // Add random gaussian noise for each of the above updated measurements
    p.x += nx;
    p.y += ny;
    p.theta += noise_dist_theta(gen);
  }

//  Particle randomP;
//  newParticleFromRandomMap(randomP);
//  particles.push_back(randomP);

  resampleNotOkParticles(std);

  normalizeWeights();
}

void SimplePF::resampleV2(double std[])
{
  sort(particles.begin(),particles.end(),descendingParticles);
  size_t n = particles.size();

  if(resampleKWorst > n) resampleKWorst =n;

  for(size_t i = n-resampleKWorst; i < n; i++)
  {
    particles[i] = getNewParticle(std);
  }
}

void SimplePF::normalizeWeights()
{
  if(particles.size() == 0) return;

  double accW=0.0;
  for(uint i = 0; i < particles.size(); i++)
    accW+=particles[i].weight;
  if(accW>0.0)
  for(uint i = 0; i < particles.size(); i++)
    particles[i].weight/=accW;
}

Particle SimplePF::getNewParticle(double std[])
{
  Particle newP;

  int destine = rand()%101;
  if(destine > 5)
  {
    newParticleFromBestEstimation(newP,std);
  }else
  { // Create a random particle in the map (Not used)
    newParticleFromRandomMap(newP);
  }
  return newP;
}

void SimplePF::newParticleFromBegin(Particle &p, double std[])
{
  newParticleFromPose(p,
                      iniX,iniY,iniTheta,
                      std);
}

void SimplePF::newParticleFromRandomMap(Particle &p)
{
  int xRange = (maxX-minX)*1e2,
      yRange = (maxY-minY)*1e2,
      thetaRange= (2*M_PI)*1e3,
      attemps=0;

  if(particles.size()>0)
    p.weight = 1.0/particles.size();
  else p.weight = 0.5;

  for(attemps = 0; attemps < 10; attemps++)
  {
    p.x = (rand()%xRange)*1e-2 +minX;
    p.y = (rand()%yRange)*1e-2 +minY;
    p.theta = (rand()%thetaRange)*1e-3 - M_PI;

    if(!isParticleOk(p))
      continue;

    // All good, the new particle was created
    break;
  }

  if(attemps>=10)
    ROS_WARN("Could not find a good pose on the map!");
}

bool SimplePF::newParticleFromBestEstimation(Particle &p, double std[])
{
  int bestPId = getGoodParticleIdToClone();
  bool doesItFound=false;

  doesItFound= newParticleFromPose(p,
                  particles[bestPId].x,
                  particles[bestPId].y,
                  particles[bestPId].theta,
                  std);

  // Copy the weight from the copied particle
  p.weight = particles[bestPId].weight;

  return doesItFound;
}

bool SimplePF::newParticleFromPose(Particle &p, double x, double y, double theta, double std[])
{
  normal_distribution<double> dist_x(0, std[0]);
  normal_distribution<double> dist_y(0, std[1]);
  normal_distribution<double> dist_theta(0, std[2]);

  int attempts=0;

  for(attempts=0; attempts < 30; attempts++)
  {
    p.x = x + dist_x(gen);
    p.y = y + dist_y(gen);
    p.theta = theta + dist_theta(gen);
    if(!isParticleOk(p))
      continue;
    break;
  }

  if(attempts >= 30)
  {
    ROS_WARN("Warning- Could not find a good pose in the map");
    return false;
  }
  return true;
}

void SimplePF::getBestParticle(Particle &p)
{
  int bestPId = getBestParticleId();
  p = particles[bestPId];
}

int SimplePF::getBestParticleId()
{
  double highest_weight = 0.0;
  int bestPId=-1;
  for (uint i = 0; i < num_particles; ++i)
  {
    if (particles[i].weight > highest_weight)
    {
      highest_weight = particles[i].weight;
      bestPId = i;
    }
  }
  return bestPId;
}

// Get a particle considering its weight as probability to be choose
int SimplePF::getGoodParticleIdToClone()
{
  // Vector of weights of the particles
  vector<double> weights;
  weights.resize(particles.size());
  for(size_t par_index = 0;
      par_index < particles.size();
      par_index++)
  {
    weights[particles[par_index].weight*100.0];
  }

  // Object for generating discrete distribution based on the weights vector
  discrete_distribution<int> weights_dist(weights.begin(), weights.end());

  int particleId=-1,attempt;

  for(attempt=0; attempt < 10; attempt++)
  {
    particleId=weights_dist(genMt19937);
    if(isParticleOk(particles[particleId]))
      break;
  }

  if(attempt==10)
    cout << "Warning could not find a good particle!" << endl;

  return weights_dist(genMt19937);
}

void SimplePF::getBestAverageParticle(int n, Particle &p)
{
  p.x=p.y=p.theta=p.weight=0.0;
  if(n==0 || particles.size()==0) return;
  if(n > particles.size()) n = particles.size();

  vector<Particle> ps = particles;
  sort(ps.begin(),ps.end(),descendingParticles);

  double accX=0.0, accY=0.0, accYaw=0.0, accW=0.0;

  for(uint i=0; i < n; i++)
  {
    Particle &p = particles[i];
    accX += p.x*p.weight;
    accY += p.y*p.weight;
    accYaw += p.theta*p.weight;
    accW += p.weight;
  }
  p.x = accX/accW;
  p.y = accY/accW;
  p.theta = accYaw/accW;
}

const vector<Particle> &SimplePF::getParticles()
{
  return particles;
}

// Convert the passed in vehicle co-ordinates into map co-ordinates from
// the perspective of the particle in question
LandmarkObs SimplePF::convertVehicleToMapCoords(LandmarkObs observationToConvert, Particle particle)
{
  // NOTE: The observations are given in the VEHICLE'S coordinate system.
  // 	     Your particles are located according to the MAP'S coordinate system.
  //       A transformation is required between the two systems.
  //   		 The following is a good resource for the theory:
  //   		 https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //  		 and the following is a good resource for the actual equation to
  //       implement (look at equation 3.33. The equation stays as it is.
  //       1. http://planning.cs.uiuc.edu/node99.html
  //       2. http://www.sunshine2k.de/articles/RotationDerivation.pdf
  LandmarkObs convertedObservation;
  convertedObservation.id = observationToConvert.id;
  convertedObservation.x = particle.x + \
                           observationToConvert.x * cos(particle.theta) - \
                           observationToConvert.y * sin(particle.theta);

  convertedObservation.y = particle.y + \
                           observationToConvert.x * sin(particle.theta) + \
                           observationToConvert.y * cos(particle.theta);

  return convertedObservation;
}

// Find the closest landmark to the current observation
vector<LandmarkObs> SimplePF::dataAssociation(/*vector<Map::single_landmark_s> landmarks,*/ vector<LandmarkObs> observations)
{
  // Vector of associated landmarks
  vector<LandmarkObs> associatedLandmarks;

  // Go through list of observations
//  for(size_t obs_index = 0; obs_index < observations.size(); obs_index++)
//  {
//      // Start of with the maximum possible value
//      double minDistance = DBL_MAX;
//      size_t indexOfLandmark;

//      // Find the landmark closest to the observation
//      for(size_t land_index = 0; land_index < landmarks.size(); land_index++)
//      {
//          double currentDistance = dist(landmarks[land_index].x_f,
//                                        landmarks[land_index].y_f,
//                                        observations[obs_index].x,
//                                        observations[obs_index].y);

//          // Update the minimum distance found and the index if
//          // another landmark is closer to this observation
//          if(currentDistance <= minDistance)
//          {
//              minDistance = currentDistance;
//              indexOfLandmark = land_index;
//          }
//      }

//      LandmarkObs closestLandmark;
//      closestLandmark.id = landmarks[indexOfLandmark].id_i;
//      closestLandmark.x = landmarks[indexOfLandmark].x_f;
//      closestLandmark.y = landmarks[indexOfLandmark].y_f;
//      associatedLandmarks.push_back(closestLandmark);
//  }

  // Return the associated landmarks
  return associatedLandmarks;
}
