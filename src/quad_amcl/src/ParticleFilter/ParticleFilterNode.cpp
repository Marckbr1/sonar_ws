#include "ParticleFilterNode.h"
//#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <random>

#include "ParticleFilterTester.h"
#include "SimplePF.h"

void ParticleFilterNode::publishPose(ros::Publisher &pub,
                                     const Particle &p,
                                     const ros::Time &time)
{
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = time;
  msg.pose.position.x = p.x;
  msg.pose.position.y = p.y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion quat_tf;
  quat_tf.setEuler(p.theta,0.0,0.0);
  tf2::convert(quat_tf,msg.pose.orientation);

  pub.publish(msg);
}

void ParticleFilterNode::drawParticle(Mat &img,
                                      const Particle &p,
                                      const string &txt,
                                      const Scalar &color,
                                      int thickness)
{
  // Get the sonar field of view polygon on satellite image
  int nPts = 15;
  Point pts[nPts];

  sat.getSonarPolyOnImg(Point2d(p.x,p.y),p.theta*180.0/M_PI,
                        pts,nPts,sonarRange,sonarFoV);

  int npts[] = {nPts};
  const Point* ppt[1] = { pts };

  polylines(img,
            ppt,npts,
            1,true,
            color,thickness);
  if(!txt.empty())
    putText(img,txt,
            pts[0],FONT_HERSHEY_DUPLEX,
            3.0,color,9);
}

bool ParticleFilterNode::setup()
{
  // Parameters
  string map_path;

  if(!pnh.getParam("map_path",map_path))
  {
    ROS_ERROR("Parameter ~map_path not defined!");
    return false;
  }
  mapPath = map_path;

  pnh.param("test",doTest,false);
  if(doTest)
  {
    bool orientationTest;
    pnh.param("orientation_test",orientationTest,false);
    pf.reset(new ParticleFilterTester(orientationTest));
    ROS_INFO("=== FILTRO SELECCIONADO: ParticleFilterTester ===");
  }
  else
  {
    pf.reset(new SimplePF);
    ROS_INFO("=== FILTRO SELECCIONADO: SimplePF ===");  // ← ESTE DEBE APARECER
  }

  pnh.param("disable_orientation",disableOrientation,false);

  if(!sat.loadMap(mapPath.string()))
  {
    ROS_ERROR_STREAM("Could not load map " << map_path);
    return false;
  }
  sat.normalizeMapAndSave();

  if(removeSatteliteOffset)
    sat.removeOffset();

  pf->setMap(&sat);

  // Topic Subscription
  subSonImg = it.subscribe("/son",1,
        &ParticleFilterNode::sonCallback,this);

  subPose = nh.subscribe("/pose",1,
        &ParticleFilterNode::poseCallback,this);

  subRank = nh.subscribe("/rank",1,
        &ParticleFilterNode::rankCallback,this);

  // Topic advertisement
  pubSatImgs = it.advertise("/sat_crop",300); // Particles view
  pubSonImg = it.advertise("/son_small",3); // Sonar image
  pubNumParticles = nh.advertise<quad_amcl::StampedInteger>("/n_particles",1);

  pubPose = nh.advertise<geometry_msgs::PoseStamped>("/pf_pose",1);
  pubAvgPose5N = nh.advertise<geometry_msgs::PoseStamped>("/pf_avg5_pose",1);
  pubAvgPose10N = nh.advertise<geometry_msgs::PoseStamped>("/pf_avg10_pose",1);
  pubAvgPose15N = nh.advertise<geometry_msgs::PoseStamped>("/pf_avg15_pose",1);

  pubOdom = nh.advertise<geometry_msgs::PoseStamped>("/odom_pose",1);

  pubMapImg = it.advertise("/map_img",1);
  pubParticlesView = it.advertise("/particles_view",1);

  return true;
}

bool ParticleFilterNode::initPF()
{
  if(!pf->initialized())
  {
    if(!hasFirstPose)
      return false;

    if(pfInitMode==PF_GUESS_INI)
      iniPFWithGuess(lastX,lastY,lastYaw);
    else if(pfInitMode==PF_RANDOM_INI)
      iniPFRandom();
  }
  return true;
}

void ParticleFilterNode::iniPFRandom()
{
  cout << "Initializing PF with random poses." << endl;

  Point2d min, max;
  sat.minMaxUTMValues(min,max);

  // Decrease map size considering the sonar range
//  min.x += sonarRange;
//  min.y += sonarRange;
//  max.x -= sonarRange;
//  max.y -= sonarRange;

  double n_x, n_y, n_theta;

  n_x = (min.x+max.x)/2.0;
  n_y = (min.y+max.y)/2.0;
  n_theta = 0.0;

  double sigma_pos_init [3] = {5*sqrt(max.x-min.x),
                               5*sqrt(max.y-min.y),
                               0.11};

  // Add noise to the ground truth for the initialization step
  if(!pf->init(n_x, n_y, n_theta, sigma_pos_init,
               lastTime,
               min.x,min.y,max.x,max.y))
  {
    ROS_ERROR("Could not init particle filter!!");
  }
}

void ParticleFilterNode::iniPFWithGuess(double x, double y, double theta)
{


  // ROS_INFO("=========================================");
  // ROS_INFO("EJECUTANDO initPFWithGuess");
  // ROS_INFO("=========================================");

  // ROS_INFO("Pose inicial recibida:");
  // ROS_INFO("  x = %.4f", x);
  // ROS_INFO("  y = %.4f", y);
  // ROS_INFO("  theta = %.4f rad (%.2f grados)", theta, theta * 180.0 / M_PI);

  // ROS_INFO("Limites del mapa:");
  // ROS_INFO("  minX = %.4f, minY = %.4f", minMapP.x, minMapP.y);
  // ROS_INFO("  maxX = %.4f, maxY = %.4f", maxMapP.x, maxMapP.y);


  Point2d minMapP,maxMapP;
  sat.minMaxUTMValues(minMapP,maxMapP); // Este es el error

  // Decrease map size considering the sonar range
  // minMapP.x += sonarRange;
  // minMapP.y += sonarRange;
  // maxMapP.x -= sonarRange;
  // maxMapP.y -= sonarRange;

  double sigma_pos_initialization[] = { 30.0,30.0,0.01};



  pf->init(x,y,theta,sigma_pos_initialization,
          lastTime,
          minMapP.x,minMapP.y,
          maxMapP.x,maxMapP.y);
}

void ParticleFilterNode::sonCallback(const sensor_msgs::ImageConstPtr &msg)
{
  ros::Time time_begin = ros::Time::now();

  if(!pf->initialized())
  {
    initPF();
    return ; // PF was not running
  }

  // Check comunication status
  checkSonEvalTimming();

  // Check timestamp
  double time = msg->header.stamp.toSec();

  // Only process image after specified time
  if(time - lastSonEvalTime < sonImgEvalPeriod)
    return ;

  // If the neural network is ready to a new evaluation
  if(pfComunicationState != READY)
    return; // Not ready to evaluate this acoustic image

  try
  {
    lastSonSmall = cv_bridge::toCvShare(msg, "mono8")->image;
//    son = cv_bridge::toCvShare(msg, "bgr8")->image;
//    cv::imshow("View son", lastSonSmall);
//    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }

  // Resize son to 256 columns 128 rows
  resize(lastSonSmall,lastSonSmall,Size(256,128));

  // Only process if the image has enought features
  if(countNonZero(lastSonSmall) < 750)
    return ; // Not enought features to match, skip the image

  cvtColor(lastSonSmall,lastSonSmall,CV_GRAY2BGR);

  // Update last processed image timestamp
  lastSonEvalTime=time;

  evaluateParticlesObservation(msg->header.stamp); // Publish msg using sonar timestamp

  ros::Time time_end = ros::Time::now();
  ros::Duration duration = time_end - time_begin;
  ROS_INFO("Soncalback %lf secs", duration.toSec());
}

void ParticleFilterNode::poseCallback(const geometry_msgs::PoseStamped &msg)
{
  ros::Time time_begin = ros::Time::now();

  // We have to transform pose in velocity
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.orientation,q);

  tf::Matrix3x3 mat(q);
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw,pitch,roll,0);

//  cout << "Orientation solution 1: " << endl
//       << "roll: " << roll*180.0/M_PI << endl
//       << "pitch: " << pitch*180.0/M_PI << endl
//       << "yaw: " << yaw*180.0/M_PI << endl;

  if(abs(roll) > 1.0)
    mat.getEulerYPR(yaw,pitch,roll,1);

//  cout << "Orientation solution 2: " << endl
//       << "roll: " << roll*180.0/M_PI << endl
//       << "pitch: " << pitch*180.0/M_PI << endl
//       << "yaw: " << yaw*180.0/M_PI << endl;

//  mat.getRPY(roll,pitch,yaw);
//  cout << "Orientation 2way " << yaw*180.0/M_PI << endl;

  double time = msg.header.stamp.toSec(),
      dt = time-lastTime,
      px = msg.pose.position.x,
      py = msg.pose.position.y,
      dx = px-lastX,
      dy = py-lastY,
      dYaw = yaw-lastYaw;
  // Getn smaller ang diff
  if(dYaw > M_PI)
    dYaw-= 2*M_PI;
  else if(dYaw < -M_PI)
    dYaw+= 2*M_PI;

  lastX = px;
  lastY = py;
  lastYaw = yaw;
  lastTime = time;
  hasFirstPose = true;

  if(!pf->initialized())
  {
    initPF();
    return ; // PF is not running
  }

  if(!odom.initialized())
  {
    odom.setInitialPose(time,
                        px,py,
                        yaw);
  }

//  ROS_INFO("New pose received! Yaw = %g p %g r %g", yaw, pitch, roll );

//  ROS_INFO("dx %g dy %g dYaw %g dt %g",dx,dy,dYaw*180.0/M_PI,dt);

  // Update current velocity
  if(dt > 0.0 && dt < 10.0)
  {
//    ROS_INFO("Prediction!");
    vx = dx/dt;
    vy = dy/dt;
    vYaw = dYaw/dt;

    // This is the global velocity
    // now we are converting to local velocity
    double lvx = vx * sin(yaw) + vy*cos(yaw),
           lvy =-vx * cos(yaw) + vy*sin(yaw);

    // Vehicle max speed is 0.65 m/s (Truncated vel.)
    // Odom 4 vel
//    if(lvx > 0.1 && lvx <= 0.3) lvx = 0.2;
//    else if(lvx > 0.3 && lvx <= 0.5) lvx = 0.4;
//    else if(lvx > 0.5) lvx = 0.6;
//    else lvx = 0.0;

    // Odom 3 Vel
    if(lvx > 0.1 && lvx <= 0.4) lvx = 0.3;
    else if(lvx > 0.4) lvx = 0.6;
    else lvx = 0.0;

//    lvx = round(lvx*1e1)*1e-1;
    lvy = 0.0;

    // Update vx and vy with local velocity
    vx = lvx;
    vy = lvy;

    if(!doTest)
    {
      // Predict particles to this instant
      pf->prediction(time,sigma_pos);

      // Update particles velocity
      pf->updateVelocity(time,vx,vy,vYaw);
      if(disableOrientation)
        pf->forceOrientation(yaw);
    }else
    {
      pf->updatePose(time,px,py,yaw);
    }

    // Update odometry
    odom.updateVelocity(time,vx,vy,vYaw);
    if(disableOrientation)
      odom.forceOrientation(yaw);

    // Update map view
    if(time - lastMapPubTime >= mapPubPeriod)
    {
      updateMapView(msg.header.stamp);
      lastMapPubTime=time;
    }

    // Publish best pose estimation from particle filter
    Particle p;
    pf->getBestParticle(p);
    publishPose(pubPose,p,msg.header.stamp);

    pf->getBestAverageParticle(5,p);
    publishPose(pubAvgPose5N,p,msg.header.stamp);

    pf->getBestAverageParticle(10,p);
    publishPose(pubAvgPose10N,p,msg.header.stamp);

    pf->getBestAverageParticle(120,p);
    publishPose(pubAvgPose15N,p,msg.header.stamp);

    // Publish pose from odometry
    if(odom.predict(time,
                    p.x,p.y,
                    p.theta))
    {
      publishPose(pubOdom,p,msg.header.stamp);
    }

  }else
  {
    cout << "Invalid timestamp " << dt
         << endl
         << "t0: " << lastTime
         << " t1: " << time << endl;

    vx = 0.0;
    vy = 0.0;
    vYaw = 0.0;
  }

  ros::Time time_end = ros::Time::now();
  ros::Duration duration = time_end - time_begin;
//  ROS_INFO("PoseCalback %lf secs", duration.toSec());
}

void ParticleFilterNode::rankCallback(const quad_amcl::StampedArrayFloat &msg)
{
  ros::Time time_begin = ros::Time::now();

  pf->updateWeights(msg.data);
  pf->resample(sigma_pos);

  pfComunicationState = READY;
  updateParticlesView(msg.header.stamp);

  ros::Time time_end = ros::Time::now();
  ros::Duration duration = time_end - time_begin;
  ROS_INFO("RankCalback %lf secs", duration.toSec());

  //  ROS_INFO("PF Ready");
}

void ParticleFilterNode::checkSonEvalTimming()
{
  static int waitingIterationCount=0;

  // If it is waiting for the particles evaluation (communication)
  if(pfComunicationState == WAITING_EVALUATION)
  {
    waitingIterationCount++;
    // If it is waiting for too long
    if(waitingIterationCount>40)
    {
      // Stop to wait...
      pfComunicationState = READY;
      waitingIterationCount=0;
    }
  }
  else
    waitingIterationCount=0;
}

void ParticleFilterNode::updateMapView(const ros::Time &time)
{
  Mat screen = sat.getMapImg();
  string txt;
  const vector<Particle> &ps = pf->getParticles();

  // Particles Color version
//  double minW, maxW;
//  pf->getMinMaxWieght(minW,maxW);

  // Draw particles
  for(uint i = 0; i < ps.size();i++)
  {
    const Particle &p = ps[i];

    drawParticle(screen,
                 p,txt,
                 Scalar(255,0,255),2);

    // Particles Color version
//    double normW=1.0;
//    if(maxW != minW)
//      normW=(p.weight-minW)/(maxW-minW);
//    const Vec3b& cPix = colors.at<Vec3b>(0,normW*255);

//    drawParticle(screen,
//                 p,txt,
//                 Scalar(cPix[0],cPix[1],cPix[2]),2);
  }

  // Draw Odom
  {
    Particle p;
    odom.predict(time.toSec(),
                 p.x,p.y,p.theta);
    txt="ODOM";

    drawParticle(screen,
                 p,txt,
                 Scalar(128,255,128),6);
  }

  // Draw GT
  {
    Particle p;
    p.x = lastX; p.y = lastY;
    p.theta = lastYaw;
    txt="GT";

    drawParticle(screen,
                 p,txt,
                 Scalar(0,255,255),12);
  }

  // Draw best particle
  {
    Particle p;
//    pf->getBestParticle(p);
    pf->getBestAverageParticle(120,p);

    drawParticle(screen,
             p,format("PF_Best"),
             Scalar(0,0,0),12);
  }

  // Resize img
  double s = 900.0/screen.cols;
  resize(screen,screen,Size(),s,s);

  // Show img on screen
  if(showImgMap)
  {
    imshow("Test", screen);
    waitKey(10);
  }

  // Publish msg
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();
  msg->header.stamp = time;
  pubMapImg.publish(msg);

}

void ParticleFilterNode::updateParticlesView(const ros::Time &time)
{
  Mat screen(600,800,CV_8UC3, Scalar(0,0,0));

  vector<Particle> ps = pf->getParticles();
  sort(ps.begin(),ps.end(),descendingParticles);

  uint t = ps.size(),n,m;
  {
    double r = double(screen.cols)/double(screen.rows),
         ri = 256.0/128.0,
        rf= r/ri,
        dm=sqrt(t/rf),// +1, // Plus one line to fit son img
        dn = rf*dm;
        m = int(ceil(dm));
        n = int(ceil(dn));
  }

  uint cellW=screen.cols/n,
       cellH=screen.rows/m,
       k=0;

  Mat img;

  for(uint i = 0; i < m && k < t; i++)
  {
    for(uint j = 0; j < n && k < t; j++)
    {
      const Particle &p = ps[k++];
      uint x=j*cellW,y=i*cellH;

      img = sat.cropSonarFoV(Point2d(p.x,p.y),
                             p.theta*180/M_PI,
                             sonarRange,
                             130.0);

      if(img.empty())
      {
        ROS_ERROR("Could not crop sat img");
        continue;
      }

      resize(img,img,Size(cellW,cellH));
      putText(img,format("%.1f",p.weight*100.0),
              Point(img.cols/2-15*3,img.rows-15),
              FONT_HERSHEY_DUPLEX,
              1.0,Scalar(255,0,255),1);

      img.copyTo(screen(Rect(x,y,cellW,cellH)));
    }
  }

  if(showParticlesView)
  {
    imshow("Particles", screen);
    waitKey(10);
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();

  msg->header.stamp = time;
  pubParticlesView.publish(msg);
}

ParticleFilterNode::ParticleFilterNode():
    pnh("~"),
    it(nh),
    r(4.0),
    colors(1,256,CV_8UC1)
{
  for(uint i =0 ;i < 256; i++)
    colors.at<uchar>(0,i) = i;

  applyColorMap(colors,colors,COLORMAP_JET);
}


int ParticleFilterNode::run()
{
//  // NOTE: These parameters are related to grading.
//  // Number of time steps before accuracy is checked by grader.
//  int time_steps_before_lock_required = 100;
//  // Max allowable runtime to pass [sec]
//  double max_runtime = 45;
//  // Max allowable translation error to pass [m]
//  double max_translation_error = 1;
//  // Max allowable yaw error [rad]
//  double max_yaw_error = 0.05;

//  // Start timer.
//  int start = clock();

//  // Time elapsed between measurements [sec]
//  double delta_t = 0.1;
//  // Sensor range [m]
//  double sensor_range = 50;


//  // Noise generation(normal distribution)
//  default_random_engine gen;
//  normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
//  normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
//  double n_range, n_heading;

//  // Read map data
//  Map map;
//  if (!read_map_data("data/map_data.txt", map))
//  {
//    cout << "Error: Could not open map file" << endl;
//    return -1;
//  }

//  // Read position data
//  vector<control_s> position_meas;
//  if (!read_control_data("data/control_data.txt", position_meas))
//  {
//    cout << "Error: Could not open position/control measurement file" << endl;
//    return -1;
//  }

//  // Read ground truth data
//  vector<ground_truth> gt;
//  if (!read_gt_data("data/gt_data.txt", gt))
//  {
//    cout << "Error: Could not open ground truth data file" << endl;
//    return -1;
//  }

//  // Run particle filter!
//  int num_time_steps = position_meas.size();

//  // Variables to keep track of the error
//  double total_error[3] = {0, 0, 0};
//  double cum_mean_error[3] = {0, 0, 0};

//  for (int i = 0; i < num_time_steps; ++i)
//  { // Main loop
//    cout << "\nTime step: " << i << endl;

//    // Read in landmark observations for current time step.
//    ostringstream file;
//    file << "data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
//    vector<LandmarkObs> observations;
//    if (!read_landmark_data(file.str(), observations))
//    {
//      cout << "Error: Could not open observation file " << i+1 << endl;
//      return -1;
//    }

//    // Initialize particle filter if this is the first time step.
//    if (!pf->initialized())
//    {

//    }
//    else
//    {
//      // Predict the vehicle's next state (noiseless).
//      pf->prediction(delta_t, sigma_pos, position_meas[i-1].velocity, position_meas[i-1].yawrate);
//    }

//    // Simulate the addition of noise to noiseless observation data.
//    vector<LandmarkObs> noisy_observations;
//    LandmarkObs obs;
//    for (int j = 0; j < observations.size(); ++j)
//    {
//      n_x = N_obs_x(gen);
//      n_y = N_obs_y(gen);
//      obs = observations[j];
//      obs.x = obs.x + n_x;
//      obs.y = obs.y + n_y;
//      noisy_observations.push_back(obs);
//    }

//    // Update the weights of the particles and resample
//    pf->updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
//    pf->resample();

//  }

//  // Output the runtime for the filter.
//  int stop = clock();
//  double runtime = (stop - start) / double(CLOCKS_PER_SEC);
//  cout << "Runtime (sec): " << runtime << endl;

//  // Print success if accuracy and runtime are sufficient
//  // NOTE: This isn't just for the starter code
//  if (runtime < max_runtime && pf->initialized())
//  {
//    cout << "Success! Your particle filter passed!" << endl;
//  }
//  else if (!pf->initialized())
//  {
//    cout << "This is the starter code. You haven't initialized your filter." << endl;
//  }
//  else
//  {
//    cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << endl;
//    return -1;
//  }

  return 0;
}

bool ParticleFilterNode::start()
{
  // Setup ROS stuff
  // Load maps
  if(!setup()) return false;
//  ROS_INFO_STREAM("Setup completed!");

  initPF();

  ros::spin();

  return true;
}

void ParticleFilterNode::evaluateParticlesObservation(const ros::Time &time)
{
  if(pfComunicationState != READY || lastSonSmall.empty())
  {
    ROS_WARN("ParticleFilter not ready to send msgs!");
    return;
  }

  const vector<Particle> &ps = pf->getParticles();

  // Predict particle pose on this instant
  pf->prediction(time.toSec(),sigma_pos);

  // Sending num. of particles
  {
    quad_amcl::StampedInteger msg;
    msg.header.stamp = time;
    msg.data = ps.size();
    pubNumParticles.publish(msg);
  }

  // Publishing son image
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                   "bgr8", lastSonSmall).toImageMsg();
    msg->header.stamp = time;
    pubSonImg.publish(msg);
  }

  // Publishing sat imgs (Particles)
  for(int i=0; i < ps.size(); i++)
  {
    const Particle &p = ps[i];


    Mat img = sat.cropSonarFoV(Point2d(p.x,p.y),
                               p.theta*180/M_PI,
                               sonarRange,
                               130.0);
    if(img.empty())
    {
      ROS_ERROR("Could not crop sat img, Si es");
      continue;
    }

    resize(img,img,Size(256,128));

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = time;
    pubSatImgs.publish(msg);
  }


  pfComunicationState = WAITING_EVALUATION;
  ROS_INFO("PF Waiting img evaluation!");

}

