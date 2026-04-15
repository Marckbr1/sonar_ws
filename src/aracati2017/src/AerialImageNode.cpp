#include "AerialImageNode.h"
#include <tf2/utils.h>

// bool AerialImageNode::getUTMRef()
// {
//   ROS_INFO("Waiting first GPS msg to estimate UTM ref.");
//   sensor_msgs::NavSatFix::ConstPtr msg =
//     ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/dgps");

//   if (msg != nullptr)
//   {
//     // Get UTM Ref
//     Point2d latLong(msg->latitude,msg->longitude);
//     utmRef = latLon2UTM(latLong);
//     hasUTMRef = true;
//     return true;
//   }
//   return false;
// }

bool AerialImageNode::getUTMRef()
{
  ROS_INFO("Waiting first GPS msg to estimate UTM ref.");
  
  // Esperar SOLO 2 segundos
  sensor_msgs::NavSatFix::ConstPtr msg =
    ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/dgps", ros::Duration(2.0));
  
  if (msg != nullptr)
  {
    Point2d latLong(msg->latitude, msg->longitude);
    utmRef = latLon2UTM(latLong);
    hasUTMRef = true;
    ROS_INFO("Got real GPS reference");
    return true;
  }
  
  // Si no hay GPS, usar origen (0,0)
  ROS_WARN("No GPS, using (0.0, 0.0) as reference");
  utmRef = Point2d(0.0, 0.0);  // Referencia en (0,0)
  hasUTMRef = true;
  return true;  // Importante: retornar true para continuar
}



bool AerialImageNode::initROS()
{
  ros::NodeHandle nh("~");

  // Get parameters:
  string aerial_img_path;
  if(!nh.getParam("aerial_image_path",aerial_img_path))
  {
    ROS_ERROR("Missing parameter aerial_image_path point to the .yaml file.");
    return false;
  }

  // Load aerial image
  if(!ai.loadMap(aerial_img_path))
  {
    ROS_ERROR("It could not load the aerial image: %s",
              aerial_img_path.c_str());
    return false;
  }
//  ai.resizeToMaxCols(800);

  // Load sonar FoV shape
  double sonOpenning, sonMinRange, sonMaxRange;
  nh.param<double>("son_fov_openning", sonOpenning, 130.0); // BlueView P900-130
  nh.param<double>("son_min_range", sonMinRange, 0.4); // BlueView P900 can be set from 0.4m to 100m
  nh.param<double>("son_max_range", sonMaxRange, 50.0); // BlueView P900 can be set from 0.4m to 100m (aracati2017 use 50m)

  sonShape.initShape(sonOpenning,
                     sonMaxRange,
                     sonMinRange);

  // Topic Subscriptions
  subGtPose = n.subscribe("/pose_gt",5,
        &AerialImageNode::gtPoseCallback,this);

  subOdomPose = n.subscribe("/odom_pose",5,    
        &AerialImageNode::odomPoseCallback,this);  // yo hice esto 

  // Topic Advertisements
  pubAerialImgs = it.advertise("/aerial_img",1);
  pubSonAerial = it.advertise("/son_aerial",1);

  // Get a UTM reference to the vehicle position
  // We assume the first DGPS msg (Ground Truth)
  // match the first vehicle position (It is true
  // on dataset aracati2017).

  // Otherwise the time difference between gps
  // and first vehicle position should be take
  // in account and the UTMRef should be interpolated
  // to get an approximated UTMRef.

  // It is not a good idea to use UTM in ROS, especially
  // on TF tree. We had some problems working
  // with high values, so we decided to remove the
  // UTM offset in the dataset.
  getUTMRef(); // This may wait forever (No timeout)

  aerialImgHeader.frame_id = "aerial_image";
  aerialImgHeader.seq = 0;

  return true;
}

void AerialImageNode::odomPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if(hasUTMRef)
  {
    Point2d UTMSonPosition = utmRef + Point2d(msg.pose.position.x,
                                           msg.pose.position.y);

    odomPts.push(UTMSonPosition);
  }
}

void AerialImageNode::gtPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if(hasUTMRef)
  {
    Point2d UTMSonPosition = utmRef + Point2d(msg.pose.position.x,
                                           msg.pose.position.y);

    gtPts.push(UTMSonPosition);
    // lastHeading = tf2::getYaw(msg.pose.orientation) ;
    lastHeading = tf2::getYaw(msg.pose.orientation) * 180.0 / M_PI ;
    ROS_INFO("Heading: %.1f grados", lastHeading);



    // double heading_NED = M_PI/2 - lastHeading;
    // if(heading_NED < 0) heading_NED += 2*M_PI;
    // ROS_INFO("Heading ENU: %.1f°, Heading NED: %.1f°", 
    //      lastHeading*180/M_PI, heading_NED*180/M_PI);


    // // Calcular el heading solo la primera vez
    // static bool headingInitialized = false;
    // static double fixedHeading = 0.0;
    
    // if(!headingInitialized) {
    //   fixedHeading = tf2::getYaw(msg.pose.orientation);
    //   // Normalizar si quieres
    //   if(fixedHeading < 0) fixedHeading += 2 * M_PI;
    //   headingInitialized = true;
    //   ROS_INFO("Orientación fija inicializada: %.1f grados", fixedHeading * 180.0 / M_PI);
    // }
    
    // lastHeading = fixedHeading;  // Siempre el mismo valor

    // DEBUG: Imprime valores
    // ROS_INFO("=== GT POSE CALLBACK ===");
    // ROS_INFO("Position UTM: (%.2f, %.2f)", UTMSonPosition.x, UTMSonPosition.y);
    // ROS_INFO("Heading: %.3f rad (%.1f grados)", lastHeading, lastHeading * 180.0 / M_PI);
    
    // // Verifica si está en rango esperado
    // if(lastHeading < -M_PI || lastHeading > M_PI) {
    //   ROS_WARN("Heading fuera de rango!");
    // }


  }
}

void AerialImageNode::publishAerialImg()
{
  Mat aerialImg = ai.getMapImg().clone();

  aerialImgHeader.stamp = ros::Time::now();

  // Too many point so we drop a few
  // when drawing...

  if(odomPts.size() > 0)
  {
      Point p = ai.UTM2Img(odomPts[odomPts.size()-1]);
      // cout << "pixel odom: " << p << endl;
      // cout << "Real Odom: " << odomPts[odomPts.size()-1] << endl;
  }
  // Draw Odom Lines on aerial image
  for(uint i =10; i < odomPts.size();i+=10)
    line(aerialImg,
         ai.UTM2Img(odomPts[i-10]),
         ai.UTM2Img(odomPts[i]),
         Scalar(255,0,0),2);
    

  // Draw GT Lines on aerial image
  for(uint i =10; i < gtPts.size();i+=10)
    line(aerialImg,
        ai.UTM2Img(gtPts[i-10]),
        ai.UTM2Img(gtPts[i]),
        Scalar(0,255,0),2);
  // cout << "odomPts size: " << odomPts.size() << endl;
  // cout << "gtPts size: " << gtPts.size() << endl;
  // cout << "map size: " << aerialImg.cols << " x " << aerialImg.rows << endl;
  if(!gtPts.empty())
  {
    const Point2d &UTMSonPosition = gtPts.last();
      
    // // DEBUG: Imprime información
    // cout << "=== SONAR DEBUG ===" << endl;
    // cout << "UTM Position: (" << UTMSonPosition.x << ", " << UTMSonPosition.y << ")" << endl;
    // cout << "Heading: " << lastHeading * 180/M_PI << " degrees" << endl;

    sonShape.drawPoly(aerialImg,ai,
                      UTMSonPosition,
                      lastHeading,
                      Scalar(255,0,255),
                      2);

    // Publish sonar aerial image
    Mat sonAerialImg = sonShape.cropSonShape(ai,
                       UTMSonPosition,
                       lastHeading);

    sensor_msgs::ImagePtr imgMsg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", sonAerialImg).toImageMsg();

    imgMsg->header = aerialImgHeader;
    pubSonAerial.publish(imgMsg);
  }

  // Publish aerial image
  sensor_msgs::ImagePtr imgMsg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", aerialImg).toImageMsg();

  imgMsg->header = aerialImgHeader;
  pubAerialImgs.publish(imgMsg);

  aerialImgHeader.seq++;
}

AerialImageNode::AerialImageNode():
  it(n),
  odomPts(6000),
  gtPts(6000)
{

}

void AerialImageNode::start()
{
  initROS();
  ros::Rate r(4);

  while(ros::ok())
  {
    publishAerialImg();
    ros::spinOnce();
    r.sleep();
  }

}
