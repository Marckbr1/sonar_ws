#include "ImageLoader.h"

//bool ImageLoader::listVideo(path folder, vector<path> &videoNames)
//{
//    videoNames.clear();

//    if(!is_directory(folder))
//    {
//        ROS_ERROR("Invalid Directory: %s",
//                  folder.c_str());

//        return false;
//    }

//    for (directory_iterator itr(folder); itr!=directory_iterator(); ++itr)
//    {
//        path ext = itr->path().extension();


//        if( ext == ".mp4" || ext == ".MP4" || ext == ".avi" ||
//            ext == ".AVI" || ext == ".mkv" || ext == ".ogv" || ext == ".MOV")
//        {
//            videoNames.push_back(itr->path());
//        }
//    }

//    return true;
//}

bool ImageLoader::setup()
{
  // Image folder
  string folder;
  if(!pnh.getParam("folder", folder))
  {
    ROS_ERROR("Img. folder not specified!!");
    return false;
  }
  imgFolder = folder;
  ROS_INFO_STREAM("Loading imgs from folder " << imgFolder);

  // Topics advertisement
  pubSatImgs = it.advertise("/sat_crop",100);
  pubSonImg = it.advertise("/son",1);
  pubNumParticles = nh.advertise<quad_amcl::StampedInteger>("/n_particles",1);

  return true;
}

ImageLoader::ImageLoader():
    pnh("~"),
    it(nh),
    r(0.25)
{
}

bool ImageLoader::start()
{
  if(!setup()) return false;

  while (ros::ok())
  {
    publishMsgs();
    r.sleep();
    ros::spinOnce();
  }
  return true;
}

void ImageLoader::publishMsgs()
{
  char str[200];
  int nParticles=80;

  ros::Time time = ros::Time::now();

  // Sending num. of particles
  {
    quad_amcl::StampedInteger msg;
    msg.header.stamp = time;
    msg.data = nParticles;
    pubNumParticles.publish(msg);
  }

  // Publishing sat imgs (Particles)
  for(int i=0; i < nParticles; i++)
  {
    sprintf(str,"sat_%05d.png",i);
    Mat img = imread((imgFolder / str).string());
    if(img.empty()) continue;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = time;
    pubSatImgs.publish(msg);
  }

  // Publishing son image
  {
    Mat sonImg = imread((imgFolder / "son_00066.png").string());
    if(sonImg.empty())
    {
      ROS_ERROR_STREAM("Couldn't find sonar image on folder " << imgFolder);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sonImg).toImageMsg();
    msg->header.stamp = time;
    pubSonImg.publish(msg);
  }
}

