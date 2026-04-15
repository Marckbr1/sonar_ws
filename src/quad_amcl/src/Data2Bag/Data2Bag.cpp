#include "Data2Bag.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <omp.h>

/**
 * @brief Generator::loadSonCorrection - Load a sonar image given its ID from
 * the corrected dataset. If the image was not corrected, it returns
 * a id=-1 and an empty matrix.
 * @param dataFixerId - Input of the correction file ID.
 * @param fr - Output heading and position of the sonar image from correctio file.
 * @param sonImg - 8bits greyscale sonar image from dataset or an empty matrix
 * if the image was not corrected.
 * @return - True if the correction file exist, false otherwise.
 */
bool Data2Bag::loadSonCorrection(unsigned dataFixerId,
                                 DatasetFrame &fr)
{
  char str[200];
  sprintf(str,"dataFix_%05d.yml",dataFixerId);

  FileStorage fs;

  if(!fs.open((fixerFilesPath/str).string(),FileStorage::READ))
      return false;

  bool isFrameCarrected=false;

  fs["correct"] >> isFrameCarrected;
  fs["image_id"] >> fr.id;
  fs["timestamp"] >> fr.timestamp;
  fs["x"] >> fr.p.x;
  fs["y"] >> fr.p.y;
  fs["heading"] >> fr.heading;

  double radAng = fr.heading*M_PI/180.0;
  Point2d direction( sin(radAng),cos(radAng) );
  fr.c = fr.p + direction*sonarRange*0.5;

  if(!isFrameCarrected)
      fr.id =-1;

  fs.release();
  return true;
}

/**
 * @brief Generator::loadSatImg - Load a satellite image in sonar shape
 * regarding a givin position and orientation
 * @param fr - Input posution and orientation
 * @param rows - Final image rows
 * @param cols - Final image cols
 * @return - Satellite image in sonar shape croped from
 * the map.
*/
Mat Data2Bag::loadSatImg(DatasetFrame &fr, int rows, int cols)
{
  Mat satImg = sat.cropSonarFoV(fr.p,
                            fr.heading,
                            sonarRange,130.0);

  copyMakeBorder(satImg,satImg,7,0,2,2,
                 BORDER_CONSTANT,Scalar(0,0,0));

  resize(satImg,satImg,Size(cols,rows));

  return satImg;
}

Mat Data2Bag::loadSonImg(int frameId)
{
  if(frameId < 0)
  {
    cout << "Invalid son ID!" << endl;
    return Mat();
  }

  Mat sonImg;

  char str[600];
  sprintf(str,"%s/Sonar/imgs/xy_img_%05d.png",
          sourceData.c_str(),
          frameId);

  sonImg = imread(str,IMREAD_ANYDEPTH);

  return sonImg;
}

bool Data2Bag::verifyPaths()
{
  // Verify if it is a directory
  if(!is_directory(sourceData))
  {
      cout << "Wrong input data: " << sourceData.string()
           << " (must be a directory)" << endl;
      return false;
  }

  if(!is_directory(fixerFilesPath))
  {
      cout << "Wrong correction data: " << fixerFilesPath.string()
           << " (must be a directory)" << endl;
      return false;
  }

  if(!is_regular_file(mapPath))
  {
      cout << "Map file not found!" << endl;
      return false;
  }

  return true;
}

void Data2Bag::writeCompressedImg(const string &name, std_msgs::Header &h, Mat &m)
{
//  cv_bridge::CvImage img(h,"mono8",m);

  sensor_msgs::CompressedImage compressedImgMsg;

//  img.toCompressedImageMsg(compressedImgMsg,cv_bridge::PNG); // It does not work!!

  compressedImgMsg.format = "png";
  compressedImgMsg.header = h;
  compressedImgMsg.header.frame_id = "son";

  std::vector<int> param(2);
  param[0] = cv::IMWRITE_PNG_COMPRESSION;
  param[1] = 9;

  // Compress the image using OpenCV
  cv::imencode(".png", m, compressedImgMsg.data, param);

  // Write the compressed image
  outBag_.write(name+"/compressed",
               h.stamp,
                compressedImgMsg);
}

void Data2Bag::writeImg(string name, std_msgs::Header &h, Mat &m)
{
  cv_bridge::CvImage img(h,"bgr8",m);

  // Write the compressed image
  outBag_.write(name,
               h.stamp,
               img.toImageMsg());
}

Data2Bag::Data2Bag()
{
  cout << "Initializing data2bag" << endl;
}

void Data2Bag::start()
{
  vector<DatasetFrame> frames;

  // Load satellite map, and full dataset with heading correction
  if(!loadDataset(frames))
  {
    cout << "Could not load dataset!" << endl;
    return ;
  }

  int firstMsg=0, nMsgs=frames.size();
  bool writeImgs=false, removeOffset=false;

  // Create out BagFile
  outBag_.open(outBagFileName.string(),
               rosbag::bagmode::Write);

  // Open file
  FILE *f =0x0;
  if(writeImgs)
    f = fopen("/home/auros/Desktop/Son/gt.csv","w");

  int msgCount=0;

//  namedWindow("son",WINDOW_NORMAL);
//  namedWindow("sat",WINDOW_NORMAL);

  Mat son16Bits, sonImg, satImg, binSat;

  char c =0;
  double fx=0.0,fy=0.0;

  // Process all dataset
  for(int i = firstMsg; i <= nMsgs && c != 32 ; i++)
  {

    DatasetFrame &fr = frames[i];
    if(fr.id < 0)
    {
      cout << "Frame ID error!" << endl;
      continue;
    }

    son16Bits = loadSonImg(fr.id);

    if(son16Bits.empty())
    {
      cout << "Skipping sonar image: " << fr.id << endl;
      continue;
    }else
    {
//        sonImg.convertTo(sonImg,CV_8UC1,1.0,-100.0);
      son16Bits.convertTo(sonImg,CV_8UC1,1.0,0.0);
//        cvtColor(sonImg,sonImg,CV_GRAY2BGR);
//      son16Bits.copyTo(sonImg);
    }

    satImg = loadSatImg(fr,
                        sonImg.rows,sonImg.cols);

    if(satImg.empty())
    {
      cout << "Skipping sat image " << fr.id << endl;
      continue;
    }

    if(i%100 == 0) // Feedback to the user (we still alive!)
    {
      cout << "Processing frame "
           << i << " of "
           << frames.size() << endl;

    }

    if(i == firstMsg && removeOffset)
    {
      fx= fr.p.x;
      fy= fr.p.y;
    }

    // Writing data into bag file.
    std_msgs::Header header;
    header.stamp = ros::Time(fr.timestamp);
    header.seq = msgCount++;
    header.frame_id = "odom";

    geometry_msgs::PoseStamped poseMsg;
    poseMsg.pose.position.x = fr.p.x-fx;
    poseMsg.pose.position.y = fr.p.y-fy;
    poseMsg.pose.position.z = 0.0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0,0.0,fr.heading*M_PI/180.0);
    tf2::convert(quat_tf,poseMsg.pose.orientation);

    poseMsg.header = header;

    outBag_.write("/pose_gt",header.stamp,poseMsg);

    writeCompressedImg("/son",header,sonImg);

    // Write imgs on folder
    if(writeImgs)
    {
      fprintf(f,"%04d, %.3f, %.3f, %.3f\n",i,fr.p.x-fx,fr.p.y-fy,fr.heading);

      string fileName;

      fileName = format("/home/auros/Desktop/Son/son/son_%04d.png",i);
      imwrite(fileName,sonImg);

      fileName = format("/home/auros/Desktop/Son/sat/sat_%04d.png",i);
      imwrite(fileName,satImg);
    }

    // Show result
//    imshow("son", sonImg);
//    imshow("sat", satImg);

//    c=waitKey(10);
  }

  if(writeImgs)
    fclose(f); // Close file

  // Close out bag file
  outBag_.close();
}

bool Data2Bag::loadDataset(vector<DatasetFrame> &frames)
{
  cout << "Loading datasets!" << endl;
  if(!verifyPaths())
    return false;


  // Load the full dataset
  {
    cout << "Loading full dataset!" << endl;
    DatasetReader dr;
    dr.loadDataset(sourceData,frames);
    sonarRange = dr.sonarRange;
    cout << "Full dataset loaded!" << endl;
  }

  if(!sat.loadMap(mapPath))
  {
      cout << "Error - Map could not be loaded!" << endl;
      return false;
  }


  // Apply heading correction
  DatasetFrame frCorrected;

  // For all corrected images
  for(uint i = 0; loadSonCorrection(i,frCorrected); i++)
  {
    if(frCorrected.id >= 0 )
    {
      if(frCorrected.id != frames[i].id)
      {
        cout << "Index error on compass correction and loaded dataset!" << endl;
        continue;
      }
      // Correct heading
      frames[i].heading = frCorrected.heading;
    }
  }
  return true;
}

int main(int argc, char *argv[])
{
  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : " << CV_MAJOR_VERSION << endl;
  cout << "Minor version : " << CV_MINOR_VERSION << endl;
  cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;

    ros::init(argc, argv, "data_2_bag");

    Data2Bag g;

 //   g.sourceData = "/media/matheusbg8/Machado/DocData/Dataset/Yacht_26_09_2017";
//    g.sourceData = "/media/matheusbg8/Backup Plus/HD_02/DocData/Dataset/Yacht_26_09_2017";
    g.sourceData = "/media/auros/Backup_Plus/HD_02/DocData/Dataset/Yacht_26_09_2017";

//    g.fixerFilesPath = "/media/matheusbg8/Machado/DocData/Yacht_26_09_2017_small_edition2_backup/";
//    g.fixerFilesPath = "/media/matheusbg8/Backup Plus/HD_02/DocData/Yacht_26_09_2017_small_edition2_backup/";
//    g.fixerFilesPath = "/home/matheusbg8/Documents/Doc/Datasets/ARACATI_2017_fix";
//    g.fixerFilesPath = "/media/auros/AuRos_Xavier/tmp2/ARACATI_2017_fix";
    g.fixerFilesPath = "/media/rafael/AuRos_Xavier/tmp2/ARACATI_2017_fix";

//    g.mapPath = "/media/matheusbg8/Machado/DocData/Maps/Yacht_Club_RG/2017-08-06.ini";
//    g.mapPath = "/media/matheusbg8/Backup Plus/HD_02/DocData/Maps/Yacht_Club_RG/2017-08-06.ini";
    g.mapPath = "/media/auros/Backup_Plus/HD_02/DocData/Maps/Yacht_Club_RG/2017-08-06.ini";

//    g.outBagFileName = "/media/matheusbg8/Backup Plus/Doc/Datasets/ARACATI_2017_new.bag";
//    g.outBagFileName = "/media/auros/AuRos_Xavier/tmp2/ARACATI_2017_8bits_new.bag";
    g.outBagFileName = "/media/rafael/AuRos_Xavier/tmp2/ARACATI_2017_8bits_new.bag";
    g.start();
}

