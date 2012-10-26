#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//publish Odometry messages
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
//TODO: use splitted code
#include "../../opencv_candidate/src/rgbd/samples/model_capture/model_capture.hpp"
#include "poseRT.hpp"


using namespace cv;
using std::cout;
using std::endl;

void publishOdometry(const cv::Mat &pose, const std::string &topicName = "odometry");
void publishOdometry(const std::vector<cv::Mat> &poses, const std::string &topicName = "odometry");

class ModelCapturer
{
  public:
    ModelCapturer();
    void addRGBDFrame(const cv::Mat &bgrImage, const cv::Mat &depth);
    void createModel();
  private:
    std::vector<cv::Mat> allBgrImages, allDepths;
    bool isLoopClosed;
};

ModelCapturer::ModelCapturer()
{
  isLoopClosed = false;
}

void ModelCapturer::addRGBDFrame(const cv::Mat &bgrImage, const cv::Mat &depth)
{
  if (isLoopClosed)
    return;

  std::cout << "adding the frame #" << allBgrImages.size() << std::endl;

  CV_Assert(bgrImage.type() == CV_8UC3);
  CV_Assert(depth.type() == CV_32FC1);
  CV_Assert(bgrImage.size() == depth.size());

  allBgrImages.push_back(bgrImage);
  allDepths.push_back(depth);

  //TODO: remove, use loop closure detection from the algorithm
  imshow("bgr view", bgrImage);
  imshow("depth view", depth);
  //TODO: move up
  int key = waitKey(30);
  if (key == 27) //escape
  {
    isLoopClosed = true;
    createModel();
  }
}

void ModelCapturer::createModel()
{
  CV_Assert(allBgrImages.size() == allDepths.size());
  //TODO: remove dumping
#if 0
  cout << "number of collected images:" << allBgrImages.size() << endl;
  for (size_t i = 0; i < allBgrImages.size(); ++i)
  {
//    imshow("p i", allBgrImages[i]);
//    imshow("p d", allDepths[i]);
//    waitKey();
    std::stringstream bgrImageFilename;
    bgrImageFilename << "data/image_" << std::setfill('0') << std::setw(5) << i << ".png";
    cout << bgrImageFilename.str() << endl;
    imwrite(bgrImageFilename.str(), allBgrImages[i]);

    std::stringstream depthFilename;
    depthFilename << "data/depth_image_" << std::setfill('0') << std::setw(5) << i << ".xml.gz";
    FileStorage fs(depthFilename.str(), FileStorage::WRITE);
    fs << "depth_image" << allDepths[i];
    fs.release();
  }
#endif




  //TODO: use the function from upcoming splitted code



  Ptr<Odometry> odometry = Algorithm::create<Odometry>("RGBD.RgbdOdometry");
  if(odometry.empty())
  {
      cout << "Can not create Odometry algorithm. Check the passed odometry name." << endl;
      return -1;
  }
  float vals[] = {525., 0., 3.1950000000000000e+02,
                  0., 525., 2.3950000000000000e+02,
                  0., 0., 1.};
  Mat cameraMatrix = Mat(3,3,CV_32FC1,vals).clone();
  odometry->set("cameraMatrix", cameraMatrix);

  // Create normals computer
  Ptr<RgbdNormals> normalsComputer = new cv::RgbdNormals(allDepths[0].rows, allDepths[0].cols, allDepths[0].depth(), cameraMatrix);

  // Fill vector of initial frames
  vector<Ptr<OdometryFrameCache> > frames;
  vector<Mat> tableMasks, tableWithObjectMasks;
  for(size_t i = 0; i < allBgrImages.size(); i++)
  {
      Ptr<OdometryFrameCache> frame;

      Mat gray;
      cvtColor(allBgrImages[i], gray, CV_BGR2GRAY);
      {
          Mat tmp;
          medianBlur(gray, tmp, 3);
          gray = tmp;
      }

      Mat cloud;
      depthTo3d(allDepths[i], cameraMatrix, cloud);

      Mat normals = (*normalsComputer)(cloud);

      Mat tableWithObjectMask, tableMask;
      cout << "Masking the frame " << i << endl;
      if(!computeTableWithObjectMask(cloud, normals, cameraMatrix, tableWithObjectMask, 0.1, &tableMask))
      {
          cout << "Skip the frame because calcTableWithObjectMask was failed" << endl;
      }
      else
      {
          frame = new OdometryFrameCache();
          frame->image = gray;
          frame->depth = allDepths[i];
          CV_Assert(!tableWithObjectMask.empty());
          frame->mask = tableWithObjectMask;
          frame->normals = normals;
      }

      frames.push_back(frame);
      tableMasks.push_back(tableMask.clone());
      tableWithObjectMasks.push_back(tableWithObjectMask.clone());
  }

  vector<Ptr<OdometryFrameCache> > keyframes;
  vector<Mat> keyframePoses;
  vector<int> indicesToBgrImages; // to frames vector
  if(!frameToFrameProcess(frames, cameraMatrix, odometry, keyframes, keyframePoses, &indicesToBgrImages))
      return -1;
#if 0
  for(size_t i = 0; i < frames.size(); i++)
  {
      if(find(indicesToBgrImages.begin(), indicesToBgrImages.end(), i) == indicesToBgrImages.end())
      {
          frames[i].release();
          allBgrImages[i].release();
          allDepths[i].release();
      }
  }
#endif

  //TODO: move up
  cout << "publishing odometry... " << keyframePoses.size() << endl;
  publishOdometry(keyframePoses, "RGBDOdometry");

  cout << "Frame-to-frame odometry result" << endl;
  showModel(allBgrImages, indicesToBgrImages, keyframes, keyframePoses, cameraMatrix, 0.005);

  vector<Mat> refinedPosesSE3;
  refineSE3Poses(keyframePoses, refinedPosesSE3);

  //TODO: move up
  publishOdometry(refinedPosesSE3, "LoopClosure");

  cout << "Result of the loop closure" << endl;
  showModel(allBgrImages, indicesToBgrImages, keyframes, refinedPosesSE3, cameraMatrix, 0.003);

  vector<Mat> refinedPosesICPSE3;
  float pointsPart = 0.05f;
  refineRgbdICPSE3Poses(keyframes, refinedPosesSE3, cameraMatrix, pointsPart, refinedPosesICPSE3);

  //TODO: move up
  publishOdometry(refinedPosesICPSE3, "RgbdICP");

  cout << "Result of RgbdICP for camera poses" << endl;
  float modelVoxelSize = 0.003f;
  showModel(allBgrImages, indicesToBgrImages, keyframes, refinedPosesICPSE3, cameraMatrix, modelVoxelSize);

#if 1
  // remove table from the further refinement
  for(size_t i = 0; i < keyframes.size(); i++)
  {
      keyframes[i]->mask = tableWithObjectMasks[indicesToBgrImages[i]] & ~tableMasks[indicesToBgrImages[i]];
      keyframes[i]->pyramidMask.clear();
      keyframes[i]->pyramidTexturedMask.clear();
      keyframes[i]->pyramidNormalsMask.clear();
      keyframes[i]->pyramidMask.clear();
  }
  pointsPart = 1.f;
  modelVoxelSize = 0.001;
#endif

  vector<Mat> refinedPosesICPSE3Landmarks;
  refineICPSE3Landmarks(keyframes, refinedPosesICPSE3, cameraMatrix, refinedPosesICPSE3Landmarks);

  //TODO: move up
  publishOdometry(refinedPosesICPSE3Landmarks, "RgbdICP_landmarks");

  cout << "Result of RgbdICP for camera poses and moving the model points" << endl;
  modelVoxelSize = 0.000001;
  showModel(allBgrImages, indicesToBgrImages, keyframes, refinedPosesICPSE3Landmarks, cameraMatrix, modelVoxelSize);
//    showModelWithNormals(allBgrImages, indicesToBgrImages, keyframes, refinedPosesICPSE3Landmarks, cameraMatrix);

}

class RGBDGrabber {
public:
  RGBDGrabber(const string &bgrImageTopic, const string &depthTopic, uint32_t queueSize);
  void callback(const sensor_msgs::ImageConstPtr &bgrImageMsg, const sensor_msgs::ImageConstPtr &depthMsg);
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport imageTransport;
  image_transport::SubscriberFilter bgrImageSubsriber, depthImageSubscriber;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> synchronizer;

  ModelCapturer modelCapturer; //we have to pass data to modelCapturer from callback so unfortunately we cannot separate classes
};

RGBDGrabber::RGBDGrabber(const string &bgrImageTopic, const string &depthTopic, uint32_t queueSize) :
    imageTransport(nh),
    bgrImageSubsriber(imageTransport, bgrImageTopic, queueSize),
    depthImageSubscriber(imageTransport, depthTopic, queueSize),
    synchronizer(SyncPolicy(queueSize), bgrImageSubsriber, depthImageSubscriber)
{
  synchronizer.registerCallback(boost::bind(&RGBDGrabber::callback, this, _1, _2));
}

void RGBDGrabber::callback(const sensor_msgs::ImageConstPtr &bgrImageMsg, const sensor_msgs::ImageConstPtr &depthMsg)
{
  ROS_INFO("calling callback");

  cv_bridge::CvImageConstPtr cvPtrBgrImage, cvPtrDepthImage;
  try
  {
//    cvPtrBgrImage = cv_bridge::toCvShare(bgrImageMsg, sensor_msgs::image_encodings::BGR8);
    cvPtrBgrImage = cv_bridge::toCvCopy(bgrImageMsg, sensor_msgs::image_encodings::BGR8);
    Mat bgrImage = cvPtrBgrImage->image;
    CV_Assert(bgrImage.type() == CV_8UC3);

//    cvPtrDepthImage = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);
    cvPtrDepthImage = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);
    Mat depth = cvPtrDepthImage->image;
    CV_Assert(depth.type() == CV_32FC1);

    modelCapturer.addRGBDFrame(bgrImage, depth);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

class RGBDExporter
{
  public:
    RGBDExporter(const std::string &basePath);

  private:
    ModelCapturer modelCapturer;
};

RGBDExporter::RGBDExporter(const std::string &basePath)
{
  vector<Mat> bgrImages, depths;
  loadTODLikeBase(basePath, bgrImages, depths);

  for (size_t i = 0; i < bgrImages.size(); ++i)
  {
    modelCapturer.addRGBDFrame(bgrImages[i], depths[i]);
  }
  modelCapturer.createModel();
}

int main(int argc, char *argv[])
{
  //TODO: use remapping
//  const string bgrImageTopic = "/head_camera/rgb/image_color";
//  const string depthTopic = "/head_camera/depth_registered/image";
  const string bgrImageTopic = "/head_mount_kinect/rgb/image_color";
  const string depthTopic = "/head_mount_kinect/depth/image";
  const uint32_t queueSize = 15; //1

  const string basePath = (argc == 2) ? argv[1] : "";

  ros::init(argc, argv, "model_capture_demo");

  namedWindow("bgr view");
  namedWindow("depth view");
  startWindowThread();

#if 0
  cout << "Start publishing" << endl;
  for (size_t i = 0; i < 100; ++i)
  {
    publishOdometry(Mat::eye(4, 4, CV_64FC1), "RGBDOdometry");
  }
#endif

//     RGBDGrabber rgbdGrabber(bgrImageTopic, depthTopic, queueSize);
  RGBDExporter rgbdExporter(basePath);

  ros::spin();

  return 0;
}


void publishOdometry(const std::vector<cv::Mat> &poses, const std::string &topicName)
{
  for (size_t i = 0; i < poses.size(); ++i)
  {
    publishOdometry(poses[i], topicName);
  }
}

void publishOdometry(const cv::Mat &poseMat, const std::string &topicName)
{
#if 0
  static ros::NodeHandle nh;

  static std::map<string, ros::Publisher> odometryPublishers;
  bool doesPublisherExist = (odometryPublishers.find(topicName) != odometryPublishers.end());
  ros::Publisher &odom_pub = odometryPublishers[topicName];
  if (!doesPublisherExist)
  {
    //TODO: move up
    const uint32_t queueSize = 50;
    odom_pub = nh.advertise<nav_msgs::Odometry>(topicName, queueSize);
  }

  tf::TransformBroadcaster odom_broadcaster;

  PoseRT pose(poseMat);
  Mat tvec = pose.getTvec();
  CV_Assert(tvec.type() == CV_64FC1);

  double x = tvec.at<double>(0);
  double y = tvec.at<double>(1);
  double z = tvec.at<double>(2);

#if 0
  static double z_static = 0.0;
  x = 0.0;
  y = 0.0;
  z = z_static;
  z_static += 0.1;
#endif

  Mat quaternionMat = pose.getQuaternion();
  CV_Assert(quaternionMat.type() == CV_64FC1);
  geometry_msgs::Quaternion quaternion;
  quaternion.x = quaternionMat.at<double>(0);
  quaternion.y = quaternionMat.at<double>(1);
  quaternion.z = quaternionMat.at<double>(2);
  quaternion.w = quaternionMat.at<double>(3);

#if 0
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 0.0;
  quaternion.w = 1.0;
#endif

  static ros::Time current_time = ros::Time::now();

  ros::Rate r(1.0);
  ros::spinOnce();               // check for incoming messages
  current_time = ros::Time::now();

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
//  odom_trans.header.frame_id = "odom";
//  odom_trans.header.frame_id = "base_link";
  odom_trans.header.frame_id = "head_mount_kinect_rgb_optical_frame";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = z;

  odom_trans.transform.rotation = quaternion;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
//  odom.header.frame_id = "odom";
//  odom.header.frame_id = "base_link";
  odom.header.frame_id = "head_mount_kinect_rgb_optical_frame";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;

  odom.pose.pose.orientation = quaternion;

#if 0
  cout << x << " " << y << " " << z << endl;
  cout << quaternion.x << " " << quaternion.y << " " << quaternion.z << " " << quaternion.w << endl;
#endif

  //set the velocity
  odom.child_frame_id = "base_link";
  //different values don't change RViz vizualization
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  //publish the message
  odom_pub.publish(odom);

  //sleep(3);
  r.sleep();
  std::cout << "publishing..." << std::endl;
#endif 
}
