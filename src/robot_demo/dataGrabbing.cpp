#include "dataGrabbing.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//TODO: use splitted code
#include <opencv_candidate_reconst3d/reconst3d.hpp>

using namespace cv;
using std::cout;
using std::endl;

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
  cout << "exit from callback" << endl;
}

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
