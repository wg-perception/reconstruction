#ifndef DATA_GRABBINNG_HPP
#define DATA_GRABBINNG_HPP

#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "modelCaptureInterface.hpp"

class RGBDGrabber {
public:
  RGBDGrabber(const std::string &bgrImageTopic, const std::string &depthTopic, uint32_t queueSize);
  void callback(const sensor_msgs::ImageConstPtr &bgrImageMsg, const sensor_msgs::ImageConstPtr &depthMsg);
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport imageTransport;
  image_transport::SubscriberFilter bgrImageSubsriber, depthImageSubscriber;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> synchronizer;

  ModelCapturer modelCapturer; //we have to pass data to modelCapturer from callback so unfortunately we cannot separate classes
};


class RGBDExporter
{
  public:
    RGBDExporter(const std::string &basePath);

  private:
    ModelCapturer modelCapturer;
};

#endif
