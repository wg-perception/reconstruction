#include "rosUtils.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "poseRT.hpp"


using namespace cv;
using std::cout;
using std::endl;

void publishOdometry(const std::vector<cv::Mat> &poses, const std::string &topicName)
{
  for (size_t i = 0; i < poses.size(); ++i)
  {
    publishOdometry(poses[i], topicName);
  }
}

void publishOdometry(const cv::Mat &poseMat, const std::string &topicName)
{
  std::cout << "publishing..." << std::endl;
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

  PoseRT pose(poseMat);
  Mat tvec = pose.getTvec();
  CV_Assert(tvec.type() == CV_64FC1);

  double x = tvec.at<double>(0);
  double y = tvec.at<double>(1);
  double z = tvec.at<double>(2);

#ifdef TEST_RVIZ_RENDERING
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

#ifdef TEST_RVIZ_RENDERING
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 0.0;
  quaternion.w = 1.0;
#endif

  //TODO: move up
//  const string frame = "odom_combined";
//  cosnt string frame = "head_mount_kinect_rgb_optical_frame";
  const string rgbdOdometryFrame = "odom_rgbd";
#ifdef USE_ON_PR2
  const string rosOdometryFrame = "/odom_combined";
  const string kinectFrame = "/head_mount_kinect_rgb_optical_frame";
#else
  const string rosOdometryFrame = "/odom";
  const string kinectFrame = "/head_camera_rgb_optical_frame";
#endif

  //TODO: move up
  ros::Rate r(100.0);
  ros::Time current_time = ros::Time::now();

  static tf::StampedTransform transform;
  static bool isTransformInitialized = false;
  if (!isTransformInitialized)
  {
      tf::TransformListener listener;
      //TODO: move up
      ros::spinOnce();
      listener.waitForTransform(rosOdometryFrame, kinectFrame, current_time, ros::Duration(10.0));
      listener.lookupTransform(rosOdometryFrame, kinectFrame, ros::Time(0), transform);
      transform.child_frame_id_ = rgbdOdometryFrame;
      isTransformInitialized = true;
  }
  transform.stamp_ = current_time;

  static tf::TransformBroadcaster br;
  br.sendTransform(transform);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = rgbdOdometryFrame;

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;

  odom.pose.pose.orientation = quaternion;

#ifdef TEST_RVIZ_RENDERING
  cout << x << " " << y << " " << z << endl;
  cout << quaternion.x << " " << quaternion.y << " " << quaternion.z << " " << quaternion.w << endl;
#endif

  //set the velocity
  odom.child_frame_id = rgbdOdometryFrame;
  //different values don't change RViz vizualization
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  //publish the message
  odom_pub.publish(odom);

  r.sleep();
  std::cout << "Done..." << std::endl;
}
