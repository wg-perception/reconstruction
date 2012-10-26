#include "rosUtils.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "poseRT.hpp"

//#define TEST_RVIZ_RENDERING
using namespace cv;

void publishOdometry(const std::vector<cv::Mat> &poses, const std::string &topicName)
{
  for (size_t i = 0; i < poses.size(); ++i)
  {
    publishOdometry(poses[i], topicName);
  }
}

void publishOdometry(const cv::Mat &poseMat, const std::string &topicName)
{
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

  static ros::Time current_time = ros::Time::now();

  //TODO: move up
  ros::Rate r(1.0);
  ros::spinOnce();               // check for incoming messages
  current_time = ros::Time::now();

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
//  odom_trans.header.frame_id = "odom";
//  odom_trans.header.frame_id = "base_link";
  //TODO: move up
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
  //TODO: move up
//  odom.header.frame_id = "odom";
//  odom.header.frame_id = "base_link";
  odom.header.frame_id = "head_mount_kinect_rgb_optical_frame";

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
  odom.child_frame_id = "base_link";
  //different values don't change RViz vizualization
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  //publish the message
  odom_pub.publish(odom);

  r.sleep();
  std::cout << "publishing..." << std::endl;
}
