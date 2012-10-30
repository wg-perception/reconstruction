#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <opencv2/core/core.hpp>

//#define TEST_RVIZ_RENDERING
#define USE_ON_PR2

void publishOdometry(const cv::Mat &pose, const std::string &topicName = "odometry");
void publishOdometry(const std::vector<cv::Mat> &poses, const std::string &topicName = "odometry");

#endif
