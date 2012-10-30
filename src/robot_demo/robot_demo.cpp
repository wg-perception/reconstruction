#include <opencv2/opencv.hpp>
#include "modelCaptureInterface.hpp"
#include "dataGrabbing.hpp"
#include "rosUtils.hpp"

using namespace cv;
using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
  //TODO: use remapping
#ifdef USE_ON_PR2
  const string bgrImageTopic = "/head_mount_kinect/rgb/image_color";
  const string depthTopic = "/head_mount_kinect/depth/image";
#else
  const string bgrImageTopic = "/head_camera/rgb/image_color";
  const string depthTopic = "/head_camera/depth_registered/image";
#endif
//  const uint32_t queueSize = 15; //1
  const uint32_t queueSize = 2; //1

  const string basePath = (argc == 2) ? argv[1] : "";

  ros::init(argc, argv, "model_capture_demo");

#ifdef TEST_RVIZ_RENDERING
  cout << "Start publishing" << endl;
  for (size_t i = 0; i < 100; ++i)
  {
    publishOdometry(Mat::eye(4, 4, CV_64FC1), "RGBDOdometry");
    sleep(2);
  }

#endif

  //this initialization is needed for RViz
  Mat eye = Mat::eye(4, 4, CV_64FC1);
  //TODO: move up
  publishOdometry(eye, "RGBDOdometry");
  publishOdometry(eye, "LoopClosure");
  publishOdometry(eye, "RgbdICP");
  publishOdometry(eye, "RgbdICP_landmarks");

  RGBDGrabber rgbdGrabber(bgrImageTopic, depthTopic, queueSize);
//  RGBDExporter rgbdExporter(basePath);

  ros::spin();


  return 0;
}

