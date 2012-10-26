#include "modelCaptureInterface.hpp"
#include "rosUtils.hpp"
//TODO: use splitted code
#include "../../opencv_candidate/src/rgbd/samples/model_capture/model_capture.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using std::cout;
using std::endl;

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
#if 1
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
