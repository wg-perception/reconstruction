#include "modelCaptureInterface.hpp"
#include "rosUtils.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/rgbd.hpp>

#include "../../../opencv_candidate/src/reconst3d/graph_optimizations.hpp"

using namespace cv;
using std::cout;
using std::endl;

//#define SAVE_COLLECTED_DATA

static void
preparePosesLinksWithoutRt(const vector<PosesLink>& srcLinks, vector<PosesLink>& dstLinks)
{
    dstLinks.resize(srcLinks.size());
    for(size_t i = 0; i < dstLinks.size(); i++)
        dstLinks[i] = PosesLink(srcLinks[i].srcIndex, srcLinks[i].dstIndex);
}

static Mat
refineObjectMask(const Mat& initObjectMask)
{
    vector<vector<Point> > contours;
    Mat initObjectMaskClone = initObjectMask.clone();
    findContours(initObjectMaskClone, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    if(contours.empty())
        return initObjectMask.clone();

    int maxMaskArea = 0.;
    int objectMaskIndex = -1;
    for(size_t i = 0; i < contours.size(); i++)
    {
        Mat mask(initObjectMask.size(), CV_8UC1, Scalar(0));
        drawContours(mask, contours, i, Scalar(255), CV_FILLED, 8);

        int curMaskArea = countNonZero(mask);
        if(curMaskArea > maxMaskArea)
        {
            maxMaskArea = curMaskArea;
            objectMaskIndex = i;
        }
    }

    Mat objectMask(initObjectMask.size(), CV_8UC1, Scalar(0));
    drawContours(objectMask, contours, objectMaskIndex, Scalar(255), CV_FILLED, 8);

    return objectMask & initObjectMask;
}

static void
prepareFramesForModelRefinement(const Ptr<TrajectoryFrames>& trajectoryFrames, const vector<int>& frameIndices, vector<Ptr<RgbdFrame> >& dstFrames)
{
    dstFrames.resize(trajectoryFrames->frames.size());
    for(size_t frameIndex = 0; frameIndex < dstFrames.size(); frameIndex++)
    {
        const Ptr<RgbdFrame> srcFrame = trajectoryFrames->frames[frameIndex];
        if(std::find(frameIndices.begin(), frameIndices.end(), frameIndex) != frameIndices.end())
        {
            // clone data for used frames because we can modify them
            dstFrames[frameIndex] = new RgbdFrame(srcFrame->image.clone(), srcFrame->depth.clone(),
                                         refineObjectMask(trajectoryFrames->objectMasks[frameIndex]), srcFrame->normals.clone(),
                                         srcFrame->ID);
        }
        else
        {
            dstFrames[frameIndex] = new RgbdFrame(srcFrame->image, srcFrame->depth,
                                         refineObjectMask(trajectoryFrames->objectMasks[frameIndex]), srcFrame->normals,
                                         srcFrame->ID);
        }
    }
}




ModelCapturer::ModelCapturer()
{
  isLoopClosed = false;

  //TODO: move up
  cameraMatrix = (Mat_<float>(3, 3) << 525.0,   0.0, 319.5,
                                         0.0, 525.0, 239.5,
                                         0.0,   0.0,   1.0);
  onlineCaptureServer.set("cameraMatrix", cameraMatrix);

  //TODO: move up
  onlineCaptureServer.set("minTranslationDiff", 0.15f);
  onlineCaptureServer.set("minRotationDiff", 15.0f);
  onlineCaptureServer.initialize(Size(640, 480));
}

void ModelCapturer::addRGBDFrame(const cv::Mat &bgrImage, const cv::Mat &depth)
{
  if (isLoopClosed)
    return;

  std::cout << "adding the frame #" << allBgrImages.size() << std::endl;

  CV_Assert(bgrImage.type() == CV_8UC3);
  CV_Assert(depth.type() == CV_32FC1);
  CV_Assert(bgrImage.size() == depth.size());

  static int frameID = 0;
  cout << "pushing frame..." << std::flush;
  Ptr<OnlineCaptureServer::FramePushOutput> odometry_frame = onlineCaptureServer.push(bgrImage, depth, frameID);
  cout << " done." << endl;
  ++frameID;

  if (odometry_frame->frameState == TrajectoryFrames::KEYFRAME)
  {
    //TODO: move up
    publishOdometry(odometry_frame->pose, "RGBDOdometry");
  }

  allBgrImages.push_back(bgrImage);
#ifdef SAVE_COLLECTED_DATA
  allDepths.push_back(depth);
#endif

  isLoopClosed = onlineCaptureServer.isLoopClosedFunc();
  if (isLoopClosed)
  {
    createModel();
  }
}

void ModelCapturer::createModel()
{
  CV_Assert(!allBgrImages[0].empty());

#ifdef SAVE_COLLECTED_DATA
  CV_Assert(allBgrImages.size() == allDepths.size());
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





  cv::Ptr<TrajectoryFrames> keyframesData = onlineCaptureServer.finalize();


  const float voxelSize = 0.005f;

  cout << "Frame-to-frame odometry result" << endl;
  ModelReconstructor::genModel(keyframesData->frames, keyframesData->poses, cameraMatrix)->show(voxelSize);

  vector<Mat> refinedPosesSE3;
  vector<int> frameIndices;
  refineGraphSE3(keyframesData->poses, keyframesData->keyframePosesLinks, refinedPosesSE3, frameIndices);

  cout << "Result of the loop closure" << endl;
  ModelReconstructor::genModel(keyframesData->frames, refinedPosesSE3, cameraMatrix, frameIndices)->show(voxelSize);
  publishOdometry(refinedPosesSE3, "LoopClosure");

  // fill posesLinks with empty Rt because we want that they will be recomputed
  vector<PosesLink> keyframePosesLinks;
  preparePosesLinksWithoutRt(keyframesData->keyframePosesLinks, keyframePosesLinks);

  vector<Mat> refinedPosesSE3RgbdICP;
  const float pointsPart = 0.05f;
  refineGraphSE3RgbdICP(keyframesData->frames, refinedPosesSE3,
                        keyframePosesLinks, cameraMatrix, pointsPart, refinedPosesSE3RgbdICP, frameIndices);

  cout << "Result of RgbdICP for camera poses" << endl;
  ModelReconstructor::genModel(keyframesData->frames, refinedPosesSE3RgbdICP, cameraMatrix, frameIndices)->show(/*voxelSize*/);
  publishOdometry(refinedPosesSE3RgbdICP, "RgbdICP");

  vector<Ptr<RgbdFrame> > objectFrames; // with mask for object points only,
                                        // they will modified while refining the object points
  prepareFramesForModelRefinement(keyframesData, frameIndices, objectFrames);

  vector<Mat> refinedSE3ICPSE3ModelPoses;
  refineGraphSE3RgbdICPModel(objectFrames, refinedPosesSE3RgbdICP,
                             keyframePosesLinks, cameraMatrix, refinedSE3ICPSE3ModelPoses, frameIndices);

  cv::Ptr<ObjectModel> model = ModelReconstructor::genModel(objectFrames, refinedSE3ICPSE3ModelPoses, cameraMatrix, frameIndices);

  cout << "Result of RgbdICP  for camera poses and model points refinement" << endl;
  model->show();
  publishOdometry(refinedSE3ICPSE3ModelPoses, "RgbdICP_landmarks");
}
