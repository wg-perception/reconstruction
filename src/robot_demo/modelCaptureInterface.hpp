#ifndef MODEL_CAPTURE_INTERFACE_HPP
#define MODEL_CAPTURE_INTERFACE_HPP

#include <opencv2/core/core.hpp>
//TODO: use splitted code
#include "../../../opencv_candidate/src/rgbd/samples/model_capture/model_capture.hpp"

class ModelCapturer
{
  public:
    ModelCapturer();
    void addRGBDFrame(const cv::Mat &bgrImage, const cv::Mat &depth);
    void createModel();
  private:
    std::vector<cv::Mat> allBgrImages, allDepths;
    bool isLoopClosed;
    OnlineCaptureServer onlineCaptureServer;
    cv::Mat cameraMatrix;
};

#endif
