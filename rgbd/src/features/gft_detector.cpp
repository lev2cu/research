#include "ccny_rgbd/features/gft_detector.h"

namespace ccny_rgbd
{

GftDetector::GftDetector():
  n_features_(200)
{
  gft_detector_ = new cv::GoodFeaturesToTrackDetector(n_features_);
}

GftDetector::~GftDetector()
{
  delete gft_detector_;
}

void GftDetector::findFeatures(RGBDFrame2& frame, const cv::Mat * input_img)
{
  gft_detector_->detect(*input_img, frame.keypoints);
  //frame.keypoints_computed_ = true;

}

} //namespace
