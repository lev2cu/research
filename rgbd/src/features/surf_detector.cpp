#include "ccny_rgbd/features/surf_detector.h"

namespace ccny_rgbd
{

SurfDetector::SurfDetector()
{
  surf_detector_ = new cv::SurfFeatureDetector();
}

SurfDetector::~SurfDetector()
{
  delete surf_detector_;
}

void SurfDetector::findFeatures(RGBDFrame2& frame, const cv::Mat * input_img)
{
  surf_detector_->detect(*input_img, frame.keypoints);

  /*
  if(compute_descriptors_)
  {
    surf_descriptor_.compute(*input_img, frame.keypoints, frame.descriptors);
    frame.descriptors_computed_ = true;
  }
  */
}

} //namespace
