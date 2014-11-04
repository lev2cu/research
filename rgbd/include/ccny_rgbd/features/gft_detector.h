#ifndef CCNY_RGBD_GFT_DETECTOR_H
#define CCNY_RGBD_GFT_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

class GftDetector: public FeatureDetector
{
  public:

    GftDetector();
    ~GftDetector();

    void findFeatures(RGBDFrame2& frame, const cv::Mat * input_img);

  private:

    int n_features_;

    //cv::GoodFeaturesToTrackDetector::Params gft_params_;

    cv::GoodFeaturesToTrackDetector * gft_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_GFT_DETECTOR_H
