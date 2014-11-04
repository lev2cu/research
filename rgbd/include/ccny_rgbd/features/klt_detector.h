#ifndef CCNY_RGBD_KLT_DETECTOR_H
#define CCNY_RGBD_KLT_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

class KltDetector: public FeatureDetector
{
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:

    KltDetector();
    ~KltDetector();

    void findFeatures(RGBDFrame2& frame, const cv::Mat * input_img);

  private:

    bool initialized_;
    bool have_points_;

    int n_features_;
    float reseed_threshold_;
    int win_size_;

    cv::Mat prev_input_img_;
    //std::vector<cv::KeyPoint> prev_keypoints_;
    //std::vector<cv::Point2f> prev_points_;

    CvPoint2D32f * pointsA_;
    CvPoint2D32f * points_good_;

    IplImage * pyrA;
    IplImage * pyrB;

    //cv::GoodFeaturesToTrackDetector::Params gft_params_;
    cv::GoodFeaturesToTrackDetector * gft_detector_;


};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KLT_DETECTOR_H

