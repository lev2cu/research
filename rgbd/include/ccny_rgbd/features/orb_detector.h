#ifndef CCNY_RGBD_ORB_DETECTOR_H
#define CCNY_RGBD_ORB_DETECTOR_H

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

class OrbDetector: public FeatureDetector
{
  public:

    OrbDetector();
    ~OrbDetector();

    void setThreshold(int threshold);
    int getThreshold() const;
    void setNFeatures(unsigned int n_features);
    unsigned int getNFeatures() const;

    void findFeatures(RGBDFrame2& frame, const cv::Mat * input_img);

  private:

    unsigned int n_features_;
    double edge_threshold_;

    //cv::ORB::CommonParams orb_params_;
    
    cv::OrbDescriptorExtractor orb_descriptor_;
    cv::OrbFeatureDetector * orb_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_ORB_DETECTOR_H
