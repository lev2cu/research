#ifndef CCNY_RGBD_FEATURE_DETECTOR_H
#define CCNY_RGBD_FEATURE_DETECTOR_H

#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl/ros/conversions.h>

#include "ccny_rgbd/structures/rgbd_frame2.h"
#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

class RGBDFrame;

class FeatureDetector
{
  public:

    FeatureDetector();
    virtual ~FeatureDetector();

    void findFeatures(RGBDFrame2& frame);
    virtual void findFeatures(RGBDFrame2& frame, const cv::Mat * input_img) = 0;

    inline double findFeaturesTimed(RGBDFrame2& frame)
    {
      struct timeval start, end;
      gettimeofday(&start, NULL);
      findFeatures(frame);
      gettimeofday(&end, NULL);
      return ccny_rgbd::msDuration(start, end);
    }

    inline void setSmooth(int smooth)
    {
      smooth_ = smooth;
    }

    inline int getSmooth() const
    {
      return smooth_;
    }

    /*
    inline void setWindow(int window)
    {
      window_ = window;
    }

    inline int getWindow() const
    {
      return window_;
    }*/

    
    inline void setMaxRange(float max_range)
    {
      max_range_ = max_range;
    }

    inline float getMaxRange() const
    {
      return max_range_;
    } 

  /*
    inline void setComputeDescriptors(bool compute_descriptors)
    {
      compute_descriptors_ = compute_descriptors;
    }

    inline bool getComputeDescriptors() const
    {
      return compute_descriptors_;
    }
*/
  protected:

    int smooth_;
    float max_range_;
    //bool compute_descriptors_;

    /*
    bool testPixel(const cv::Mat& img,
                   const PointCloudT& cloud_in,
                   double z, int n_i, int n_j,
                   int& corr_index);  
  
    void constructFeaturePointCloud(RGBDFrame2& frame, const cv::Mat * input_img);  
    */
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_FEATURE_DETECTOR_H
