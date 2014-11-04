#ifndef CCNY_RGBD_RGBD_FRAME2_H
#define CCNY_RGBD_RGBD_FRAME2_H

#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

class RGBDFrame2
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDFrame2(const sensor_msgs::ImageConstPtr& rgb_msg,
               const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    int id;

    PointCloudFeature features;
      
    std::vector<cv::KeyPoint> keypoints; //without removing empty z

    cv::Mat * getRGBImage()   const { return &cv_ptr_rgb_->image; }
    cv::Mat * getDepthImage() const { return &cv_ptr_depth_->image; }

    void constructFeatureCloud(float max_range);

  protected:

    cv_bridge::CvImagePtr cv_ptr_rgb_;
    cv_bridge::CvImagePtr cv_ptr_depth_;

    image_geometry::PinholeCameraModel model_;
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_FRAME_H
