#include "ccny_rgbd/structures/rgbd_frame2.h"

namespace ccny_rgbd
{

RGBDFrame2::RGBDFrame2(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg,
                       const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // TODO: Share vs copy?
  cv_ptr_rgb_   = cv_bridge::toCvCopy(rgb_msg);
  cv_ptr_depth_ = cv_bridge::toCvCopy(depth_msg);

  // create camera model
  model_.fromCameraInfo(info_msg);
}

void RGBDFrame2::constructFeatureCloud(float max_range)
{
  // Use correct principal point from calibration
  float center_x = model_.cx();
  float center_y = model_.cy();

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / model_.fx();
  float constant_y = 1.0 / model_.fy();

  // iterate over all keypoints and project to 3D
  // TODO: resize before
  for (unsigned int i = 0; i < keypoints.size(); ++i)
  {
    // calculate pixel coordinates
    int u = (int)(keypoints[i].pt.x);
    int v = (int)(keypoints[i].pt.y);

    uint16_t z_raw = cv_ptr_depth_->image.at<uint16_t>(v, u);

    // skip invalid data
    if (z_raw == 0) continue;

    float z = z_raw * 0.001; //convert to meters
    if (z <= max_range)
    {
      PointFeature p;
      p.x = (keypoints[i].pt.x - center_x) * constant_x * z;
      p.y = (keypoints[i].pt.y - center_y) * constant_y * z;
      p.z = z;

      features.points.push_back(p);
    }
  }

  features.header = cv_ptr_rgb_->header;
  features.height = 1;
  features.width = features.points.size();
  features.is_dense = true;
}

} // namespace ccny_rgbd
