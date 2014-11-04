#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

RGBDKeyframe::RGBDKeyframe(const RGBDFrame2& frame):
  RGBDFrame2(frame),
  max_data_range_(4.5)
{

}

void RGBDKeyframe::constructDataCloud()
{
  // Use correct principal point from calibration
  float center_x = model_.cx();
  float center_y = model_.cy();

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / model_.fx();
  float constant_y = 1.0 / model_.fy();

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  data.points.resize(cv_ptr_rgb_->image.rows * cv_ptr_rgb_->image.cols);
  for (int v = 0; v < cv_ptr_rgb_->image.rows; ++v)
  for (int u = 0; u < cv_ptr_rgb_->image.cols; ++u)
  {
    unsigned int index = v * cv_ptr_rgb_->image.cols + u;

    uint16_t z_raw = cv_ptr_depth_->image.at<uint16_t>(v, u);
    float z = z_raw * 0.001; //convert to meters

    PointT p;

    // check for invalid measurements
    if (z != 0 &&  z <= max_data_range_)
    {
      // fill in XYZ
      p.x = (u - center_x) * constant_x * z;
      p.y = (v - center_y) * constant_y * z;
      p.z = z;
    }
    else
      p.x = p.y = p.z = bad_point;

    cv::Vec3b& bgr = cv_ptr_rgb_->image.at<cv::Vec3b>(v,u);
    uint32_t color = (bgr[2] << 16) + (bgr[1] << 8) + bgr[0];
    p.rgb = *reinterpret_cast<float*>(&color);

    data.points[index] = p;
  }

  data.header = cv_ptr_rgb_->header;
  data.height = cv_ptr_rgb_->image.rows;
  data.width  = cv_ptr_rgb_->image.cols;
  data.is_dense = false;
}


} // namespace
