#ifndef CCNY_RGBD_RGBD_KEYFRAME_H
#define CCNY_RGBD_RGBD_KEYFRAME_H

#include "ccny_rgbd/structures/rgbd_frame2.h"

namespace ccny_rgbd
{

class RGBDKeyframe: public RGBDFrame2
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDKeyframe(const RGBDFrame2& frame);
  
    tf::Transform pose;
    PointCloudT   data;
    //PointCloudT data_downsampled;

    double path_length_linear;
    double path_length_angular;

    void constructDataCloud();

  private:

    float max_data_range_;    // maximum z range for dense data
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_KEYFRAME_H
