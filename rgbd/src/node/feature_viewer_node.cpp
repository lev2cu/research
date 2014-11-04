#include "ccny_rgbd/features/feature_viewer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FeatureViewer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::FeatureViewer fv(nh, nh_private);
  ros::spin();
  return 0;
}
