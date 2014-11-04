#include "ccny_rgbd/registration/sparse_tracker_am.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SparseTrackerAM");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::SparseTrackerAM st_am(nh, nh_private);
  ros::spin();
  return 0;
}
