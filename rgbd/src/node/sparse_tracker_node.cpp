#include "ccny_rgbd/registration/sparse_tracker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SparseTracker");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::SparseTracker st(nh, nh_private);
  ros::spin();
  return 0;
}
