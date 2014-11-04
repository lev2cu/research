#include "ccny_rgbd/registration/dense_tracker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DenseTracker");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::DenseTracker dt(nh, nh_private);
  printf("!\n");
  ros::spin();
  return 0;
}
