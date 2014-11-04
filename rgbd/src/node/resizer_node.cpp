#include "ccny_rgbd/filter/resizer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Resizer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::Resizer r(nh, nh_private);
  ros::spin();
  return 0;
}
