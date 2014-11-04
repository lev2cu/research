#include "ccny_rgbd/registration/vo_icp.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "VisualOdometryICP");
  
  printf("! %d.%d !\n", ROS_VERSION_MAJOR, ROS_VERSION_MINOR);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::VOICP vo(nh, nh_private);
  ros::spin();
  return 0;
}
