#include "ccny_rgbd/nodelet/resizer_nodelet.h"

typedef ccny_rgbd::ResizerNodelet ResizerNodelet;

PLUGINLIB_DECLARE_CLASS (ccny_rgbd, ResizerNodelet, ResizerNodelet, nodelet::Nodelet);

void ccny_rgbd::ResizerNodelet::onInit ()
{
  NODELET_INFO("Initializing Resizer Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  r_ = new ccny_rgbd::Resizer(nh, nh_private);  
}
