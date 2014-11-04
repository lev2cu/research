#ifndef CCNY_RGBD_RESIZER_H
#define CCNY_RGBD_RESIZER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

const std::string sub_topic_       = "/camera/depth_registered/points";
const std::string pub_vga_topic_   = "/filtered/vga";
const std::string pub_qvga_topic_  = "/filtered/qvga";
const std::string pub_qqvga_topic_ = "/filtered/qqvga";

class Resizer
{
  public:

    Resizer(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~Resizer();

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_cloud_subscriber_;

    ros::Publisher vga_pub_;
    ros::Publisher qvga_pub_;
    ros::Publisher qqvga_pub_;

    // **** parameters

    bool publish_qvga_;
    bool publish_qqvga_;

    unsigned int width ;
    unsigned int height;

    unsigned int qwidth ;
    unsigned int qheight;

    unsigned int qqwidth ;
    unsigned int qqheight;


    void pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr);
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RESIZER_H
