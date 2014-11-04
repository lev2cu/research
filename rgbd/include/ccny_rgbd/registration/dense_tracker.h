#ifndef CCNY_RGBD_DENSE_TRACKER_H
#define CCNY_RGBD_DENSE_TRACKER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ccny_gicp/gicp.h>
#include <ccny_gicp/types.h>
#include <ccny_gicp/gicp_align.h>

#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

const std::string sub_topic_    = "/camera/depth_registered/points";
const std::string scene_topic_  = "/scene";

class DenseTracker
{
  public:
    
    typedef pcl::PointXYZRGB        PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudTPtr;
    typedef pcl::octree::OctreePointCloudStorage<PointGICP> Octree;
    typedef pcl::KdTreeFLANN<PointGICP> KdTree;

    DenseTracker(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~DenseTracker();

    void pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr);

  private:
    
    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_cloud_subscriber_;
    ros::Publisher scene_pub_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // **** parameters 

    std::string world_frame_;
    std::string odom_frame_;

    bool use_vgf_data_filter_;
    double vgf_data_res_;
    double vgf_data_range_;

    double octree_resolution_;

    double gicp_epsilon_;
    int nn_count_;

    bool use_additive_model_;

    // **** variables

    unsigned int cloud_count_;
    tf::Transform world_to_odom_;

    PointCloudGICP::Ptr model_;
    Octree* octree_;
    ccny_gicp::GICPAlign reg_;

    ccny_gicp::GICPPointSetKd     gicp_data_;

    ccny_gicp::GICPPointSetOctree * gicp_model_octree_;
    ccny_gicp::GICPPointSetKd     * gicp_model_kdtree_;

    // **** private functions

    void initParams();
    bool getOdomToCameraTf(const PointCloudT::ConstPtr& cloud_in_ptr, tf::Transform& o2k);
    void filterCloud(const PointCloudT::ConstPtr& cloud_in_ptr,
                     const PointCloudT::Ptr& data_ptr);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_DENSE_TRACKER_H
