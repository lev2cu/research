#ifndef CCNY_RGBD_SPARSE_TRACKER_H
#define CCNY_RGBD_SPARSE_TRACKER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <ccny_gicp/gicp.h>
#include <ccny_gicp/types.h>
#include <ccny_gicp/gicp_align.h>

#include "ccny_rgbd/util.h"

#include "ccny_rgbd/PublishFrame.h"
#include "ccny_rgbd/PublishAllFrames.h"

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/features/canny_detector.h"
#include "ccny_rgbd/features/orb_detector.h"
#include "ccny_rgbd/features/surf_detector.h"
#include "ccny_rgbd/features/gft_detector.h"
#include "ccny_rgbd/registration/icp_kd.h"
#include "ccny_rgbd/registration/feature_history.h"
#include "ccny_rgbd/registration/loop_solver.h"


namespace ccny_rgbd
{

const std::string sub_topic_           = "/camera/depth_registered/points";
const std::string pub_features_topic_  = "/features";
const std::string pub_keyframes_topic_ = "/keyframes";

class SparseTracker
{
  typedef std::vector<RGBDFrame, Eigen::aligned_allocator<RGBDFrame> > KeyframeVector;

  public:

    SparseTracker(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~SparseTracker();

    void pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_msg);

    bool publishAllFramesSrvCallback(ccny_rgbd::PublishAllFrames::Request& request,
                                     ccny_rgbd::PublishAllFrames::Response& response);

    bool publishFrameSrvCallback(ccny_rgbd::PublishFrame::Request& request,
                                 ccny_rgbd::PublishFrame::Response& response);

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_cloud_sub_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher features_pub_;
    ros::Publisher keyframes_pub_;

    ros::ServiceServer pub_frames_service_;
    ros::ServiceServer pub_frame_service_;

    // **** parameters 

    std::string fixed_frame_;
    std::string base_frame_;

    int detector_type_; // Orb / Surf / Canny

    int reg_type_; // PCL-kd / GICP-nocov

  /*
    bool   use_vgf_features_filter_;
    double vgf_features_res_;
    bool   use_vgf_model_filter_;
    double vgf_model_res_;
  */
    double icp_tf_epsilon_;
    double icp_max_corresp_dist_;
    int    icp_max_iterations_;
    int    min_features_;

    double kf_dist_eps_;
    double kf_angle_eps_;

    // **** variables

    boost::mutex mutex_;

    int  frame_count_;
    bool initialized_;

    FeatureDetector * feature_detector_;

    FeatureHistory<PointFeature>   feature_history_;

    KeyframeVector keyframes_;

    tf::Transform b2c_; // Base (moving) frame to Camera-optical frame
    tf::Transform f2b_; // Fixed frame to Base (moving) frame

    ccny_rgbd::ICPKd<PointFeature, PointFeature> reg_;

    ros::Time last_icp_time_;

    tf::Transform     last_keyframe_f2b_;
    PointCloudFeature last_keyframe_features_;

    ccny_gicp::GICPAlign gicp_reg_;

    LoopSolver * loop_solver_;
  
    // **** private functions

    bool getBaseToCameraTf(const PointCloudT::ConstPtr cloud_in_ptr);
    void initParams();

    bool ICP (PointCloudFeature::Ptr& features_ptr, tf::Transform& corr);
    bool GICP(PointCloudFeature::Ptr& features_ptr, tf::Transform& corr);

    void broadcastTF(const PointCloudT::ConstPtr cloud_in_ptr);

    void publishFrame(int i);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_SPARSE_TRACKER_H
