#ifndef CCNY_RGBD_SPARSE_TRACKER_F_H
#define CCNY_RGBD_SPARSE_TRACKER_F_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <pcl/registration/icp.h>

#include <pcl/io/io.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <ccny_gicp/gicp.h>
#include <ccny_gicp/types.h>
#include <ccny_gicp/gicp_align.h>
#include <pcl/registration/ia_ransac.h>

#include "ccny_rgbd/Loop.h"
#include "ccny_rgbd/util.h"

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/features/canny_detector.h"
#include "ccny_rgbd/features/orb_detector.h"
#include "ccny_rgbd/features/surf_detector.h"

#include "ccny_rgbd/registration/icp_kd.h"
#include "ccny_rgbd/registration/feature_history.h"

#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/structures/edge.h"

#include <ccny_toro/treeoptimizer3.hh>
#include <ccny_toro/posegraph3.hh>
#include <ccny_toro/posegraph.hh>

namespace ccny_rgbd
{

const std::string sub_topic_          = "/camera/depth_registered/points";
const std::string pub_model_topic_      = "/model";
const std::string pub_features_topic_   = "/features";
const std::string pub_pose_topic_     = "/pose_est";

const std::string pub_poses_topic_     = "/poses";
const std::string pub_edges_topic_     = "/edges";
const std::string pub_frames_topic_     = "/frames";
const std::string pub_all_frames_topic_     = "/all_frames";

class SparseTrackerF
{
  public:

    SparseTrackerF(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~SparseTrackerF();

    void pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_msg);

    bool loopSrvCallback(ccny_rgbd::Loop::Request& request,
                         ccny_rgbd::Loop::Response& response);

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_cloud_subscriber_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher features_pub_;
    ros::Publisher model_pub_;

    ros::Publisher pose_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher edges_pub_;
    ros::Publisher frames_pub_;
    ros::Publisher all_frames_pub_;

    ros::ServiceServer loop_service_;

    // **** parameters 

    //bool use_orb_;

    int detector_type_;

    double kf_dist_eps_ ;
    double kf_angle_eps_;

    std::string fixed_frame_;
    std::string base_frame_;

    bool use_canny_features;
    bool use_orb_features;

    bool   use_vgf_features_filter_;
    double vgf_features_res_;

    bool   use_vgf_model_filter_;
    double vgf_model_res_;

    double icp_tf_epsilon_;
    double icp_max_corresp_dist_;
    int    icp_max_iterations_;
    int    min_features_;

    double ransac_matching_distance_;
    double ransac_eps_reproj_;
    double ransac_inlier_threshold_;
    double ransac_max_iterations_;

    // **** variables

    boost::mutex mutex_;

    tf::Transform last_f2b_; // TODO - is this needed? just look up last keyframe

    std::vector<RGBDFrame, Eigen::aligned_allocator<RGBDFrame> > keyframes_;
    std::vector<Edge> edges_;

    int  frame_count_;
    bool initialized_;

    FeatureDetector * feature_detector_;

    FeatureHistory<PointFeature>  feature_history_;

    tf::Transform b2c_; // Base frame to Camera frame
    tf::Transform f2b_; // Fixed frame to Base frame

    ccny_rgbd::ICPKd<PointFeature, PointFeature> reg_;

    ros::Time last_icp_time_;

    PointCloudFeature::Ptr model_;

    // **** private functions

    bool getBaseToCameraTf(const PointCloudT::ConstPtr cloud_in_ptr);
    void initParams();
    void computeAllEdges();
    void computeLastEdge();

    void computeEdgeAssociationsRansac();

    void loopClose();

    bool extractFeatures(RGBDFrame& frame);

    bool ICP(PointCloudFeature::Ptr& features_ptr);

    void broadcastTF(const PointCloudT::ConstPtr cloud_in_ptr);

    void publishFrames();

    inline void fixAngleD(double& a)
    {
      while (a  >  M_PI) a -= 2.0*M_PI;
      while (a <= -M_PI) a += 2.0*M_PI;
    }
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_SPARSE_TRACKER_H
