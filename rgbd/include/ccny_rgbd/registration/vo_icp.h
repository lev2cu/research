#ifndef CCNY_RGBD_VO_ICP_H
#define CCNY_RGBD_VO_ICP_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

//#include <ccny_gicp/gicp.h>
//#include <ccny_gicp/types.h>
//#include <ccny_gicp/gicp_align.h>

#include "ccny_rgbd/util.h"

#include "ccny_rgbd/PublishFrame.h"
#include "ccny_rgbd/PublishAllFrames.h"

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/features/klt_detector.h"
#include "ccny_rgbd/features/orb_detector.h"
#include "ccny_rgbd/features/surf_detector.h"
#include "ccny_rgbd/features/gft_detector.h"

#include "ccny_rgbd/structures/rgbd_frame2.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/registration/icp_kd.h"
#include "ccny_rgbd/registration/feature_history.h"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace ccny_rgbd
{

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

//const std::string sub_topic_           = "/camera/depth_registered/points";
const std::string pub_features_topic_  = "/features";
const std::string pub_keyframes_topic_ = "/keyframes";
const std::string pub_pose_topic_      = "/rgbd_vo";
const std::string sub_vel_topic_       = "/vel";

class VOICP
{
  typedef Eigen::aligned_allocator<RGBDKeyframe> KeyframeAllocator;
  typedef std::vector<RGBDKeyframe, KeyframeAllocator> KeyframeVector;

  public:

    VOICP(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~VOICP();

    //void pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_msg);

    bool publishAllFramesSrvCallback(ccny_rgbd::PublishAllFrames::Request& request,
                                     ccny_rgbd::PublishAllFrames::Response& response);

    bool publishFrameSrvCallback(ccny_rgbd::PublishFrame::Request& request,
                                 ccny_rgbd::PublishFrame::Response& response);

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;
    ros::Subscriber info_sub_;
    ros::Subscriber vel_sub_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher features_pub_;
    ros::Publisher keyframes_pub_;
    ros::Publisher pose_pub_;

    ros::ServiceServer pub_frames_service_;
    ros::ServiceServer pub_frame_service_;

    ros::NodeHandlePtr rgb_nh_;

    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<image_transport::ImageTransport> depth_it_;
    
    // Subscriptions
    
    typedef image_transport::SubscriberFilter ImageSubFilter;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubFilter;

    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;

    // **** parameters 

    std::string fixed_frame_;
    std::string base_frame_;

    bool publish_keyframes_;

    std::string detector_type_; // Orb / Surf / GFT / Canny

    double icp_tf_epsilon_;
    double icp_max_corresp_dist_;
    int    icp_max_iterations_;
    int    min_features_;

    double kf_dist_eps_;
    double kf_angle_eps_;

    // **** variables

    boost::mutex mutex_;

    geometry_msgs::Twist twist_;

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
  
    // **** private functions

    void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg);

    void velCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg);

    void initParams();
    bool ICP (PointCloudFeature::Ptr& features_ptr, tf::Transform& corr);
    bool getBaseToCameraTf(const std_msgs::Header& header);
    void broadcastTF(const std_msgs::Header& header);
    void publishKeyframe(int i);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_VO_ICP_H
