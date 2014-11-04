#ifndef CCNY_RGBD_SPARSE_TRACKER_AM_H
#define CCNY_RGBD_SPARSE_TRACKER_AM_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl_ros/point_cloud.h>

#include <pcl/registration/icp.h>

#include <pcl/io/io.h>
//#include <pcl/io/impl/io.hpp>

#include "ccny_rgbd/features/canny_detector.h"
#include "ccny_rgbd/features/orb_detector.h"
#include "ccny_rgbd/registration/icp_kd.h"
#include "ccny_rgbd/registration/feature_history.h"
#include "ccny_rgbd/util.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

namespace ccny_rgbd
{

const std::string sub_topic_          = "/camera/depth_registered/points";
const std::string pub_orb_model_topic_    = "/model/orb";
const std::string pub_canny_model_topic_    = "/model/canny";
const std::string pub_orb_features_topic_   = "/features/orb";
const std::string pub_canny_features_topic_ = "/features/canny";
const std::string pub_pose_topic_     = "/pose_est";

class SparseTrackerAM
{
  public:

    SparseTrackerAM(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~SparseTrackerAM();

    void pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_msg);

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_cloud_subscriber_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher orb_features_pub_;
    ros::Publisher canny_features_pub_;
    ros::Publisher orb_model_pub_;
    ros::Publisher canny_model_pub_;
    ros::Publisher pose_pub_;

    ros::Publisher marker_pub_;

    // **** parameters 

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

    // **** variables

    boost::mutex mutex_;

    int  frame_count_;
    bool initialized_;

    CannyDetector  canny_detector_;
    OrbDetector    orb_detector_;

    //FeatureHistory<PointOrb>   orb_history_;
    //FeatureHistory<PointCanny> canny_history_;

    tf::Transform b2c_; // Base frame to Camera frame
    tf::Transform f2b_; // Fixed frame to Base frame

    tf::Transform pf2b_[20]; // Fixed frame to Base frame

    //pcl::IterativeClosestPoint<PointOrb, PointOrb> orb_reg_;
    ccny_rgbd::ICPKd<PointOrb,   PointOrb>   orb_reg_;
    ccny_rgbd::ICPKd<PointCanny, PointCanny> canny_reg_;

    //void align
    //pcl::VoxelGrid<PointOrb> vgf_model_;

    ros::Time last_icp_time_;

    PointCloudOrb::Ptr model_;

    // **** alpha beta

    bool use_alpha_beta_;
    double alpha_;
    double beta_;

    ros::Time prev_time_;

    btVector3 v_linear_;
    btVector3 v_angular_;

    double v_x_, v_y_, v_z_;
    double v_roll_, v_pitch_, v_yaw_;
    btVector3 vel_;

    // **** private functions

    bool getBaseToCameraTf(const PointCloudT::ConstPtr cloud_in_ptr);
    void initParams();

    bool extractOrbFeatures(const PointCloudT::ConstPtr cloud_in_ptr,
                                  PointCloudOrb::Ptr& features_ptr);

    bool extractCannyFeatures(const PointCloudT::ConstPtr cloud_in_ptr,
                                    PointCloudCanny::Ptr& features_ptr);

    bool OrbICP(PointCloudOrb::Ptr& orb_features_ptr);
    bool CannyICP(PointCloudCanny::Ptr& canny_features_ptr);

    //void buildModel(const PointCloudT::ConstPtr cloud_in_ptr,
    //                      PointCloudOrb::Ptr model_ptr);

    void broadcastTF(const PointCloudT::ConstPtr cloud_in_ptr);

    tf::Transform tfFromEigen(Eigen::Matrix4f trans)
    {
     btMatrix3x3 btm;
     btm.setValue(trans(0,0),trans(0,1),trans(0,2),
                trans(1,0),trans(1,1),trans(1,2),
                trans(2,0),trans(2,1),trans(2,2));
     btTransform ret;
     ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
     ret.setBasis(btm);
     return ret;
    }

    Eigen::Matrix4f eigenFromTf(const tf::Transform& tf)
    {
      Eigen::Matrix4f out_mat;

       double mv[12];
       tf.getBasis().getOpenGLSubMatrix(mv);

       tf::Vector3 origin = tf.getOrigin();

       out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
       out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
       out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

       out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
       out_mat (0, 3) = origin.x ();
       out_mat (1, 3) = origin.y ();
       out_mat (2, 3) = origin.z ();

      return out_mat;
    }

    inline void getXYZRPY(const tf::Transform& t,
                                double& x,    double& y,     double& z,
                                double& roll, double& pitch, double& yaw)
    {
      x = t.getOrigin().getX();
      y = t.getOrigin().getY();
      z = t.getOrigin().getZ();

      btMatrix3x3 m(t.getRotation());
      m.getRPY(roll, pitch, yaw);
    }

    inline void fixAngleD(double& a)
    {
      while (a  >   M_PI) a -= 2.0*M_PI;
      while (a <= - M_PI) a += 2.0*M_PI;
    }

    inline float getUrand()
    {
      return ((float)rand() / (float)RAND_MAX)*2.0 - 1.0;
    }

    inline float getNrand()
    {
      float U = getUrand();
      float V = getUrand();

      float Z = sqrt(-2.0 * log(U)) * cos(2.0 * M_PI * V);

      //printf (" ================= %f ================= \n", Z);
      return Z;
    }
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_SPARSE_TRACKER_H
