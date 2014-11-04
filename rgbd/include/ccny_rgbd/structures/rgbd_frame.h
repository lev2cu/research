#ifndef CCNY_RGBD_RGBD_FRAME_H
#define CCNY_RGBD_RGBD_FRAME_H

#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/image_encodings.h>
#include <pcl/ros/conversions.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "ccny_rgbd/util.h"
//#include "ccny_rgbd/features/orb_detector.h"
//#include "ccny_rgbd/features/surf_detector.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

#include <pcl/kdtree/kdtree.h>

#include <ccny_gicp/gicp.h>
#include <ccny_gicp/types.h>
#include <ccny_gicp/gicp_align.h>

#include "ccny_rgbd/registration/icp_kd.h"

namespace ccny_rgbd
{

class RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDFrame();
    
    int id;

    PointCloudT   data;
    PointCloudT   data_downsampled;

    PointCloudFeature rgb_features;
    PointCloudFeature depth_features;

    std::vector<cv::KeyPoint> keypoints; //without removing empty z
    cv::Mat descriptors;

    bool keypoints_computed_;
    bool descriptors_computed_;

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    tf::Transform pose;

    double path_length_linear;
    double path_length_angular;

    void computeRGBImage();
    void computeDepthImage();
    void computeUDepthImage();
    void computeFeatureBBX();

    void simpleFilter();

    cv::Mat * getRGBImage();
    cv::Mat * getDepthImage();
    cv::Mat * getUDepthImage();

    inline int getOrbRGBFeaturesCount()   const { return rgb_features.points.size(); }
    inline int getOrbDepthFeaturesCount() const { return depth_features.points.size(); }

    static bool poseDistOverlap       (const RGBDFrame& a, const RGBDFrame& b);
    static bool totalPathlengthOverlap(const RGBDFrame& a, const RGBDFrame& b);
    static bool bbxOverlap            (const RGBDFrame& a, const RGBDFrame& b);

    static bool ransacMatchingOverlap(
      RGBDFrame& frame_src, RGBDFrame& frame_dst, 
      tf::Transform& transform, float matching_distance, 
      float eps_reproj, float inlier_threshold,
      PointCloudT::Ptr cloud_src = boost::shared_ptr<PointCloudT>(new PointCloudT()), 
      PointCloudT::Ptr cloud_dst = boost::shared_ptr<PointCloudT>(new PointCloudT()));

    static bool computeTfGICP(RGBDFrame& frame_src, RGBDFrame& frame_dst, 
                              tf::Transform& transform);

    static void constructClouds(
      const std::vector<cv::DMatch>& inlier_matches,
      const RGBDFrame& frame_src, 
      const RGBDFrame& frame_dst, 
      PointCloudT::Ptr& cloud_src,
      PointCloudT::Ptr& cloud_dst);

/*
    static bool get4RandomIndices(std::vector<int>& random_indices,
                                  unsigned int size);
*/

  private:

    bool bbx_computed_;
    bool rgb_image_computed_;
    bool depth_image_computed_;
    bool u_depth_image_computed_;

    cv::Mat depth_img_;
    cv::Mat u_depth_img_;
    cv::Mat rgb_img_;
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_FRAME_H
