#include "ccny_rgbd/registration/sparse_tracker_am.h"

namespace ccny_rgbd {

SparseTrackerAM::SparseTrackerAM(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  frame_count_(0),
  initialized_(false)
{
  ROS_INFO("Starting Sparse Tracker AM");

  srand(time(NULL));

  // **** init variables

  f2b_.setIdentity();

  for (int i = 0; i < 20; ++i)
    pf2b_[i].setIdentity();

  v_x_     = 0;
  v_y_     = 0;
  v_z_     = 0;
  v_roll_  = 0;
  v_pitch_ = 0;
  v_roll_  = 0;

  v_linear_  = btVector3(0.0, 0.0, 0.0);
  v_angular_ = btVector3(0.0, 0.0, 0.0);

  prev_time_ = ros::Time::now();

  // **** init parameters

  initParams();

  model_ = boost::make_shared<PointCloudOrb>();
  model_->header.frame_id = fixed_frame_;
 
  // **** publishers

  orb_features_pub_ = nh_.advertise<PointCloudT>(
    pub_orb_features_topic_, 1);
  canny_features_pub_ = nh_.advertise<PointCloudT>(
    pub_canny_features_topic_, 1);
  orb_model_pub_    = nh_.advertise<PointCloudT>(
   pub_orb_model_topic_, 1);
  canny_model_pub_    = nh_.advertise<PointCloudT>(
   pub_canny_model_topic_, 1);
  pose_pub_     = nh_.advertise<geometry_msgs::PoseStamped>(pub_pose_topic_, 1);

  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
   "visualization_markers", 1);

  // **** subscribers

  point_cloud_subscriber_ = nh_.subscribe<PointCloudT>(
    sub_topic_, 1, &SparseTrackerAM::pointCloudCallback, this);
}

SparseTrackerAM::~SparseTrackerAM()
{
  ROS_INFO("Destroying Feature Viewer"); 
}

void SparseTrackerAM::initParams()
{
  int icp_value_threshold;
  int icp_nr_neighbors;
  double max_value_diff;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/base_link";

  if (!nh_private_.getParam ("use_vgf_features_filter", use_vgf_features_filter_))
    use_vgf_features_filter_ = false;
  if (!nh_private_.getParam ("vgf_features_res", vgf_features_res_))
    vgf_features_res_ = 0.01;

  if (!nh_private_.getParam ("use_vgf_model_filter", use_vgf_model_filter_))
    use_vgf_model_filter_ = false;
  if (!nh_private_.getParam ("vgf_model_res", vgf_model_res_))
    vgf_model_res_ = 0.01;

  if (!nh_private_.getParam ("icp_tf_epsilon", icp_tf_epsilon_))
    icp_tf_epsilon_ = 0.000001;


  if (!nh_private_.getParam ("icp_nr_neighbors", icp_nr_neighbors))
    icp_nr_neighbors = 16;

  if (!nh_private_.getParam ("min_features", min_features_))
    min_features_ = 10;

  bool use_value_rejection;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;
  //int history_size;

  //if (!nh_private_.getParam ("history_size", history_size))
  //  history_size = 5;
  if (!nh_private_.getParam ("icp_max_iterations", icp_max_iterations_))
    icp_max_iterations_ = 30;
  if (!nh_private_.getParam ("icp_tf_epsilon", icp_tf_epsilon_))
    icp_tf_epsilon_ = 0.000001;
  if (!nh_private_.getParam ("use_value_rejection", use_value_rejection))
    use_value_rejection = false;
  if (!nh_private_.getParam ("use_ransac_rejection", use_ransac_rejection))
    use_ransac_rejection = false;
  if (!nh_private_.getParam ("icp_max_corresp_dist", icp_max_corresp_dist_))
    icp_max_corresp_dist_ = 0.25;
  if (!nh_private_.getParam ("max_value_diff", max_value_diff))
    max_value_diff = 10.0;
  if (!nh_private_.getParam ("ransac_inlier_threshold", ransac_inlier_threshold))
    ransac_inlier_threshold = 0.10;

  //orb_history_.setCapacity(history_size);
  //canny_history_.setCapacity(history_size);

  orb_reg_.setMaxIterations(icp_max_iterations_);
  orb_reg_.setTransformationEpsilon(icp_tf_epsilon_);
  orb_reg_.setMaxCorrDist(icp_max_corresp_dist_);
  orb_reg_.setMaxValueDiff(max_value_diff);
  orb_reg_.setUseValueRejection(use_value_rejection);
  orb_reg_.setUseRANSACRejection(use_ransac_rejection);
  orb_reg_.setRANSACThreshold(ransac_inlier_threshold);

  canny_reg_.setMaxIterations(icp_max_iterations_/2);
  canny_reg_.setTransformationEpsilon(icp_tf_epsilon_);
  canny_reg_.setMaxCorrDist(icp_max_corresp_dist_);
  canny_reg_.setMaxValueDiff(max_value_diff);
  canny_reg_.setUseValueRejection(use_value_rejection);
  canny_reg_.setUseRANSACRejection(use_ransac_rejection);
  canny_reg_.setRANSACThreshold(ransac_inlier_threshold);

  int orb_smooth;
  double max_range;

  if (!nh_private_.getParam ("max_range", max_range))
    max_range = 3.5;

  if (!nh_private_.getParam ("orb_smooth", orb_smooth))
    orb_smooth = 3;

  orb_detector_.setSmooth(orb_smooth);

  orb_detector_.setMaxRange(max_range);
  canny_detector_.setMaxRange(max_range);

  if (!nh_private_.getParam ("use_alpha_beta", use_alpha_beta_))
    use_alpha_beta_ = true;
  if (!nh_private_.getParam ("alpha", alpha_))
    alpha_ = 1.0;
  if (!nh_private_.getParam ("beta", beta_))
    beta_ = 0.8;

  //orb_reg_.setTransformationEpsilon (icp_tf_epsilon_);
  //orb_reg_.setMaxCorrespondenceDistance (icp_max_corresp_dist_);
  //orb_reg_.setMaximumIterations (icp_max_iterations_);
}

void SparseTrackerAM::pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr)
{
  boost::mutex::scoped_lock(mutex_);

  struct timeval start_callback, end_callback;
  struct timeval start_features, end_features;
  struct timeval start_model, end_model;
  struct timeval start_icp, end_icp;

  gettimeofday(&start_callback, NULL);

  // **** initialize ***********************************************************

  if (!initialized_)
  {
    initialized_ = getBaseToCameraTf(cloud_in_ptr);
    if (!initialized_) return;
  }

  // **** extract features *****************************************************

  gettimeofday(&start_features, NULL);

  // **** ab prediction

  ros::Time cur_time = cloud_in_ptr->header.stamp;
  double dt = (cur_time  - prev_time_).toSec();
  prev_time_ = cur_time;

  tf::Transform predicted_f2b;

  double pr_x, pr_y, pr_z, pr_roll, pr_pitch, pr_yaw;

  if (use_alpha_beta_)
  {
    double cx, cy, cz, croll, cpitch, cyaw;
    getXYZRPY(f2b_, cx, cy, cz, croll, cpitch, cyaw);

    pr_x     = cx     + v_x_     * dt;
    pr_y     = cy     + v_y_     * dt;
    pr_z     = cz     + v_z_     * dt;
    pr_roll  = croll  + v_roll_  * dt;
    pr_pitch = cpitch + v_pitch_ * dt;
    pr_yaw   = cyaw   + v_yaw_   * dt;

    btVector3 pr_pos(pr_x, pr_y, pr_z);
    btQuaternion pr_q;
    pr_q.setRPY(pr_roll, pr_pitch, pr_yaw);

    predicted_f2b.setOrigin(pr_pos);
    predicted_f2b.setRotation(pr_q);
  }
  else
  {
    predicted_f2b = f2b_;
  }

  PointCloudOrb::Ptr orb_features_ptr = boost::make_shared<PointCloudOrb>();
  bool orb_extract_result = extractOrbFeatures(cloud_in_ptr, orb_features_ptr);
  pcl::transformPointCloud (*orb_features_ptr , *orb_features_ptr, eigenFromTf(predicted_f2b * b2c_));
  orb_features_ptr->header.frame_id = fixed_frame_;

  // FIXME - removed canny

  PointCloudCanny::Ptr canny_features_ptr = boost::make_shared<PointCloudCanny>();
  bool canny_extract_result;// = extractCannyFeatures(cloud_in_ptr, canny_features_ptr);
  pcl::transformPointCloud (*canny_features_ptr , *canny_features_ptr, eigenFromTf(predicted_f2b * b2c_));
  canny_features_ptr->header.frame_id = fixed_frame_;

  gettimeofday(&end_features, NULL);

  // **** ICP ******************************

  gettimeofday(&start_icp, NULL);

  bool orb_icp_result = false;
  bool canny_icp_result = false;

  if (orb_extract_result)
  {
    //printf("~ORB~\n");
    orb_icp_result = OrbICP(orb_features_ptr);

    double eps_roll_  = 0.3;
    double eps_pitch_ = 0.3;
    double eps_yaw_   = 0.3;
    double eps_x_     = 0.3;
    double eps_y_     = 0.3;
    double eps_z_     = 0.3;

    if (orb_icp_result)
    {
      tf::Transform corr = tfFromEigen(orb_reg_.getFinalTransformation());

      // particles add error to motion
      double c_x, c_y, c_z, c_roll, c_pitch, c_yaw;
      double e_x, e_y, e_z, e_roll, e_pitch, e_yaw;
      getXYZRPY(corr, c_x, c_y, c_z, c_roll, c_pitch, c_yaw);

      for (int i = 0; i < 20; i++)
      {
        e_roll  = getUrand() * eps_roll_  * c_roll;
        e_pitch = getUrand() * eps_pitch_ * c_pitch;
        e_yaw   = getUrand() * eps_yaw_   * c_yaw;
        e_x = getUrand() * eps_x_  * c_x;
        e_y = getUrand() * eps_y_  * c_y;
        e_z = getUrand() * eps_z_  * c_z;

        btVector3 e_pos(c_x + e_x, c_y + e_y, c_z + e_z);
        btQuaternion e_q;
        e_q.setRPY(c_roll + e_roll, c_pitch + e_pitch, c_yaw + e_yaw);

        tf::Transform e_corr;

        e_corr.setOrigin(e_pos);
        e_corr.setRotation(e_q);

        pf2b_[i] = e_corr * pf2b_[i];
      }

      // end particles

      tf::Transform measured_f2b = corr * predicted_f2b;

      // **** ab estmation
      if (use_alpha_beta_)
      {
        double m_x, m_y, m_z, m_roll, m_pitch, m_yaw;
        getXYZRPY(measured_f2b, m_x, m_y, m_z, m_roll, m_pitch, m_yaw);

        // residuals

        double r_x     = m_x     - pr_x;
        double r_y     = m_y     - pr_y;
        double r_z     = m_z     - pr_z;
        double r_roll  = m_roll  - pr_roll;
        double r_pitch = m_pitch - pr_pitch;
        double r_yaw   = m_yaw   - pr_yaw;

        fixAngleD(r_roll);
        fixAngleD(r_pitch);
        fixAngleD(r_yaw);

        // final position

        double f_x     = pr_x     + alpha_ * r_x;
        double f_y     = pr_y     + alpha_ * r_y;
        double f_z     = pr_z     + alpha_ * r_z;
        double f_roll  = pr_roll  + alpha_ * r_roll;
        double f_pitch = pr_pitch + alpha_ * r_pitch;
        double f_yaw   = pr_yaw   + alpha_ * r_yaw;

        btVector3 f_pos(f_x, f_y, f_z);
        btQuaternion f_q;
        f_q.setRPY(f_roll, f_pitch, f_yaw);

        f2b_.setOrigin(f_pos);
        f2b_.setRotation(f_q);

        // final velocity

        v_x_     = v_x_     + beta_ * r_x     / dt;
        v_y_     = v_y_     + beta_ * r_y     / dt;
        v_z_     = v_z_     + beta_ * r_z     / dt;
        v_roll_  = v_roll_  + beta_ * r_roll  / dt;
        v_pitch_ = v_pitch_ + beta_ * r_pitch / dt;
        v_yaw_   = v_yaw_   + beta_ * r_yaw   / dt;

        //printf("VEL: %f, %f, %f\n", v_x_, v_y_, v_z_);
      }
      else
      {
        f2b_ = measured_f2b;
      }
    }
  }


  if (!orb_icp_result)
    printf("ERROR\n");

  gettimeofday(&end_icp, NULL);

  // **** ADD features to model

  gettimeofday(&start_model, NULL);

  if (model_->points.size() == 0)
  {
    *model_ += *orb_features_ptr;
  }
  else
  {
    pcl::KdTreeFLANN<PointOrb> tree_model;
    tree_model.setInputCloud(model_);

    std::vector<int> indices;
    std::vector<float> distances;

    indices.resize(1);
    distances.resize(1);

    for (int i = 0; i < orb_features_ptr->points.size(); ++i)
    {
      PointOrb& p = orb_features_ptr->points[i];

      int n_found = tree_model.nearestKSearch(p, 1, indices, distances);

      if (n_found == 0)
      {
        model_->points.push_back(p);
        model_->width++;
      }
      else
      {
        if ( distances[0] > 0.01)
        {
          //found far away, insert new one

          model_->points.push_back(p);
          model_->width++;
        }
        else
        {
          // found near, modify old one

          //PointOrb& q = model_->points[indices[0]];
          //q.x = 0.5*(p.x + q.x);
          //q.y = 0.5*(p.y + q.y);
          //q.z = 0.5*(p.z + q.z);
        }
      }
    }
  }

  gettimeofday(&end_model, NULL);

  // *** broadcast tf **********************************************************

  broadcastTF(cloud_in_ptr);

  // *** counter  **************************************************************

  frame_count_++;

  // **** print diagnostics ****************************************************

  gettimeofday(&end_callback, NULL);

  double features_dur  = msDuration(start_features, end_features);
  double model_dur     = msDuration(start_model,    end_model);
  double icp_dur       = msDuration(start_icp,      end_icp);
  double callback_dur  = msDuration(start_callback, end_callback);

  //int model_frames = orb_history_.getSize();
  int icp_iterations = orb_reg_.getFinalIterations();
  int orb_features_count = orb_features_ptr->points.size();
  int canny_features_count =canny_features_ptr->points.size();
  int model_size = model_->points.size();

  printf("F[%d][%d] %.1f \t M[%d] %.1f \t ICP[%d] %.1f \t TOTAL %.1f\n",
      orb_features_count, canny_features_count, features_dur,
      model_size, model_dur,
      icp_iterations,
      icp_dur, callback_dur);

}

void SparseTrackerAM::broadcastTF(const PointCloudT::ConstPtr cloud_in_ptr)
{
  tf::StampedTransform transform_msg(
   f2b_, cloud_in_ptr->header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform (transform_msg);

  visualization_msgs::MarkerArray marker_array;

  for (int i = 0; i < 20; ++i)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = cloud_in_ptr->header.stamp;

    marker.ns = "particle_poses";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //tf::quaternionTFToMsg(pf2b_[i].getRotation() marker.pose.orientation);
    tf::poseTFToMsg(pf2b_[i], marker.pose);

    // Set the scale of the marker
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = (i / 20.0);
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker

    marker_array.markers.push_back(marker);
  }

  marker_pub_.publish(marker_array);
}

/*
void SparseTracker::buildModel(const PointCloudT::ConstPtr cloud_in_ptr,
                                     PointCloudOrb::Ptr model_ptr)
{
 // **** build model ********************************************************

  model_ptr->header = cloud_in_ptr->header;
  orb_history_.getAll(*model_ptr);


  // **** publish

  PointCloudT model_vis;
  pcl::copyPointCloud<PointOrb, PointT>(*model_ptr, model_vis);

  //model_pub_.publish(model_vis);
}
*/

bool SparseTrackerAM::extractCannyFeatures(const PointCloudT::ConstPtr cloud_in_ptr,
                                                 PointCloudCanny::Ptr& features_ptr)
{
  features_ptr->header = cloud_in_ptr->header;

  // **** Extract ORB features

  canny_detector_.findFeatures(*cloud_in_ptr, *features_ptr);

  if (features_ptr->points.size() < min_features_)
  {
    return false;
  }

  // **** Publish Features *****************************************************

  features_ptr->header = cloud_in_ptr->header;

  PointCloudT features_vis;
  pcl::copyPointCloud<PointCanny, PointT>(*features_ptr, features_vis);
  canny_features_pub_.publish(features_vis);

  return true;
}

bool SparseTrackerAM::extractOrbFeatures(const PointCloudT::ConstPtr cloud_in_ptr,
                                               PointCloudOrb::Ptr& features_ptr)
{
  features_ptr->header = cloud_in_ptr->header;

  // **** Extract ORB features

  orb_detector_.findFeatures(*cloud_in_ptr, *features_ptr);

  // **** Filter Features ******************************************************

  if (features_ptr->points.size() < min_features_)
  {
    return false;
  }

  // **** Publish Features *****************************************************

  features_ptr->header = cloud_in_ptr->header;

  PointCloudT features_vis;
  pcl::copyPointCloud<PointOrb, PointT>(*features_ptr, features_vis);
  orb_features_pub_.publish(features_vis);

  return true;
}

bool SparseTrackerAM::getBaseToCameraTf(const PointCloudT::ConstPtr cloud_in_ptr)
{
  tf::StampedTransform tf_m;

  try
  {
    tf_listener_.waitForTransform(
      base_frame_, cloud_in_ptr->header.frame_id, cloud_in_ptr->header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, cloud_in_ptr->header.frame_id, cloud_in_ptr->header.stamp, tf_m);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Sparse Tracker: Transform unavailable (%s)", ex.what());
    return false;
  }

  b2c_ = tf_m;

  return true;
}

bool SparseTrackerAM::OrbICP(PointCloudOrb::Ptr& orb_features_ptr)
{
  // **** build orb model *****************************************************

  //PointCloudOrb::Ptr orb_model_ptr = boost::make_shared<PointCloudOrb>();
  //orb_model_ptr->header.frame_id = fixed_frame_;
  //orb_history_.getAll(*orb_model_ptr);

  if (model_->points.size() == 0) return false;

  PointCloudT orb_model_vis;
  pcl::copyPointCloud<PointOrb, PointT>(*model_, orb_model_vis);
  orb_model_pub_.publish(orb_model_vis);

  // **** orb icp ***********************************************************

  pcl::KdTreeFLANN<PointOrb> tree_data;
  pcl::KdTreeFLANN<PointOrb> tree_model;

  tree_data.setInputCloud(orb_features_ptr);
  tree_model.setInputCloud(model_);

  orb_reg_.setDataCloud  (&*orb_features_ptr);
  orb_reg_.setModelCloud (&*model_);

  orb_reg_.setDataTree  (&tree_data);
  orb_reg_.setModelTree (&tree_model);

  bool result = orb_reg_.align();

  return result;
}
/*
bool SparseTrackerAM::CannyICP(PointCloudCanny::Ptr& canny_features_ptr)
{
  // **** build canny model **************************************************

   PointCloudCanny::Ptr canny_model_ptr = boost::make_shared<PointCloudCanny>();
   canny_model_ptr->header.frame_id = fixed_frame_;
   canny_history_.getAll(*canny_model_ptr);

   PointCloudT canny_model_vis;
   pcl::copyPointCloud<PointCanny, PointT>(*canny_model_ptr, canny_model_vis);
   canny_model_pub_.publish(canny_model_vis);

   // **** canny icp ***********************************************************

   pcl::KdTreeFLANN<PointCanny> tree_data;
   pcl::KdTreeFLANN<PointCanny> tree_model;

   tree_data.setInputCloud(canny_features_ptr);
   tree_model.setInputCloud(canny_model_ptr);
   canny_reg_.setDataCloud  (&*canny_features_ptr);
   canny_reg_.setModelCloud (&*canny_model_ptr);
   canny_reg_.setDataTree  (&tree_data);
   canny_reg_.setModelTree (&tree_model);

   bool result = canny_reg_.align();

   return result;
}
*/
} //namespace ccny_rgbd
