#include "ccny_rgbd/registration/sparse_tracker.h"

namespace ccny_rgbd {

SparseTracker::SparseTracker(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  frame_count_(0),
  initialized_(false)
{
  ROS_INFO("Starting Sparse Tracker");

  // **** init parameters

  initParams();

  // **** init variables

  f2b_.setIdentity();
  last_keyframe_f2b_ = f2b_;

  loop_solver_ = new LoopSolver(nh_, nh_private, &keyframes_, &b2c_);

  // **** publishers

  features_pub_ = nh_.advertise<PointCloudFeature>(
    pub_features_topic_, 1);
   keyframes_pub_ = nh_.advertise<PointCloudT>(
    pub_keyframes_topic_, 1);

  // **** subscribers

  point_cloud_sub_ = nh_.subscribe<PointCloudT>(
    sub_topic_, 1, &SparseTracker::pointCloudCallback, this);

  // **** services

 pub_frame_service_ = nh_.advertiseService(
    "publish_keyframe", &SparseTracker::publishFrameSrvCallback, this);
  pub_frames_service_ = nh_.advertiseService(
    "publish_keyframes", &SparseTracker::publishAllFramesSrvCallback, this);
}

SparseTracker::~SparseTracker()
{
  ROS_INFO("Destroying Feature Viewer"); 

  delete feature_detector_;
}

void SparseTracker::initParams()
{
  int history_size;

  // TODO - change to dynamic
  kf_dist_eps_  = 0.15; 
  kf_angle_eps_ = 15.0 * M_PI / 180.0; 

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/base_link";

/*
  if (!nh_private_.getParam ("use_vgf_features_filter", use_vgf_features_filter_))
    use_vgf_features_filter_ = false;
  if (!nh_private_.getParam ("vgf_features_res", vgf_features_res_))
    vgf_features_res_ = 0.01;
  if (!nh_private_.getParam ("use_vgf_model_filter", use_vgf_model_filter_))
    use_vgf_model_filter_ = false;
  if (!nh_private_.getParam ("vgf_model_res", vgf_model_res_))
    vgf_model_res_ = 0.01;
*/

  if (!nh_private_.getParam ("min_features", min_features_))
    min_features_ = 10;
  if (!nh_private_.getParam ("history_size", history_size))
    history_size = 5;

  // **** registration parameters

  int icp_nr_neighbors;
  bool use_value_rejection;
  double max_value_diff;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;

  if (!nh_private_.getParam ("reg/type", reg_type_))
    reg_type_ = 0;
  if (!nh_private_.getParam ("reg/max_iterations", icp_max_iterations_))
    icp_max_iterations_ = 30;
  if (!nh_private_.getParam ("reg/tf_epsilon", icp_tf_epsilon_))
    icp_tf_epsilon_ = 0.000001;
  if (!nh_private_.getParam ("reg/use_value_rejection", use_value_rejection))
    use_value_rejection = false;
  if (!nh_private_.getParam ("reg/use_ransac_rejection", use_ransac_rejection))
    use_ransac_rejection = false;
  if (!nh_private_.getParam ("reg/max_corresp_dist", icp_max_corresp_dist_))
    icp_max_corresp_dist_ = 0.15;
  if (!nh_private_.getParam ("reg/max_value_diff", max_value_diff))
    max_value_diff = 10.0;
  if (!nh_private_.getParam ("reg/ransac_inlier_threshold", ransac_inlier_threshold))
    ransac_inlier_threshold = 0.10;
  if (!nh_private_.getParam ("reg/nr_neighbors", icp_nr_neighbors))
    icp_nr_neighbors = 16;

  feature_history_.setCapacity(history_size);

  reg_.setMaxIterations(icp_max_iterations_);
  reg_.setTransformationEpsilon(icp_tf_epsilon_);
  reg_.setMaxCorrDist(icp_max_corresp_dist_);
  reg_.setMaxValueDiff(max_value_diff);
  reg_.setUseValueRejection(use_value_rejection);
  reg_.setUseRANSACRejection(use_ransac_rejection);
  reg_.setRANSACThreshold(ransac_inlier_threshold);

  // **** common feature params

  double max_range;
  int smooth;
  int window;
  bool compute_descriptors;

  if (!nh_private_.getParam ("feature/detector_type", detector_type_))
    detector_type_ = 0;
  if (!nh_private_.getParam ("feature/smooth", smooth))
    smooth = 0;
  if (!nh_private_.getParam ("feature/window", window))
    window = 0;
  if (!nh_private_.getParam ("feature/max_range", max_range))
    max_range = 5.5;
  if (!nh_private_.getParam ("feature/compute_descriptors", compute_descriptors))
    compute_descriptors = false;

  if (detector_type_ == 0)
    feature_detector_ = new OrbDetector();
  else if (detector_type_ == 1)
    feature_detector_ = new SurfDetector();
  else if (detector_type_ == 2)
    feature_detector_ = new CannyDetector();
  else if (detector_type_ == 3)
    feature_detector_ = new GftDetector();
  else
    ROS_FATAL("Wrong detector type");

  feature_detector_->setSmooth(smooth);
  feature_detector_->setWindow(window);
  feature_detector_->setComputeDescriptors(compute_descriptors);
  feature_detector_->setMaxRange(max_range);

  // **** gicp

  GICPParams params;

  if (!nh_private_.getParam ("gicp/max_distance", params.max_distance))
     params.max_distance = 0.10;
  if (!nh_private_.getParam ("gicp/solve_rotation", params.solve_rotation))
     params.solve_rotation = true;
  if (!nh_private_.getParam ("gicp/max_iterations", params.max_iteration))
     params.max_iteration = 20;
  if (!nh_private_.getParam ("gicp/max_iteration_inner", params.max_iteration_inner))
     params.max_iteration_inner = 20;
  if (!nh_private_.getParam ("gicp/epsilon", params.epsilon))
     params.epsilon = 1e-4;
  if (!nh_private_.getParam ("gicp/epsilon_rot", params.epsilon_rot))
     params.epsilon_rot = 1e-3;

  params.debug = false;

  gicp_reg_.setParams(params);
  gicp_reg_.setUseColor(false);
}

void SparseTracker::pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr)
{
  boost::mutex::scoped_lock(mutex_);

  struct timeval start_callback, end_callback;
  struct timeval start_frame, end_frame;
  struct timeval start_features, end_features;
  struct timeval start_icp, end_icp;

  gettimeofday(&start_callback, NULL);

  // **** initialize ***********************************************************

  if (!initialized_)
  {
    initialized_ = getBaseToCameraTf(cloud_in_ptr);
    if (!initialized_) return;
  }

  // **** create rgbd frame **************************************************

  gettimeofday(&start_frame, NULL);

  RGBDFrame frame;

  frame.id = keyframes_.size();
  frame.data = *cloud_in_ptr; 
  frame.computeRGBImage();
  
  frame.simpleFilter();

  gettimeofday(&end_frame, NULL);

  // **** extract features *****************************************************

  gettimeofday(&start_features, NULL);

  feature_detector_->findFeatures(frame);
  features_pub_.publish(frame.rgb_features);

  PointCloudFeature::Ptr features_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  pcl::transformPointCloud (frame.rgb_features , *features_ptr, eigenFromTf(f2b_* b2c_));
  features_ptr->header.frame_id = fixed_frame_;

  bool feature_extract_result =
    ((int)frame.rgb_features.points.size() >= min_features_);

  gettimeofday(&end_features, NULL);

  // **** ICP ******************************

  gettimeofday(&start_icp, NULL);

  bool icp_result = false;
  tf::Transform corr;

  if (feature_extract_result)
  {
    if(reg_type_ == 0)
    {
      icp_result = ICP(features_ptr, corr);
    }
    else if (reg_type_ == 1)
    {
      icp_result = GICP(features_ptr, corr);
    }
  }

  if (icp_result) f2b_ = corr * f2b_;
  else printf("ICP ERROR\n");

  if(!feature_extract_result)
    feature_history_.reset();

  pcl::transformPointCloud (frame.rgb_features , *features_ptr, eigenFromTf(f2b_* b2c_));
  features_ptr->header.frame_id = fixed_frame_;

  feature_history_.add(*features_ptr);

  gettimeofday(&end_icp, NULL);

  // *** broadcast tf **********************************************************

  broadcastTF(cloud_in_ptr);

  // *** keyframe testing & generation *****************************************

  // compute the distance and angle change since last keyframe pose
  double dist, angle;
  getTfDifference(last_keyframe_f2b_, f2b_, dist, angle);

  if (keyframes_.empty() || dist > kf_dist_eps_ || angle > kf_angle_eps_)
  {
    frame.pose = f2b_* b2c_;
    frame.computeFeatureBBX();
    keyframes_.push_back(frame);

    if(keyframes_.size() == 1)
    {
      // first frame
      frame.path_length_linear  = 0.0;
      frame.path_length_angular = 0.0;
    }
    else
    {
      RGBDFrame& prev_frame =  keyframes_[keyframes_.size() - 2];

      frame.path_length_linear  = prev_frame.path_length_linear  + dist;
      frame.path_length_angular = prev_frame.path_length_angular + angle;

      loop_solver_->addVertex(frame);
    }

    // save the features from this keyframe, to be used in model for ICP
    last_keyframe_features_ = *features_ptr;  

    last_keyframe_f2b_ = f2b_;
  }

  // *** counter  **************************************************************

  frame_count_++;

  // **** print diagnostics ****************************************************

  gettimeofday(&end_callback, NULL);

  double frame_dur     = msDuration(start_frame,    end_frame);
  double features_dur  = msDuration(start_features, end_features);
  double icp_dur       = msDuration(start_icp,      end_icp);
  double callback_dur  = msDuration(start_callback, end_callback);

  int icp_iterations;
  
  if      (reg_type_ == 0) icp_iterations = reg_.getFinalIterations();
  else if (reg_type_ == 1) icp_iterations = gicp_reg_.getFinalIterations();

  int features_count = features_ptr->points.size();

  printf("Fr %.1f \t F[%d] %.1f \t ICP[%d] %.1f \t TOTAL %.1f\n",
    frame_dur,
    features_count, features_dur,
    icp_iterations, icp_dur, 
    callback_dur);
}

void SparseTracker::broadcastTF(const PointCloudT::ConstPtr cloud_in_ptr)
{
  tf::StampedTransform transform_msg(
   f2b_, cloud_in_ptr->header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform (transform_msg);
}

bool SparseTracker::ICP(PointCloudFeature::Ptr& features_ptr, tf::Transform& corr)
{
  // **** build model ***************************************************

  PointCloudFeature::Ptr model_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  // add in recent features
  model_ptr->header.frame_id = fixed_frame_;
  feature_history_.getAll(*model_ptr);

  // add in last keyframe features
  if (!keyframes_.empty())
    *model_ptr += last_keyframe_features_;

  if (model_ptr->points.empty())
  {
    ROS_WARN("No points in model");
    return false;
  }

  // TODO - model/data vgf downsampling here?

  // **** icp ***********************************************************

  pcl::KdTreeFLANN<PointFeature> tree_data;
  pcl::KdTreeFLANN<PointFeature> tree_model;

  tree_data.setInputCloud(features_ptr);
  tree_model.setInputCloud(model_ptr);

  reg_.setDataCloud  (&*features_ptr);
  reg_.setModelCloud (&*model_ptr);

  reg_.setDataTree  (&tree_data);
  reg_.setModelTree (&tree_model);

  bool result = reg_.align();

  corr = tfFromEigen(reg_.getFinalTransformation());

  return result;
}

bool SparseTracker::getBaseToCameraTf(const PointCloudT::ConstPtr cloud_in_ptr)
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

bool SparseTracker::GICP(PointCloudFeature::Ptr& features_ptr, tf::Transform& corr)
{
  // ****  set up gicp data set ************************************************

  ccny_gicp::GICPPointSetKd gicp_data;

  for (unsigned int i = 0; i < features_ptr->points.size(); ++i)
  {
    PointGICP p;
    p.x   = features_ptr->points[i].x;
    p.y   = features_ptr->points[i].y;
    p.z   = features_ptr->points[i].z;
    //p.rgb = features_ptr->points[i].rgb;

    p.C[0][0] = 0.01; p.C[0][1] = 0.00; p.C[0][2] = 0.00;
    p.C[1][0] = 0.00; p.C[1][1] = 0.01; p.C[1][2] = 0.00;
    p.C[2][0] = 0.00; p.C[2][1] = 0.00; p.C[2][2] = 0.01 * p.z;

    gicp_data.AppendPoint(p);
  }

  gicp_reg_.setData(&gicp_data);

  // **** set up gicp model set ************************************************

  ccny_gicp::GICPPointSetKd gicp_model;

  PointCloudFeature::Ptr model_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  // add in recent features
  model_ptr->header.frame_id = fixed_frame_;
  feature_history_.getAll(*model_ptr);

  // add in last keyframe features
  if (!keyframes_.empty())
    *model_ptr += last_keyframe_features_;

  if (model_ptr->points.empty()) return false;

  for (unsigned int i = 0; i < model_ptr->points.size(); ++i)
  {
    PointGICP p;
    p.x   = model_ptr->points[i].x;
    p.y   = model_ptr->points[i].y;
    p.z   = model_ptr->points[i].z;
    //p.rgb = model_ptr->points[i].rgb;

    p.C[0][0] = 0.01; p.C[0][1] = 0.00; p.C[0][2] = 0.00;
    p.C[1][0] = 0.00; p.C[1][1] = 0.01; p.C[1][2] = 0.00;
    p.C[2][0] = 0.00; p.C[2][1] = 0.00; p.C[2][2] = 0.01;

    gicp_model.AppendPoint(p);
  }

  // create kd tree
  KdTree gicp_model_kdtree;
  gicp_model_kdtree.setInputCloud(gicp_model.getCloud());
  gicp_model.setKdTree(&gicp_model_kdtree);

  gicp_reg_.setModel(&gicp_model);

  // **** icp ******************************************************************

  Eigen::Matrix4f corr_eigen;
  gicp_reg_.align(corr_eigen);
  corr = tfFromEigen(corr_eigen); 

  return true;
}

bool SparseTracker::publishFrameSrvCallback(
  ccny_rgbd::PublishFrame::Request& request,
  ccny_rgbd::PublishFrame::Response& response)
{
  publishFrame(request.id);
  usleep(50000);
}

bool SparseTracker::publishAllFramesSrvCallback(
  ccny_rgbd::PublishAllFrames::Request&  request,
  ccny_rgbd::PublishAllFrames::Response& response)
{
  loop_solver_->publishEdges();

  for (unsigned int i = 0; i < keyframes_.size(); i += request.step)
  {
    publishFrame(i);
    usleep(25000);
  }

  return true;
}

void SparseTracker::publishFrame(int i)
{
  PointCloudT keyframe_data;

  pcl::transformPointCloud(
    keyframes_[i].data, keyframe_data, eigenFromTf(keyframes_[i].pose));

  keyframe_data.header.frame_id="odom";

  keyframes_pub_.publish(keyframe_data);
  usleep(50000);
}

} //namespace ccny_rgbd
