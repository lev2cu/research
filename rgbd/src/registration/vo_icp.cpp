#include "ccny_rgbd/registration/vo_icp.h"

namespace ccny_rgbd {

VOICP::VOICP(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  frame_count_(0),
  initialized_(false)
{
  ROS_INFO("Starting VO ICP");

  // **** init parameters

  initParams();

  // **** init variables

  f2b_.setIdentity();
  last_keyframe_f2b_ = f2b_;

  twist_.linear.x = 0.0;
  twist_.linear.y = 0.0;
  twist_.linear.z = 0.0;
  twist_.angular.x = 0.0;
  twist_.angular.y = 0.0;
  twist_.angular.y = 0.0;

  // **** publishers

  features_pub_ = nh_.advertise<PointCloudFeature>(
    pub_features_topic_, 1);
  keyframes_pub_ = nh_.advertise<PointCloudT>(
    pub_keyframes_topic_, 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
    pub_pose_topic_, 1);

  // **** services

  pub_frame_service_ = nh_.advertiseService(
    "publish_keyframe", &VOICP::publishFrameSrvCallback, this);
  pub_frames_service_ = nh_.advertiseService(
    "publish_keyframes", &VOICP::publishAllFramesSrvCallback, this);

  // **** subscribers

  vel_sub_ = nh_.subscribe(
    "vel", 1, &VOICP::velCallback, this);

  image_transport::ImageTransport rgb_it(nh_);
  image_transport::ImageTransport depth_it(nh_);

  //image_transport::TransportHints hints("raw", ros::TransportHints(), nh_private_);
  sub_depth_.subscribe(
    depth_it, "/camera/depth_registered/image_rect_raw", 1);
  sub_rgb_.subscribe(
    rgb_it, "/camera/rgb/image_rect_color", 1);
  sub_info_.subscribe(
    nh_, "/camera/rgb/camera_info", 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;
  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&VOICP::imageCb, this, _1, _2, _3));  
}

VOICP::~VOICP()
{
  ROS_INFO("Destroying VO ICP"); 

  delete feature_detector_;
}

void VOICP::velCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
  twist_ = twist_msg->twist;
}

void VOICP::initParams()
{
  // TODO - change to dynamic
  kf_dist_eps_  = 0.15; 
  kf_angle_eps_ = 15.0 * M_PI / 180.0; 

  int history_size;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  if (!nh_private_.getParam ("min_features", min_features_))
    min_features_ = 10;
  if (!nh_private_.getParam ("history_size", history_size))
    history_size = 5;
  if (!nh_private_.getParam ("publish_keyframes", publish_keyframes_))
    publish_keyframes_ = true;

  // **** registration parameters

  int icp_nr_neighbors;
  bool use_value_rejection;
  double max_value_diff;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;

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
    detector_type_ = "GFT";
  if (!nh_private_.getParam ("feature/smooth", smooth))
    smooth = 0;
  if (!nh_private_.getParam ("feature/window", window))
    window = 0;
  if (!nh_private_.getParam ("feature/max_range", max_range))
    max_range = 5.5;
  if (!nh_private_.getParam ("feature/compute_descriptors", compute_descriptors))
    compute_descriptors = false;

  if (detector_type_ == "ORB")
    feature_detector_ = new OrbDetector();
  else if (detector_type_ == "SURF")
    feature_detector_ = new SurfDetector();
  else if (detector_type_ == "GFT")
    feature_detector_ = new GftDetector();
  else if (detector_type_ == "KLT")
    feature_detector_ = new KltDetector();
  else
    ROS_FATAL("Wrong detector type");

  feature_detector_->setSmooth(smooth);
  feature_detector_->setMaxRange(max_range);
}

void VOICP::imageCb(
  const sensor_msgs::ImageConstPtr& depth_msg,
  const sensor_msgs::ImageConstPtr& rgb_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  struct timeval start_callback, end_callback;
  struct timeval start_frame, end_frame;
  struct timeval start_icp, end_icp;
  struct timeval start_features, end_features;
  struct timeval start_data, end_data;

  gettimeofday(&start_callback, NULL);

  // **** initialize ***********************************************************

  if (!initialized_)
  {
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    if (!initialized_) return;
  }

  // **** create frame ***********************************************************

  gettimeofday(&start_frame, NULL);
  RGBDFrame2 frame(rgb_msg, depth_msg, info_msg);
  frame.id = keyframes_.size();
  gettimeofday(&end_frame, NULL);

  // **** find features ***********************************************************

  gettimeofday(&start_features, NULL);
  feature_detector_->findFeatures(frame);

  PointCloudFeature::Ptr features_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  // rotate into the fixed frame
  pcl::transformPointCloud(frame.features , *features_ptr, eigenFromTf(f2b_* b2c_));
  features_ptr->header.frame_id = fixed_frame_;

  bool feature_extract_result =
    ((int)frame.features.points.size() >= min_features_);

  // optional
  features_pub_.publish(frame.features);

  gettimeofday(&end_features, NULL);

  // **** Prediction ******************************
/*
  tf::Transform pred_tf;
  pred_tf.setIdentity();

  if (0)  // alpha beta prediciton
  {
    float dt = (rgb_msg->header.stamp - last_icp_time_).toSec();
    last_icp_time_ = rgb_msg->header.stamp;

    tf::Vector3 pred_pos(
      twist_.linear.x * dt, twist_.linear.y * dt, twist_.linear.z * dt);

    tf::Quaternion pred_q;
    pred_q.setRPY( 
      twist_.angular.x * dt, twist_.angular.y * dt, twist_.angular.z * dt);

    pred_tf.setOrigin(pred_pos);
    pred_tf.setRotation(pred_q);

    pcl::transformPointCloud (*features_ptr, *features_ptr, eigenFromTf(pred_tf));
  }*/

  // **** ICP ******************************

  gettimeofday(&start_icp, NULL);

  tf::Transform corr;

  bool icp_result = ICP(features_ptr, corr);

  if (icp_result)
  { 
    //f2b_ = corr * pred_tf * f2b_;
    f2b_ = corr * f2b_;
    broadcastTF(rgb_msg->header);
  }

  // TODO: is this the best thing to do?
  if(!feature_extract_result) feature_history_.reset();

  // transform features using the new estimate for the fixed frame
  pcl::transformPointCloud (frame.features , *features_ptr, eigenFromTf(f2b_* b2c_));
  features_ptr->header.frame_id = fixed_frame_;

  feature_history_.add(*features_ptr);

  gettimeofday(&end_icp, NULL);

  // *** keyframe testing & generation *****************************************

  // compute the distance and angle change since last keyframe pose
  double dist, angle;

  if(keyframes_.empty())
  {
    // first keyframe
    RGBDKeyframe keyframe(frame);
    keyframe.pose = f2b_ * b2c_;
    keyframe.path_length_linear  = 0.0;
    keyframe.path_length_angular = 0.0;

    keyframes_.push_back(keyframe);// TODO - unneccesary copy here?

    last_keyframe_features_ = *features_ptr;  
    last_keyframe_f2b_ = f2b_;

    if (publish_keyframes_) publishKeyframe(keyframes_.size() - 1);
  }
  else
  {
    // check distance between current pose and prev. frame
    RGBDKeyframe& prev_keyframe = keyframes_[keyframes_.size() - 1];
    getTfDifference(prev_keyframe.pose, f2b_ * b2c_, dist, angle);

    // if we moved sufficeiently, create new keyframe
    if (dist > kf_dist_eps_ || angle > kf_angle_eps_)
    {
      RGBDKeyframe keyframe(frame);
      keyframe.pose = f2b_ * b2c_;
      keyframe.path_length_linear  = prev_keyframe.path_length_linear  + dist;
      keyframe.path_length_angular = prev_keyframe.path_length_angular + angle;
      keyframes_.push_back(keyframe); // TODO - unneccesary copy here?

      if (publish_keyframes_) publishKeyframe(keyframes_.size() - 1);

      last_keyframe_features_ = *features_ptr;  
      last_keyframe_f2b_ = f2b_;

      //loop_solver_->addVertex(frame);
    } 
  }

  // *** counter  **************************************************************

  frame_count_++;

  // **** print diagnostics ****************************************************

  gettimeofday(&end_callback, NULL);

  double frame_dur     = msDuration(start_frame,    end_frame);
  double features_dur  = msDuration(start_features, end_features);
  double data_dur      = msDuration(start_data,     end_data);
  double icp_dur       = msDuration(start_icp,      end_icp);
  double callback_dur  = msDuration(start_callback, end_callback);

  int features_count = frame.features.points.size();
  int icp_iterations = reg_.getFinalIterations();

  printf("Fr %.1f \t Ft[%d] %.1f \t ICP[%d] %.1f \t TOTAL %.1f\n",
    frame_dur,
    features_count, features_dur,
    icp_iterations, icp_dur,
    callback_dur);
}

void VOICP::broadcastTF(const std_msgs::Header& header)
{
  tf::StampedTransform transform_msg(
   f2b_, header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform (transform_msg);

  geometry_msgs::PoseStamped pose_msg;
  tf::poseTFToMsg(f2b_, pose_msg.pose);
  pose_msg.header = header;
  pose_pub_.publish(pose_msg);
}

bool VOICP::ICP(PointCloudFeature::Ptr& features_ptr, tf::Transform& corr)
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

bool VOICP::getBaseToCameraTf(const std_msgs::Header& header)
{
  tf::StampedTransform tf_m;

  try
  {
    tf_listener_.waitForTransform(
      base_frame_, header.frame_id, header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, header.frame_id, header.stamp, tf_m);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Sparse Tracker: Transform unavailable (%s)", ex.what());
    return false;
  }

  b2c_ = tf_m;

  return true;
}

bool VOICP::publishFrameSrvCallback(
  ccny_rgbd::PublishFrame::Request& request,
  ccny_rgbd::PublishFrame::Response& response)
{
  if (request.id < 0 || request.id >= (int)keyframes_.size())
  {
    ROS_ERROR("request.id %d out of bounds (%d keyframes)", request.id, keyframes_.size());
    return false;
  }

  publishKeyframe(request.id);
  usleep(50000);
  return true;
}

bool VOICP::publishAllFramesSrvCallback(
  ccny_rgbd::PublishAllFrames::Request&  request,
  ccny_rgbd::PublishAllFrames::Response& response)
{
  if (request.step <= 0)
  {
    ROS_ERROR("request.step has to be >= 1");
    return false;
  }

  for (int i = 0; i < (int)keyframes_.size(); i += request.step)
  {
    publishKeyframe(i);
    usleep(25000);
  }

  return true;
}

void VOICP::publishKeyframe(int i)
{
  PointCloudT keyframe_data;

  keyframes_[i].constructDataCloud();

  pcl::transformPointCloud(
    keyframes_[i].data, keyframe_data, eigenFromTf(keyframes_[i].pose));

  keyframe_data.header.frame_id = fixed_frame_;

  keyframes_pub_.publish(keyframe_data);
}

} //namespace ccny_rgbd
