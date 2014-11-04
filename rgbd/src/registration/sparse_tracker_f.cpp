#include "ccny_rgbd/registration/sparse_tracker_f.h"

namespace ccny_rgbd {

SparseTrackerF::SparseTrackerF(ros::NodeHandle nh, ros::NodeHandle nh_private):
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
  last_f2b_ = f2b_;

  if (detector_type_ == 0)
    feature_detector_ = new OrbDetector();
  else if (detector_type_ == 1)
    feature_detector_ = new SurfDetector();
  else if (detector_type_ == 2)
    feature_detector_ = new CannyDetector();
  else
    ROS_FATAL("Wrong detector type");

  // **** publishers

  features_pub_ = nh_.advertise<PointCloudT>(
    pub_features_topic_, 1);
  model_pub_    = nh_.advertise<PointCloudT>(
    pub_model_topic_, 1);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
    pub_pose_topic_, 1);
  poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
    pub_poses_topic_, 1);
  frames_pub_ = nh_.advertise<PointCloudT>(
    pub_frames_topic_, 1);
  all_frames_pub_ = nh_.advertise<PointCloudT>(
    pub_all_frames_topic_, 1);
  edges_pub_ = nh_.advertise<visualization_msgs::Marker>(
    pub_edges_topic_, 1);

  // **** subscribers

  point_cloud_subscriber_ = nh_.subscribe<PointCloudT>(
    sub_topic_, 1, &SparseTrackerF::pointCloudCallback, this);

  // **** services

  loop_service_ = nh_.advertiseService(
    "loop", &SparseTrackerF::loopSrvCallback, this);
}

SparseTrackerF::~SparseTrackerF()
{
  ROS_INFO("Destroying Feature Viewer"); 

  delete feature_detector_;
}

void SparseTrackerF::initParams()
{
  kf_dist_eps_ = 0.15; // TODO - change to dynamic
  kf_angle_eps_ = 10.0 * M_PI / 180.0; 
  int icp_nr_neighbors;

  bool use_value_rejection;
  double max_value_diff;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;

  int history_size;
  double max_range;

  int orb_smooth;
  int orb_window;
  int n_orb_features;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/base_link";

  if (!nh_private_.getParam ("detector_type", detector_type_))
    detector_type_ = 0;

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

  if (!nh_private_.getParam ("history_size", history_size))
    history_size = 5;
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

  if (!nh_private_.getParam ("ransac_matching_distance", ransac_matching_distance_))
    ransac_matching_distance_ = 10.0;
  if (!nh_private_.getParam ("ransac_eps_reproj", ransac_eps_reproj_))
    ransac_eps_reproj_ = 10.0;
  if (!nh_private_.getParam ("ransac_inlier_threshold", ransac_inlier_threshold_))
    ransac_inlier_threshold_ = 0.40;
  if (!nh_private_.getParam ("ransac_max_iterations", ransac_max_iterations_))
    ransac_max_iterations_ = 100;

  feature_history_.setCapacity(history_size);

  reg_.setMaxIterations(icp_max_iterations_);
  reg_.setTransformationEpsilon(icp_tf_epsilon_);
  reg_.setMaxCorrDist(icp_max_corresp_dist_);
  reg_.setMaxValueDiff(max_value_diff);
  reg_.setUseValueRejection(use_value_rejection);
  reg_.setUseRANSACRejection(use_ransac_rejection);
  reg_.setRANSACThreshold(ransac_inlier_threshold);

  if (!nh_private_.getParam ("max_range", max_range))
    max_range = 3.5;
  if (!nh_private_.getParam ("orb_smooth", orb_smooth))
    orb_smooth = 0;
  if (!nh_private_.getParam ("orb_window", orb_window))
    orb_window = 0;

  feature_detector_->setSmooth(orb_smooth);
  feature_detector_->setWindow(orb_window);
  feature_detector_->setComputeDescriptors(use_value_rejection);
  feature_detector_->setMaxRange(max_range);
}

void SparseTrackerF::loopClose()
{
  int treeType = 1 ;
  bool ignorePreconditioner=false;
  bool adaptiveRestart = false;
  int loop_iterations = 50;

  AISNavigation::TreeOptimizer3::EdgeCompareMode compareMode =
    AISNavigation::EVComparator<AISNavigation::TreeOptimizer3::Edge*>::CompareLevel;

  AISNavigation::TreeOptimizer3 pg;

  pg.verboseLevel = 2;
  pg.restartOnDivergence = adaptiveRestart;

  // construct vertices

  for (unsigned int i = 0; i < keyframes_.size(); ++i)
  {
    double x, y, z, roll, pitch, yaw;
    getXYZRPY(keyframes_[i].pose, x, y, z, roll, pitch, yaw);

    Pose3<double> v_pose(x, y, z, roll, pitch, yaw);

    AISNavigation::TreePoseGraph3::Vertex* v = pg.addVertex(i, v_pose);
    if (v){
      v->transformation = AISNavigation::TreePoseGraph3::Transformation(v_pose);
    }
  }

  // construct edges

  for (unsigned int i = 0; i < edges_.size(); ++i)
  {
    if(!edges_[i].tf_computed) continue;

    double x, y, z, roll, pitch, yaw;

    getXYZRPY(edges_[i].a2b, x, y, z, roll, pitch, yaw);

    Transformation3<double> t(x, y, z, roll, pitch, yaw);
    //Transformation3<double> t(0, 0, 0, 0, 0, 0);

    DMatrix<double> inf(6,6);
    inf = DMatrix<double>::I(6);

    bool result = pg.addEdge(pg.vertex(edges_[i].index_a),
                             pg.vertex(edges_[i].index_b),
                             t, inf);

    if(!result)
      printf("Fatal, attempting to insert an edge between non existing nodes\n");
  }

  printf("#nodes: %d #edges %d\n", pg.vertices.size(), pg.edges.size());


  if (treeType == 0)
    pg.buildSimpleTree();
  else
    pg.buildMST(pg.vertices.begin()->first);

  pg.initializeOnTree();
  pg.initializeTreeParameters();
  pg.initializeOptimization(compareMode);

  double l = pg.totalPathLength();
  int nEdges=pg.edges.size();
  double apl=l/(double)(nEdges);
  printf("Average path length = %f\n", apl);
  printf("Complexity of an iteration = %f\n", l);

  // optimize

  for (int i=0; i < loop_iterations; i++)
  {
     pg.iterate(0, ignorePreconditioner);

     /*
     if (1)
     {
       char b[30];
       sprintf(b,"/home/idryanov/ros/%04d",i);
       std::string output = b;
       printf("saving to %s", output.c_str());
       pg.saveGnuplot(output.c_str());
     }*/

     // compute the error and dump it
     double mte, mre, are, ate;
     double error = pg.error(&mre, &mte, &are, &ate);
     printf("iteration %d RotGain = %f\n", i, pg.getRotGain());
     printf("\t global error = %f\n", error);
  }

  // update transformations from poses?
  for (AISNavigation::TreePoseGraph3::VertexMap::iterator it = pg.vertices.begin();
      it != pg.vertices.end();
      it++)
  {
    AISNavigation::TreePoseGraph3::Vertex * v = it->second;
    v->pose = v->transformation.toPoseType();
  }

  for (unsigned int i = 0; i < keyframes_.size(); ++i)
  {
    //Pose3<double> pose = pg.vertex(i)->transformation.toPoseType();
    Pose3<double> pose = pg.vertex(i)->pose;

    btVector3 v(pose.x(), pose.y(), pose.z());
    btQuaternion q = tf::createQuaternionFromRPY(pose.roll(), pose.pitch(), pose.yaw());

    tf::Transform t;
    t.setOrigin(v);
    t.setRotation(q);

    keyframes_[i].pose = t;
  }

  for (unsigned int i = 0; i < edges_.size(); ++i)
  {
   edges_[i].tf_computed = false;
  }

}

void SparseTrackerF::pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr)
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

  // **** create rgbd frame **************************************************

  RGBDFrame frame;

  frame.id = keyframes_.size();
  frame.data = *cloud_in_ptr; 
  frame.computeRGBImage();

  // **** extract features *****************************************************

  gettimeofday(&start_features, NULL);

  PointCloudFeature::Ptr features_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  feature_detector_->findFeatures(frame);

  pcl::transformPointCloud (frame.rgb_features , *features_ptr, eigenFromTf(f2b_* b2c_));

  PointCloudT features_vis;
  bool feature_extract_result;

  feature_extract_result = (frame.rgb_features.points.size() >= min_features_);

  pcl::copyPointCloud<PointFeature, PointT>(frame.rgb_features, features_vis);

  //  Publish Features
  features_pub_.publish(features_vis);

  features_ptr->header.frame_id = fixed_frame_;

  gettimeofday(&end_features, NULL);

  // **** ICP ******************************

  gettimeofday(&start_icp, NULL);

  bool icp_result = false;

  if (feature_extract_result && !feature_history_.isEmpty())
  {
    icp_result = ICP(features_ptr);

    if (icp_result)
    {
      tf::Transform corr = tfFromEigen(reg_.getFinalTransformation());
      f2b_ = corr * f2b_;
    }
  }

  if (!icp_result)
    printf("ERROR\n");

  if(!feature_extract_result)
    feature_history_.reset();

  feature_history_.add(*features_ptr);

  gettimeofday(&end_icp, NULL);

  // *** broadcast tf **********************************************************

  broadcastTF(cloud_in_ptr);

  // *** keyframe **************************************************************

  // compute the distance and angle change since last keyframe pose
  double dist, angle;
  getTfDifference(last_f2b_, f2b_, dist, angle);

  if (keyframes_.empty() || dist > kf_dist_eps_ || angle > kf_angle_eps_)
  {
    frame.pose = f2b_* b2c_;
    frame.computeFeatureBBX();

    if(keyframes_.empty())
    {
      frame.path_length_linear  = 0.0;
      frame.path_length_angular = 0.0;
    }
    else
    {
      RGBDFrame& prev_frame =  keyframes_[keyframes_.size() - 1];

      frame.path_length_linear  = prev_frame .path_length_linear  + dist;
      frame.path_length_angular = prev_frame .path_length_angular + angle;

      // create edge to previous frame

      tf::Transform corr = tfFromEigen(reg_.getFinalTransformation());

      Edge e;
      e.index_a = keyframes_.size() - 1;
      e.index_b = keyframes_.size();

      e.consecutive_association = true;

      tf::Transform prev_f2c = keyframes_[e.index_a].pose;

      e.a2b = prev_f2c.inverse() * (f2b_ * b2c_);
      e.tf_computed = true;
      edges_.push_back(e);

      // check for other edges
      for (unsigned int i = 0; i < keyframes_.size() - 1; ++i)
      {
        bool overlap = RGBDFrame::bbxOverlap            (frame, keyframes_[i]) &&
                       //RGBDFrame::totalPathlengthOverlap(frame, keyframes_[i]) &&
                       RGBDFrame::poseDistOverlap       (frame, keyframes_[i]);
        if (overlap)
        {
          Edge e;
          e.index_a = i;
          e.index_b = keyframes_.size();
          edges_.push_back(e);
        }
      }

      //RGBDFrame::testMatching(frame, keyframes_[0], matching_distance_);
    }

    //ROS_WARN ("KEYFRAME %f %f %f, %f",
    //    frame.path_length_linear,
    //    frame.path_length_angular * 180.0 / M_PI);

    pcl::VoxelGrid<PointT> vgf;
    vgf.setFilterFieldName ("z");
    vgf.setFilterLimits (-10, 10);
    vgf.setLeafSize (vgf_features_res_, vgf_features_res_, vgf_features_res_);
    vgf.setInputCloud(cloud_in_ptr);
    vgf.filter(frame.data_downsampled);

    keyframes_.push_back(frame);

    last_f2b_ = f2b_;

    publishFrames();
  }

  // *** counter  **************************************************************

  frame_count_++;

  // **** print diagnostics ****************************************************

  gettimeofday(&end_callback, NULL);

  double features_dur  = msDuration(start_features, end_features);
  double model_dur     = msDuration(start_model,    end_model);
  double icp_dur       = msDuration(start_icp,      end_icp);
  double callback_dur  = msDuration(start_callback, end_callback);

  int model_frames = feature_history_.getSize();
  int icp_iterations = reg_.getFinalIterations();
  int features_count = features_ptr->points.size();

  printf("F[%d] %.1f \t M[%d] %.1f \t ICP[%d] %.1f \t TOTAL %.1f\n",
      features_count,
      features_dur,
      model_frames, model_dur,
      icp_iterations,
      icp_dur, callback_dur);
}

void SparseTrackerF::publishFrames()
{
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.frame_id = fixed_frame_;
  pose_array_msg.header.stamp = ros::Time::now();

  for (unsigned int i = 0; i < keyframes_.size(); ++i)
  {
	  geometry_msgs::Pose p;
	  tf::poseTFToMsg(keyframes_[i].pose * b2c_.inverse(), p);
	  pose_array_msg.poses.push_back(p);
  }

  poses_pub_.publish(pose_array_msg);

  // edges

  visualization_msgs::Marker marker_a;
  visualization_msgs::Marker marker_r;

  marker_a.header.frame_id = fixed_frame_;
  marker_r.header.frame_id = fixed_frame_;

  marker_a.header.stamp = ros::Time::now();
  marker_r.header.stamp = ros::Time::now();

  marker_a.type = visualization_msgs::Marker::LINE_LIST;
  marker_r.type = visualization_msgs::Marker::LINE_LIST;

  marker_a.color.r = 1.0;
  marker_a.color.g = 0.0;
  marker_a.color.b = 0.0;
  marker_a.color.a = 1.0;

  marker_r.color.r = 0.0;
  marker_r.color.g = 1.0;
  marker_r.color.b = 0.0;
  marker_r.color.a = 1.0;

  // Set the scale of the marker
  marker_a.scale.x = 0.002;
  marker_r.scale.x = 0.004;

  // Set the marker action.  Options are ADD and DELETE
  marker_a.action = visualization_msgs::Marker::ADD;
  marker_r.action = visualization_msgs::Marker::ADD;

  marker_a.ns = "edges (goemtric)";
  marker_a.id = 0;
  marker_r.ns = "edges (ransac)";
  marker_r.id = 1;

  marker_a.lifetime = ros::Duration();
  marker_r.lifetime = ros::Duration();

  int edgecount=0;

  for (unsigned int i = 0; i < edges_.size(); ++i)
  {
    Edge& edge = edges_[i];

    RGBDFrame& A = keyframes_[edge.index_a];
    RGBDFrame& B = keyframes_[edge.index_b];

    btTransform f2b_A = A.pose * b2c_.inverse();
    btTransform f2b_B = B.pose * b2c_.inverse();

    geometry_msgs::Point point_A;
    geometry_msgs::Point point_B;

    point_A.x = f2b_A.getOrigin().getX();
    point_A.y = f2b_A.getOrigin().getY();
    point_A.z = f2b_A.getOrigin().getZ();

    point_B.x = f2b_B.getOrigin().getX();
    point_B.y = f2b_B.getOrigin().getY();
    point_B.z = f2b_B.getOrigin().getZ();

    marker_a.points.push_back(point_A);
    marker_a.points.push_back(point_B);

    /*
    if(edges_[i].tf_computed)
    {
      marker_r.points.push_back(point_A);
      marker_r.points.push_back(point_B);

      // predicted b

      btTransform f2b_B_pred = (A.pose * edges_[i].a2b) * b2c_.inverse();
      geometry_msgs::Point point_B_pred;

      point_B_pred.x = f2b_B_pred.getOrigin().getX();
      point_B_pred.y = f2b_B_pred.getOrigin().getY();
      point_B_pred.z = f2b_B_pred.getOrigin().getZ();

      marker_p.points.push_back(point_A);
      marker_p.points.push_back(point_B_pred);
    }
    else
    {
      edgecount++;
      marker_o.points.push_back(point_A);
      marker_o.points.push_back(point_B);
    }
    */

    if (edge.ransac_association)
    {
      marker_r.points.push_back(point_A);
      marker_r.points.push_back(point_B);
    }
  }

  printf("[[[ EDGES: %d ]]]\n", edgecount);

  //edges_pub_.publish(marker_p);
  edges_pub_.publish(marker_a);
  edges_pub_.publish(marker_r);

  // cloud

  //if (keyframes_.size() > 0)
  //  frames_pub_.publish(keyframes_[keyframes_.size() - 1].data);
}

void SparseTrackerF::broadcastTF(const PointCloudT::ConstPtr cloud_in_ptr)
{
  tf::StampedTransform transform_msg(
   f2b_, cloud_in_ptr->header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform (transform_msg);
}

/*
void SparseTrackerF::buildModel(const PointCloudT::ConstPtr cloud_in_ptr,
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

void SparseTrackerF::computeLastEdge()
{
  // only ICP last

  printf("ICP on last edge\n");

  RGBDFrame& a = keyframes_[keyframes_.size() - 1];
  RGBDFrame& b = keyframes_[0];

  // *** GICP

  ccny_gicp::GICPAlign reg_;
  GICPParams params;
  params.max_distance = 1.00;
  params.solve_rotation = true;
  params.max_iteration = 20;
  params.max_iteration_inner = 20;
  params.epsilon = 5e-4;
  params.epsilon_rot = 2e-3;
  params.debug = false;
  reg_.setParams(params);

  PointCloudT::Ptr model_ptr =
    boost::shared_ptr<PointCloudT> (new PointCloudT());
  *model_ptr = a.data;

  pcl::transformPointCloud (*model_ptr , *model_ptr, eigenFromTf(a.pose));

  PointCloudT::Ptr data_ptr =
    boost::shared_ptr<PointCloudT> (new PointCloudT());
  *data_ptr = b.data;

  pcl::transformPointCloud (*data_ptr , *data_ptr, eigenFromTf(b.pose));

  // ****  set up gicp data set ************************************************

  ccny_gicp::GICPPointSetKd gicp_data;
  gicp_data.setNormalCount(20);

  for (unsigned int j = 0; j < data_ptr->points.size(); ++j)
  {
    GICPPoint p;
    p.x = data_ptr->points[j].x;
    p.y = data_ptr->points[j].y;
    p.z = data_ptr->points[j].z;
    p.rgb = data_ptr->points[j].rgb;

    gicp_data.AppendPoint(p);
  }

  // create kd tree
  KdTree gicp_data_kdtree;
  gicp_data_kdtree.setInputCloud(gicp_data.getCloud());

  // compute matrices
  gicp_data.setKdTree(&gicp_data_kdtree);
  gicp_data.SetGICPEpsilon(0.0001);
  gicp_data.computeMatrices();

  reg_.setData(&gicp_data);

  // ****  set up gicp model set ************************************************

  ccny_gicp::GICPPointSetKd gicp_model;
  gicp_model.setNormalCount(20);
  PointCloudGICP d;

  for (unsigned int j = 0; j < model_ptr->points.size(); ++j)
  {
    GICPPoint p;
    p.x = model_ptr->points[j].x;
    p.y = model_ptr->points[j].y;
    p.z = model_ptr->points[j].z;
    p.rgb = model_ptr->points[j].rgb;

    //d.points.push_back(p);
    gicp_model.AppendPoint(p);
  }

  // create kd tree
  KdTree gicp_model_kdtree;
  gicp_model_kdtree.setInputCloud(gicp_model.getCloud());

  // compute matrices
  gicp_model.setKdTree(&gicp_model_kdtree);
  gicp_model.SetGICPEpsilon(0.0001);
  gicp_model.computeMatrices();

  reg_.setModel(&gicp_model);

  Eigen::Matrix4f corr_eigen;
  int iterations = reg_.align(corr_eigen);
  tf::Transform corr = tfFromEigen(corr_eigen);

  tf::Transform f2c_a = a.pose;    // fixed to camera a
  tf::Transform f2c_b = corr * b.pose; // fixed to camera b, as reported by a

  Edge e;
  e.index_a = keyframes_.size() - 1;
  e.index_b = 0;
  e.a2b = f2c_a.inverse() * f2c_b; // b as seen in a's frame
  e.tf_computed = true;
  edges_.push_back(e);

  printf("ICP on last edge done\n");
}

void SparseTrackerF::computeAllEdges()
{
  for (unsigned int i = 0; i < edges_.size(); ++i)
  {
    Edge& edge = edges_[i];

    if (edge.ransac_association || edge.consecutive_association)
    {
      //if (!edges_[i].tf_computed)
      if (1)
      {
        RGBDFrame& a = keyframes_[edges_[i].index_a];
        RGBDFrame& b = keyframes_[edges_[i].index_b];
        printf("ICP on edge %d of %d [%d to %d]\n", i, edges_.size(), a.id, b.id);

        /*
        // icp from frame a (model) to frame b (data)


        PointCloudOrb::Ptr model_ptr =
          boost::shared_ptr<PointCloudOrb> (new PointCloudOrb());
        *model_ptr = a.features;

        pcl::transformPointCloud (*model_ptr , *model_ptr, eigenFromTf(a.pose));

        PointCloudOrb::Ptr data_ptr =
          boost::shared_ptr<PointCloudOrb> (new PointCloudOrb());
        *data_ptr = b.features;

        pcl::transformPointCloud (*data_ptr , *data_ptr, eigenFromTf(b.pose));

        pcl::SampleConsensusInitialAlignment< PointOrb, PointOrb, PointOrb> sac_ia;

        sac_ia.setInputTarget (model_ptr);
        sac_ia.setTargetFeatures (model_ptr);

        sac_ia.setInputCloud (data_ptr);
        sac_ia.setSourceFeatures (data_ptr);

        PointCloudOrb registration_output;
        sac_ia.align (registration_output);


        // **** orb icp ***********************************************************

        // icp from frame a (model) to frame b (data)

        pcl::KdTreeFLANN<PointOrb> tree_data;
        pcl::KdTreeFLANN<PointOrb> tree_model;

        tree_data.setInputCloud(data_ptr);
        tree_model.setInputCloud(model_ptr);

        orb_reg_.setDataCloud  (&*data_ptr);
        orb_reg_.setModelCloud (&*model_ptr);

        orb_reg_.setDataTree  (&tree_data);
        orb_reg_.setModelTree (&tree_model);

        bool result = orb_reg_.align();

        if (result)
        {
          tf::Transform prev_a2b = edges_[i].a2b;

          tf::Transform corr = tfFromEigen(orb_reg_.getFinalTransformation());

          tf::Transform f2c_a = a.pose;    // fixed to camera a
          //tf::Transform f2c_b = corr * f2c_a; // fixed to camera b, as reported by a

          tf::Transform X_f2b_last = f2c_a * b2c_.inverse();;
          tf::Transform X_f2b = corr * X_f2b_last;

          edges_[i].a2b = (X_f2b_last * b2c_).inverse() * (X_f2b * b2c_);

          //edges_[i].a2b = f2c_a.inverse() * f2c_b; // b as seen in a's frame

        tf::Transform corr = tfFromEigen(orb_reg_.getFinalTransformation());
          */

        /*
        // FIXME: coordinate frames?
        if (RGBDFrame::ransacOverlap(a, b))
        {
          //edges_[i].a2b = corr;

          edges_[i].tf_computed = true;

          // test
          double t1, t2;
          getTfDifference(edges_[i].a2b, prev_a2b, t1, t2);

          printf("ERROR IS:  %f, %f\n", t1, t2*180.0 / M_PI);
        }
        */

        // *** GICP

        ccny_gicp::GICPAlign reg_;
        GICPParams params;
        params.max_distance = 1.00;
        params.solve_rotation = true;
        params.max_iteration = 10;
        params.max_iteration_inner = 20;
        params.epsilon = 5e-3;
        params.epsilon_rot = 2e-2;
        params.debug = false;
        reg_.setParams(params);

        PointCloudT::Ptr model_ptr =
          boost::shared_ptr<PointCloudT> (new PointCloudT());
        *model_ptr = a.data_downsampled;

        pcl::transformPointCloud (*model_ptr , *model_ptr, eigenFromTf(a.pose));

        PointCloudT::Ptr data_ptr =
          boost::shared_ptr<PointCloudT> (new PointCloudT());
        *data_ptr = b.data_downsampled;

<<<<<<< HEAD
        pcl::transformPointCloud (*data_ptr , *data_ptr, eigenFromTf(b.pose));

        // ****  set up gicp data set ************************************************

        ccny_gicp::GICPPointSetKd gicp_data;
        gicp_data.setNormalCount(15);

        for (unsigned int j = 0; j < data_ptr->points.size(); ++j)
        {
          GICPPoint p;
          p.x = data_ptr->points[j].x;
          p.y = data_ptr->points[j].y;
          p.z = data_ptr->points[j].z;
          p.rgb = data_ptr->points[j].rgb;
=======
      if (e_dist < 0.5 && e_angle < 20.0 * M_PI/180.0)
      {
        edges_[i].tf_computed = true;
        edges_[i].a2b = new_a2b; // b as seen in a's frame
      }
      else
      {
        printf("\tError TOO BIG\n");
      }
      

    }
  }
}
>>>>>>> d32abce3277f25248f21539b55ec3e8290d983ac

          gicp_data.AppendPoint(p);
        }

        // create kd tree
        KdTree gicp_data_kdtree;
        gicp_data_kdtree.setInputCloud(gicp_data.getCloud());

        // compute matrices
        gicp_data.setKdTree(&gicp_data_kdtree);
        gicp_data.SetGICPEpsilon(0.0001);
        gicp_data.computeMatrices();

        reg_.setData(&gicp_data);

        // ****  set up gicp model set ************************************************

        ccny_gicp::GICPPointSetKd gicp_model;
        gicp_model.setNormalCount(15);

        for (unsigned int j = 0; j < model_ptr->points.size(); ++j)
        {
          GICPPoint p;
          p.x = model_ptr->points[j].x;
          p.y = model_ptr->points[j].y;
          p.z = model_ptr->points[j].z;
          p.rgb = model_ptr->points[j].rgb;

          //d.points.push_back(p);
          gicp_model.AppendPoint(p);
        }

        // create kd tree
        KdTree gicp_model_kdtree;
        gicp_model_kdtree.setInputCloud(gicp_model.getCloud());

        // compute matrices
        gicp_model.setKdTree(&gicp_model_kdtree);
        gicp_model.SetGICPEpsilon(0.0001);
        gicp_model.computeMatrices();

        reg_.setModel(&gicp_model);

        Eigen::Matrix4f corr_eigen;
        tf::Transform corr_temp;
        corr_temp.setIdentity();
        corr_eigen = eigenFromTf(corr_temp);

        int iterations = reg_.align(corr_eigen);
        tf::Transform corr = tfFromEigen(corr_eigen);

        tf::Transform f2c_a = a.pose;    // fixed to camera a
        tf::Transform f2c_b = corr * b.pose; // fixed to camera b, as reported by a

        tf::Transform new_a2b = f2c_a.inverse() * f2c_b;

        // test
        double e_dist, e_angle;

        getTfDifference(f2c_b, b.pose, e_dist, e_angle);
        //printf("ERROR IS:  %f, %f\n", e_dist, e_angle*180.0 / M_PI);

        if (e_dist < 0.5 && e_angle < 20.0 * M_PI/180.0)
        {
          edges_[i].tf_computed = true;
          edges_[i].a2b = new_a2b; // b as seen in a's frame
        }
        else
        {
          printf("\tError TOO BIG\n");
        }


      }
    }
  }
}

bool SparseTrackerF::getBaseToCameraTf(const PointCloudT::ConstPtr cloud_in_ptr)
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

bool SparseTrackerF::ICP(PointCloudFeature::Ptr& features_ptr)
{
  // **** build orb model *****************************************************

  PointCloudFeature::Ptr model_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  model_ptr->header.frame_id = fixed_frame_;
  feature_history_.getAll(*model_ptr);

  PointCloudT model_vis;
  pcl::copyPointCloud<PointFeature, PointT>(*model_ptr, model_vis);
  model_pub_.publish(model_vis);

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

  return result;
}

bool SparseTrackerF::loopSrvCallback(ccny_rgbd::Loop::Request& request,
                                     ccny_rgbd::Loop::Response& response)
{
  boost::mutex::scoped_lock(mutex_);

  ROS_WARN("LOOP CLOSE CALLED");

  struct timeval start, end;

  gettimeofday(&start, NULL);

  computeEdgeAssociationsRansac();

  computeAllEdges();
  //computeLastEdge();
  loopClose();

  publishFrames();

  PointCloudT cloud;
  cloud.header.frame_id = fixed_frame_;
  cloud.header.stamp = ros::Time::now();

  for (int i = 0; i < keyframes_.size(); ++i)
  {
    PointCloudT t;
    pcl::transformPointCloud(keyframes_[i].data, t, eigenFromTf(keyframes_[i].pose));
    t.header.frame_id = fixed_frame_;
    //cloud += t;

    all_frames_pub_.publish(t);
  }

  gettimeofday(&end, NULL);
  double callback_dur  = msDuration(start, end);
  ROS_INFO("LOOP: %f", callback_dur);

  return true;
}

void SparseTrackerF::computeEdgeAssociationsRansac()
{
  for (unsigned int i = 0; i < edges_.size(); ++i)
  {
    Edge& edge = edges_[i];

    if (edge.consecutive_association) continue; // skip consecutive edges

    RGBDFrame& A = keyframes_[edge.index_a];
    RGBDFrame& B = keyframes_[edge.index_b];

    printf("RANSAC association for edge %d of %d\n:", i, edges_.size());
    bool ransac_association =
     RGBDFrame::ransacMatchingOverlap(A, B, ransac_matching_distance_,
                                      ransac_eps_reproj_,
                                      ransac_inlier_threshold_,
                                      ransac_max_iterations_);

    edge.ransac_association = ransac_association;

    publishFrames();
  }
}

} //namespace ccny_rgbd
