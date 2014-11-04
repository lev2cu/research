#include "ccny_rgbd/registration/dense_tracker.h"

namespace ccny_rgbd {

DenseTracker::DenseTracker(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO("Starting Dense Tracker");

  // **** initialize parameters

  initParams();

  // **** initial state

  cloud_count_ = 0;
  world_to_odom_.setIdentity();

  model_ = boost::shared_ptr<PointCloudGICP> (new PointCloudGICP());

  model_->header.frame_id = world_frame_;

  octree_ = new Octree(octree_resolution_);
  octree_->setInputCloud(model_);

  gicp_model_octree_ = new ccny_gicp::GICPPointSetOctree();
  gicp_model_kdtree_ = new ccny_gicp::GICPPointSetKd();
  gicp_model_octree_->setCloud(model_);
  gicp_model_octree_->setOctree(octree_);

   // **** publishers

  scene_pub_ = nh_.advertise<PointCloudGICP>(
    scene_topic_, 1);

  // **** subscribers

  point_cloud_subscriber_ = nh_.subscribe<PointCloudT>(
    sub_topic_, 1, &DenseTracker::pointCloudCallback, this);
}

DenseTracker::~DenseTracker()
{
  ROS_INFO("Destroying Octree Slam"); 
}

void DenseTracker::initParams()
{
  if (!nh_private_.getParam ("use_additive_model", use_additive_model_))
    use_additive_model_ = false;

  if (!nh_private_.getParam ("fixed_frame", world_frame_))
    world_frame_ = "world";
  if (!nh_private_.getParam ("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  if (!nh_private_.getParam ("use_vgf_data_filter", use_vgf_data_filter_))
    use_vgf_data_filter_ = false;
  if (!nh_private_.getParam ("vgf_data_res", vgf_data_res_))
    vgf_data_res_ = 0.01;
  if (!nh_private_.getParam ("vgf_data_range", vgf_data_range_))
    vgf_data_range_ = 3.5;

  if (!nh_private_.getParam ("octree_resolution", octree_resolution_))
    octree_resolution_ = 0.05;

  // **** gicp params

  if (!nh_private_.getParam ("gicp_epsilon", gicp_epsilon_))
     gicp_epsilon_ = 0.004;

  if (!nh_private_.getParam ("nn_count", nn_count_))
    nn_count_ = 20;

  bool use_color;
  if (!nh_private_.getParam ("use_color", use_color))
    use_color = false;
  reg_.setUseColor(use_color);

  double max_color_diff;
  if (!nh_private_.getParam ("max_color_diff", max_color_diff))
    max_color_diff = 50;
  reg_.setMaxColorDiff(max_color_diff);

  GICPParams params;

  if (!nh_private_.getParam ("max_distance", params.max_distance))
     params.max_distance = 0.20;
  if (!nh_private_.getParam ("solve_rotation", params.solve_rotation))
     params.solve_rotation = true;
  if (!nh_private_.getParam ("max_iterations", params.max_iteration))
     params.max_iteration = 10;
  if (!nh_private_.getParam ("max_iteration_inner", params.max_iteration_inner))
     params.max_iteration_inner = 20;
  if (!nh_private_.getParam ("epsilon", params.epsilon))
     params.epsilon = 5e-4;
  if (!nh_private_.getParam ("epsilon_rot", params.epsilon_rot))
     params.epsilon_rot = 2e-3;

  params.debug = false;

  reg_.setParams(params);
}

bool DenseTracker::getOdomToCameraTf(const PointCloudT::ConstPtr& cloud_in_ptr, tf::Transform& o2k)
{
  tf::StampedTransform o2k_tf;

  try
  {
    tf_listener_.waitForTransform (
      odom_frame_, cloud_in_ptr->header.frame_id, cloud_in_ptr->header.stamp, ros::Duration(0.5));
    tf_listener_.lookupTransform (
      odom_frame_, cloud_in_ptr->header.frame_id, cloud_in_ptr->header.stamp, o2k_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("TF unavailable %s", ex.what());
    return false;
  }

  o2k = o2k_tf;
  return true;
}

void DenseTracker::filterCloud(const PointCloudT::ConstPtr& cloud_in_ptr,
                             const PointCloudT::Ptr& data_ptr)
{

  if (use_vgf_data_filter_)
  {
    // downsample using a voxel grid filer
    // and limit the depth

    pcl::VoxelGrid<PointT> vgf1; //TODO make member

    vgf1.setLeafSize (vgf_data_res_, vgf_data_res_, vgf_data_res_);
    vgf1.setFilterFieldName ("z");
    vgf1.setFilterLimits (0, vgf_data_range_);
    vgf1.setInputCloud (cloud_in_ptr);
    vgf1.filter (*data_ptr);
  }
  else
  {
    // don't downsample, but still remove nan's

    for (unsigned int i = 0; i < cloud_in_ptr->points.size(); ++i)
    {
      if (!isnan(cloud_in_ptr->points[i].z))
        data_ptr->points.push_back(cloud_in_ptr->points[i]);
    }

    data_ptr->is_dense = true;
    data_ptr->width  = data_ptr->points.size();
    data_ptr->height = 1;
    data_ptr->header = cloud_in_ptr->header;
  }
}

void DenseTracker::pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr)
{
  struct timeval start_, end_;    // used for timing
  struct timeval start, end;    // used for timing

  gettimeofday(&start_, NULL);

  // **** get odom to kinect tf ************************************************

  gettimeofday(&start, NULL);

  tf::Transform o2k;
  bool r = getOdomToCameraTf(cloud_in_ptr, o2k);
  if (!r) return;

  gettimeofday(&end, NULL);
  printf("TF %.1f\t", msDuration(start, end));

  // **** create data cloud  ***************************************************

  gettimeofday(&start, NULL);

  PointCloudT::Ptr data_ptr =
    boost::shared_ptr<PointCloudT> (new PointCloudT());

  filterCloud(cloud_in_ptr, data_ptr);

  gettimeofday(&end, NULL);
  printf("F %.1f\t", msDuration(start, end));

  // **** transform to best guess for world frame ******************************

  gettimeofday(&start, NULL);

  pcl_ros::transformPointCloud (*data_ptr, *data_ptr, world_to_odom_ * o2k);
  data_ptr->header.frame_id = world_frame_;

  gettimeofday(&end, NULL);
  printf("TF2 %.1f\t", msDuration(start, end));

  // ****  set up gicp data set ************************************************

  gettimeofday(&start, NULL);

  ccny_gicp::GICPPointSetKd gicp_data;
  gicp_data.setNormalCount(nn_count_);
  PointCloudGICP d;

  for (unsigned int i = 0; i < data_ptr->points.size(); ++i)
  {
    PointGICP p;
    p.x = data_ptr->points[i].x;
    p.y = data_ptr->points[i].y;
    p.z = data_ptr->points[i].z;
    p.rgb = data_ptr->points[i].rgb;

    //d.points.push_back(p);
    gicp_data.AppendPoint(p);
  }

  //gicp_data.setCloud(d.makeShared());

  // create kd tree
  KdTree gicp_data_kdtree;
  gicp_data_kdtree.setInputCloud(gicp_data.getCloud());

  // compute matrices
  gicp_data.setKdTree(&gicp_data_kdtree);
  gicp_data.SetGICPEpsilon(gicp_epsilon_);

  gicp_data.computeMatrices();

  reg_.setData(&gicp_data);

  gettimeofday(&end, NULL);
  printf("D %.1f\t", msDuration(start, end));

  // **** icp ******************************************************************

  gettimeofday(&start, NULL);

  tf::Transform corr;
  Eigen::Matrix4f corr_eigen;

  if (cloud_count_ > 0)
  {
    KdTree kdtree;

    if (use_additive_model_)
    {
      reg_.setModel(gicp_model_octree_);
    }
    else
    {
      gicp_model_kdtree_->setCloud(model_);

      // create kd tree
      kdtree.setInputCloud(model_);
      gicp_model_kdtree_->setKdTree(&kdtree);
      gicp_model_kdtree_->SetGICPEpsilon(gicp_epsilon_);

      reg_.setModel(gicp_model_kdtree_);
    }

    reg_.align(corr_eigen);
    corr = tfFromEigen(corr_eigen);
  }
  else
  {
    corr.setIdentity();
  }

  gettimeofday(&end, NULL);
  printf("ICP[%d] %.1f\t", reg_.getFinalIterations(), msDuration(start, end));

  // **** add to scene *********************************************************

  gettimeofday(&start, NULL);

  if (use_additive_model_)
  {
    //printf("starting\n");
    pcl::transformPointCloud (*data_ptr, *data_ptr, corr_eigen);

    for (unsigned int i = 0; i < data_ptr->points.size(); ++i)
    {
      PointGICP p1;
      p1.x = data_ptr->points[i].x;
      p1.y = data_ptr->points[i].y;
      p1.z = data_ptr->points[i].z;
      p1.rgb = data_ptr->points[i].rgb;

      PointGICP p2;
      p2.x = gicp_data.getCloud()->points[i].x;
      p2.y = gicp_data.getCloud()->points[i].y;
      p2.z = gicp_data.getCloud()->points[i].z;
      p2.rgb = gicp_data.getCloud()->points[i].rgb;

      int index;

      bool new_point = octree_->addPointWithReplacement(p1, model_, index);

      if (cloud_count_ > 0)
      {
        if (new_point)
        {
          memcpy((*gicp_model_octree_)[index].C, gicp_data[i].C, sizeof (double) * 9);
          //gicp_model_octree_->computeMatrix(index);
        }
        else
        {
          memcpy((*gicp_model_octree_)[index].C, gicp_data[i].C, sizeof (double) * 9);
          //gicp_model_octree_->computeMatrix(index);
        }
      }
    }

    if (cloud_count_ == 0)
    {
      gicp_model_octree_->computeMatrices();
    }
  }
  else
  {
    // replace model with current data

    // FIXME
    pcl_ros::transformPointCloud (*data_ptr, *data_ptr, corr);

    model_->points.clear();

    for (unsigned int i = 0; i < data_ptr->points.size(); ++i)
    {
      PointGICP p1;
      p1.x = data_ptr->points[i].x;
      p1.y = data_ptr->points[i].y;
      p1.z = data_ptr->points[i].z;
      p1.rgb = data_ptr->points[i].rgb;
      memcpy(p1.C, gicp_data[i].C, sizeof (double) * 9);

      /*
      GICPPoint p2;
      p2.x   = gicp_data.getCloud()->points[i].x;
      p2.y   = gicp_data.getCloud()->points[i].y;
      p2.z   = gicp_data.getCloud()->points[i].z;
      p2.rgb = gicp_data.getCloud()->points[i].rgb;
      memcpy(p2.C, gicp_data[i].C, sizeof (double) * 9);


       DOESN'T WORK? slightly different results after applying translation
      if ( p1.x !=  p2.x || p1.y !=  p2.y || p1.z !=  p2.z)
      {
        printf("\t[%d] (%f %f %f)\t(%f %f %f)\n",
            i, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
      }
       */

      model_->points.push_back(p1); //p1 works, p2 doesn't
    }
  }

  model_->header.stamp = cloud_in_ptr->header.stamp;

  gettimeofday(&end, NULL);

  int num_model_points;

  if (use_additive_model_)
    num_model_points = gicp_model_octree_->NumPoints();
  else
    num_model_points = gicp_model_kdtree_->NumPoints();

  printf("[%d] M: %.1f\t", num_model_points, msDuration(start, end));

  // *** visualize *********************************************************

  //gettimeofday(&start, NULL);

  //scene_pub_.publish(model_);

  // **** publish *********************************************************

  world_to_odom_ =  corr * world_to_odom_;// corr;

  tf::StampedTransform world_to_odom_tf_(
    world_to_odom_, cloud_in_ptr->header.stamp, world_frame_, odom_frame_);
  tf_broadcaster_.sendTransform (world_to_odom_tf_);

  cloud_count_++;

  //gettimeofday(&end, NULL);
  //printf("Pub: %.1f\t", msDuration(start, end));

  gettimeofday(&end_, NULL);
  printf("ALL: %.1f\t", msDuration(start_, end_));
  printf("\n");
}

} //namespace
