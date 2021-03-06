#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

RGBDFrame::RGBDFrame():
  keypoints_computed_(false),
  descriptors_computed_(false),
  bbx_computed_(false),
  rgb_image_computed_(false),
  depth_image_computed_(false),
  u_depth_image_computed_(false)
{

}

cv::Mat * RGBDFrame::getRGBImage()
{
  assert(rgb_image_computed_);
  return &rgb_img_;
}

cv::Mat * RGBDFrame::getDepthImage()
{
  assert(depth_image_computed_);
  return &depth_img_;
}

cv::Mat * RGBDFrame::getUDepthImage()
{
  assert(u_depth_image_computed_);
  return &u_depth_img_;
}

void RGBDFrame::computeRGBImage()
{
  sensor_msgs::Image image;
  cv_bridge::CvImagePtr cv_ptr_rgb;

  try
  {
    pcl::toROSMsg (data, image); //convert the cloud
  }
  catch (std::runtime_error e)
  {
    ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
  }
  cv_ptr_rgb = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

  rgb_img_ = cv::Mat (cv_ptr_rgb->image.rows, cv_ptr_rgb->image.cols, CV_8UC1);
  cvtColor(cv_ptr_rgb->image, rgb_img_, CV_BGR2GRAY);

  rgb_image_computed_ = true;
}

void RGBDFrame::computeDepthImage()
{
  // **** build raw image from point cloud

  depth_img_ = cv::Mat(data.height, data.width, CV_32FC1);

  for(unsigned int j = 0; j < data.height; ++j)
  for(unsigned int i = 0; i < data.width;  ++i)
  {
    unsigned int index = j * data.width + i;
    float z = data.points[index].z;
    depth_img_.at<float>(j,i) = z;
  }

  depth_image_computed_ = true;
}

void RGBDFrame::computeUDepthImage()
{
  // **** build raw image from point cloud

  u_depth_img_ = cv::Mat(data.height, data.width, CV_8UC1);

  for(unsigned int j = 0; j < data.height; ++j)
  for(unsigned int i = 0; i < data.width;  ++i)
  {
    unsigned int index = j * data.width + i;
    float z = data.points[index].z;
    if(isnan(z))
      u_depth_img_.at<uint8_t>(j,i) = 0;
    else
    {
      uint8_t uz = (uint8_t)std::min((z / 5.0) * 255.0, 255.0);
      u_depth_img_.at<uint8_t>(j,i) = uz;
    }
  }

  u_depth_image_computed_ = true;
}

void RGBDFrame::computeFeatureBBX()
{
  // needs to be recomputed if pose is changed

  // FIXME: depth/rgb features
  assert(!rgb_features.empty());

  PointCloudFeature transformed_features;

  pcl::transformPointCloud(rgb_features, transformed_features, eigenFromTf(pose));

  min_x = max_x = transformed_features.points[0].x;
  min_y = max_y = transformed_features.points[0].y;
  min_z = max_z = transformed_features.points[0].z;

  for(unsigned int i = 1; i < transformed_features.points.size(); i++)
  {
    PointFeature& p = transformed_features.points[i];

    double x = p.x;
    double y = p.y;
    double z = p.z;

    if      (x < min_x) min_x = x;
    else if (x > max_x) max_x = x;

    if      (y < min_y) min_y = y;
    else if (y > max_y) max_y = y;

    if      (z < min_z) min_z = z;
    else if (z > max_z) max_z = z;
  }

  bbx_computed_ = true;

  //printf("X: [%f  %f] \t Y:[%f  %f] \t Z:[%f  %f]\n", 
  //       min_x, max_x, min_y, max_y, min_z, max_z);
}

bool RGBDFrame::poseDistOverlap(const RGBDFrame& a, const RGBDFrame& b)
{
  double max_dist_diff = 2.00;
  double max_angle_diff = 45.0 * M_PI / 180.0;

  bool result = true;

  double dist, angle;
  getTfDifference(a.pose, b.pose, dist, angle);

  if (angle > max_angle_diff) result = false;
  if (dist > max_dist_diff)   result = false;

  return result;
}

bool RGBDFrame::totalPathlengthOverlap(const RGBDFrame& a, const RGBDFrame& b)
{
  bool result = true;

  double l_diff = std::abs(a.path_length_linear  - b.path_length_linear);
  double a_diff = std::abs(a.path_length_angular - b.path_length_angular);

  if (l_diff < 1.0 && a_diff < (45.0 * M_PI / 180.0)) result = false;

  return result;
}

bool RGBDFrame::bbxOverlap(const RGBDFrame& a, const RGBDFrame& b)
{
  assert(a.bbx_computed_ && b.bbx_computed_);

  bool result = true;

  if (a.min_x >= b.max_x) result = false;
  if (a.max_x <= b.min_x) result = false;

  if (a.min_y >= b.max_y) result = false;
  if (a.max_y <= b.min_y) result = false;

  if (a.min_z >= b.max_z) result = false;
  if (a.max_z <= b.min_z) result = false;

  return result;
}

/*
bool RGBDFrame::ransacDataOverlap(const RGBDFrame& a, const RGBDFrame& b)
{
   double ransac_inlier_threshold_ = 0.10;
   int ransac_max_iterations_ = 100;
   double inliers_fraction_threshold_ = 0.50;

   // a = source (data)
   // b = target (model)

   int N = a.rgb_features.points.size();    // the size of the data cloud

   std::vector<int> source_indices;
   source_indices.resize(N);
   std::vector<int> target_indices;
   target_indices.resize(N);

   PointCloudFeature::Ptr source_ptr = boost::shared_ptr<PointCloudFeature>();
   source_ptr.reset(new PointCloudFeature(a.rgb_features));;
   pcl::transformPointCloud(*source_ptr, *source_ptr, eigenFromTf(a.pose));

   PointCloudFeature::Ptr target_ptr = boost::shared_ptr<PointCloudFeature>();
   target_ptr.reset(new PointCloudFeature(b.rgb_features));
   pcl::transformPointCloud(*target_ptr, *target_ptr, eigenFromTf(b.pose));

   pcl::KdTreeFLANN<PointFeature> tree_model;
   tree_model.setInputCloud(target_ptr);

   std::vector<int>   nn_indices  (1); // for nn search
   std::vector<float> nn_dists_sq (1); // for nn search

   int corr_cnt = 0;

   for (int idx = 0; idx < N; ++idx)
   {
	 PointFeature& p = source_ptr->points[idx];

     int nn_retrieved = tree_model.nearestKSearch(
       p, 1, nn_indices, nn_dists_sq);

     if (nn_retrieved != 0)// && nn_dists_sq[0] < 0.15)
     {
       source_indices[corr_cnt] = idx;
       target_indices[corr_cnt] = nn_indices[0];
       corr_cnt++;
     }
   }

   // Resize to the actual number of valid correspondences
   source_indices.resize(corr_cnt);
   target_indices.resize(corr_cnt);

   pcl::SampleConsensusModelRegistration<PointFeature>::Ptr sac_model;
   sac_model.reset (new pcl::SampleConsensusModelRegistration<PointFeature> (source_ptr, source_indices) );

   // Pass the target_indices
   sac_model->setInputTarget (target_ptr, target_indices);

   pcl::RandomSampleConsensus<PointFeature> sac(sac_model, ransac_inlier_threshold_);
   sac.setMaxIterations (ransac_max_iterations_);

   bool result = sac.computeModel();

   // Compute the set of inliers

   int n;

   if (result)
   {
     std::vector<int> inliers;
     sac.getInliers (inliers);
     n = inliers.size();
   }
   else
   {
     n = 0;
   }

   double inliers_fraction = (double)n / (double)N;

   printf("{{ %d %d %d %f}}\n", a.rgb_features.size(), b.rgb_features.size(), n,  inliers_fraction);

   return (inliers_fraction > inliers_fraction_threshold_);

}

bool RGBDFrame::ransacOverlap(const RGBDFrame& a, const RGBDFrame& b)
{
  double ransac_inlier_threshold_ = 0.15;
  int ransac_max_iterations_ = 1000;
  double inliers_fraction_threshold_ = 0.33;
  double ransac_probability = 0.5;

  // a = source (data)
  // b = target (model)

  int N = a.rgb_features.points.size();    // the size of the data cloud

  std::vector<int> source_indices;
  source_indices.resize(N);
  std::vector<int> target_indices;
  target_indices.resize(N);

  PointCloudFeature::Ptr source_ptr = boost::shared_ptr<PointCloudFeature>();
  source_ptr.reset(new PointCloudFeature(a.rgb_features));;
  pcl::transformPointCloud(*source_ptr, *source_ptr, eigenFromTf(a.pose));

  PointCloudFeature::Ptr target_ptr = boost::shared_ptr<PointCloudFeature>();
  target_ptr.reset(new PointCloudFeature(b.rgb_features));
  pcl::transformPointCloud(*target_ptr, *target_ptr, eigenFromTf(b.pose));

  pcl::KdTreeFLANN<PointFeature> tree_model;
  tree_model.setInputCloud(target_ptr);

  std::vector<int>   nn_indices  (1); // for nn search
  std::vector<float> nn_dists_sq (1); // for nn search

  int corr_cnt = 0;

  for (int idx = 0; idx < N; ++idx)
  {
    PointFeature& p = source_ptr->points[idx];

    int nn_retrieved = tree_model.nearestKSearch(
      p, 1, nn_indices, nn_dists_sq);

    if (nn_retrieved != 0)// && nn_dists_sq[0] < 0.15)
    {
      source_indices[corr_cnt] = idx;
      target_indices[corr_cnt] = nn_indices[0];
      corr_cnt++;
    }
  }

  // Resize to the actual number of valid correspondences
  source_indices.resize(corr_cnt);
  target_indices.resize(corr_cnt);

  pcl::SampleConsensusModelRegistration<PointFeature>::Ptr sac_model;
  sac_model.reset (new pcl::SampleConsensusModelRegistration<PointFeature> (source_ptr, source_indices) );

  // Pass the target_indices
  sac_model->setInputTarget (target_ptr, target_indices);

  pcl::RandomSampleConsensus<PointFeature> sac(sac_model, ransac_inlier_threshold_);
  sac.setMaxIterations (ransac_max_iterations_);
  sac.setProbability(ransac_probability);

  bool result = sac.computeModel();

  // Compute the set of inliers

  int n;

  if (result)
  {
    std::vector<int> inliers;
    sac.getInliers (inliers);
    n = inliers.size();
  }
  else
  {
    n = 0;
  }

  double inliers_fraction = (double)n / (double)N;

  printf("{{ %d %d %d %f}}\n", a.rgb_features.size(), b.rgb_features.size(), n,  inliers_fraction);

  return (inliers_fraction > inliers_fraction_threshold_);
}
*/

bool RGBDFrame::ransacMatchingOverlap(
  RGBDFrame& frame_src, RGBDFrame& frame_dst, 
  tf::Transform& transform, float matching_distance, 
  float eps_reproj, float inlier_threshold,
  PointCloudT::Ptr cloud_src, 
  PointCloudT::Ptr cloud_dst)
{
  // **** params

  bool use_icp_refinement_sparse = true;

  // **** if needed, detect keypoints
    
  if (!frame_src.keypoints_computed_)
  {
    cv::SurfFeatureDetector detector;
    detector.detect(frame_src.rgb_img_, frame_src.keypoints);
    frame_src.keypoints_computed_ = true;
  }
  if (!frame_dst.keypoints_computed_)
  {
    cv::SurfFeatureDetector detector;
    detector.detect(frame_dst.rgb_img_, frame_dst.keypoints);
    frame_dst.keypoints_computed_ = true;
  }

  // **** if needed, extract descriptors

  if (!frame_src.descriptors_computed_)
  {
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(frame_src.rgb_img_, frame_src.keypoints, frame_src.descriptors);
    frame_src.descriptors_computed_ = true;
  }
  if (!frame_dst.descriptors_computed_)
  {
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(frame_dst.rgb_img_, frame_dst.keypoints, frame_dst.descriptors);
    frame_dst.descriptors_computed_ = true;
  }

  // **** match the descriptors
  // TODO: try 
  cv::FlannBasedMatcher matcher;
  //cv::BruteForceMatcher<cv::L2<float> > matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(frame_src.descriptors, frame_dst.descriptors, matches);

  std::vector<cv::DMatch> good_matches;

  for (unsigned int i = 0; i < matches.size(); ++i)
  {
    if (matches[i].distance < matching_distance)
      good_matches.push_back(matches[i]);
  }

  // **** create vectors of feature points from correspondences

  std::vector<cv::Point2f> h_src_pts;
  std::vector<cv::Point2f> h_dst_pts;

  for(unsigned int i = 0; i < good_matches.size(); ++i)
  {
    int q_idx = good_matches[i].queryIdx;
    int t_idx = good_matches[i].trainIdx; 

    h_src_pts.push_back(frame_src.keypoints[q_idx].pt);
    h_dst_pts.push_back(frame_dst.keypoints[t_idx].pt);
  }

  // **** Find inliers using ransac

  bool ransac_overlap;
  double inlier_ratio;
  std::vector<cv::DMatch> inlier_matches;

  cv::Mat status;
  cv::Mat h = cv::findHomography(h_src_pts, h_dst_pts, CV_RANSAC, eps_reproj, status);

  for (unsigned int m = 0; m < good_matches.size(); m++) 
  { 
    if (status.at<char>(m, 0) == 1)
      inlier_matches.push_back(good_matches[m]);
  }

  // drawing the results
/*
  cv::namedWindow("matches", CV_WINDOW_NORMAL);
  cv::Mat img_matches;
  cv::drawMatches(frame_src.rgb_img_, keypoints_src, 
                  frame_dst.rgb_img_, keypoints_dst, good_matches, img_matches);
  cv::imshow("matches", img_matches);
  cv::waitKey(50);
*/

  // **** Check if ratio of inliers is high enough

  inlier_ratio = (double)inlier_matches.size() / (double)good_matches.size();
  ransac_overlap = (inlier_ratio > inlier_threshold);

  // ***** compute rigid transformation from inliers

  if (ransac_overlap)
  {
    // display & save
    cv::namedWindow("inlier matches", CV_WINDOW_NORMAL);
    cv::Mat img_inlier_matches;
    cv::drawMatches(frame_src.rgb_img_, frame_src.keypoints, 
                    frame_dst.rgb_img_, frame_dst.keypoints, 
                    inlier_matches, img_inlier_matches);
    //cv::imshow("inlier matches", img_inlier_matches);
    //cv::waitKey(50);

    std::stringstream ss;
    ss << frame_src.id << "_to_" << frame_dst.id;
    cv::imwrite("/home/idryanov/ros/images/" + ss.str() + ".png", img_inlier_matches);

    // create 3D point clouds
    constructClouds(inlier_matches, frame_src, frame_dst, cloud_src, cloud_dst);

    // @TODO - needed?  
    pcl::transformPointCloud(*cloud_src, *cloud_src, eigenFromTf(frame_src.pose));
    pcl::transformPointCloud(*cloud_dst, *cloud_dst, eigenFromTf(frame_dst.pose));
  
    // estimate using simple svd
    Eigen::Matrix4f transform_eigen;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
    svd.estimateRigidTransformation(*cloud_src, *cloud_dst, transform_eigen);
    transform = tfFromEigen(transform_eigen);

    // refine estimate using icp
    if (use_icp_refinement_sparse)
    {
      pcl::transformPointCloud(*cloud_src, *cloud_src, eigenFromTf(transform));

      ccny_rgbd::ICPKd<PointT, PointT> reg;
    
      reg.setMaxIterations(20);
      reg.setTransformationEpsilon(0.0000001);
      reg.setMaxCorrDist(0.15);
      reg.setUseValueRejection(false);
      reg.setUseRANSACRejection(true);
      reg.setRANSACThreshold(0.15);

      pcl::KdTreeFLANN<PointT> tree_data;
      pcl::KdTreeFLANN<PointT> tree_model;

      tree_data.setInputCloud(cloud_src);
      tree_model.setInputCloud(cloud_dst);

      reg.setDataCloud  (&*cloud_src);
      reg.setModelCloud (&*cloud_dst);

      reg.setDataTree  (&tree_data);
      reg.setModelTree (&tree_model);

      reg.align();
      Eigen::Matrix4f icp_corr_eigen = reg.getFinalTransformation();
      tf::Transform icp_corr = tfFromEigen(icp_corr_eigen);

      transform = icp_corr * transform;
    }
  }

  return ransac_overlap;
}

bool RGBDFrame::computeTfGICP(
  RGBDFrame& frame_src, RGBDFrame& frame_dst, 
  tf::Transform& transform)
{
  // **** params

  double vgf_res = 0.02;
  double gicp_epsilon = 0.001;
  int gicp_normal_count = 20;

  // **** create GICP aligner
  // TODO: outside

  ccny_gicp::GICPAlign aligner;
  aligner.setUseColor(false);
  GICPParams params;
  params.max_distance = 0.30;
  params.solve_rotation = true;
  params.max_iteration = 20;
  params.max_iteration_inner = 20;
  params.epsilon = 5e-4;
  params.epsilon_rot = 2e-3;
  params.debug = false;
  aligner.setParams(params);

  // **** create voxel grid filter
  // TODO: outside

  pcl::VoxelGrid<PointT> vgf;
  vgf.setFilterFieldName ("z");
  vgf.setFilterLimits (0, 5); // TODO
  vgf.setLeafSize (vgf_res, vgf_res, vgf_res);

  // **** create clouds

  PointCloudT::Ptr model_ptr = boost::shared_ptr<PointCloudT> (new PointCloudT());
  PointCloudT::Ptr data_ptr  = boost::shared_ptr<PointCloudT> (new PointCloudT());

  *model_ptr = frame_dst.data;
  *data_ptr  = frame_src.data;

  // **** downsample 

  vgf.setInputCloud(model_ptr);
  vgf.filter(*model_ptr);
  
  vgf.setInputCloud(data_ptr);
  vgf.filter(*data_ptr);

  // **** transform to word frame

  pcl::transformPointCloud (*model_ptr, *model_ptr, eigenFromTf(frame_dst.pose));
  pcl::transformPointCloud (*data_ptr , *data_ptr,  eigenFromTf(frame_src.pose)); // TODO
  pcl::transformPointCloud (*data_ptr , *data_ptr,  eigenFromTf(transform));

  // **** prepare data

  ccny_gicp::GICPPointSetKd p1;

  for (unsigned int j = 0; j < data_ptr->points.size(); ++j)
  {
    PointGICP p;
    p.x = data_ptr->points[j].x;
    p.y = data_ptr->points[j].y;
    p.z = data_ptr->points[j].z;
    p.rgb = data_ptr->points[j].rgb;

    if (!isnan(p.z)) p1.AppendPoint(p);
  }

  KdTree data_kdtree;
  data_kdtree.setInputCloud(p1.getCloud());
  p1.setNormalCount(gicp_normal_count);
  p1.setKdTree(&data_kdtree);
  p1.SetGICPEpsilon(gicp_epsilon);
  p1.computeMatrices();
  aligner.setData(&p1);

  // **** prepare model

  ccny_gicp::GICPPointSetKd p2;

  for (unsigned int j = 0; j < model_ptr->points.size(); ++j)
  {
    PointGICP p;
    p.x = data_ptr->points[j].x;
    p.y = data_ptr->points[j].y;
    p.z = data_ptr->points[j].z;
    p.rgb = data_ptr->points[j].rgb;

    if (!isnan(p.z)) p2.AppendPoint(p);
  }

  KdTree model_kdtree;
  model_kdtree.setInputCloud(p2.getCloud());
  p2.setKdTree(&model_kdtree);
  p2.SetGICPEpsilon(gicp_epsilon);
  p2.setNormalCount(gicp_normal_count);
  p2.computeMatrices();
  aligner.setModel(&p2);

  // **** align

  Eigen::Matrix4f transform_eigen;
  aligner.align(transform_eigen);
  transform = tfFromEigen(transform_eigen);

  return true;
}

void RGBDFrame::constructClouds(
  const std::vector<cv::DMatch>& inlier_matches,
  const RGBDFrame& frame_src, 
  const RGBDFrame& frame_dst, 
  PointCloudT::Ptr& cloud_src,
  PointCloudT::Ptr& cloud_dst)
{
  // **** params
  
  double max_dist = 15.0;

  int cols = frame_src.rgb_img_.cols;

  // create 3D point clouds
  for (unsigned int i = 0; i < inlier_matches.size(); ++i)
  {
    int q_idx = inlier_matches[i].queryIdx;
    int t_idx = inlier_matches[i].trainIdx;

    const cv::KeyPoint& keypoint_src = frame_src.keypoints[q_idx];
    const cv::KeyPoint& keypoint_dst = frame_dst.keypoints[t_idx];

    int index_src_x = (int)keypoint_src.pt.x;
    int index_src_y = (int)keypoint_src.pt.y;
    int index_dst_x = (int)keypoint_dst.pt.x;
    int index_dst_y = (int)keypoint_dst.pt.y;

    unsigned int index_src = index_src_y * cols + index_src_x;
    unsigned int index_dst = index_dst_y * cols + index_dst_x;

    PointT point_src = frame_src.data[index_src];
    PointT point_dst = frame_dst.data[index_dst];

    // TODO - max range as param?
    if (!isnan(point_src.z) && point_src.z < max_dist &&
        !isnan(point_dst.z) && point_dst.z < max_dist)
    {
      cloud_src->push_back(point_src);
      cloud_dst->push_back(point_dst);
    }
  }
}

    /*
    // estimate using icp
    ccny_rgbd::ICPKd<PointT, PointT> reg;
  
    reg.setMaxIterations(20);
    reg.setTransformationEpsilon(0.0000001);
    reg.setMaxCorrDist(3.0);
    reg.setUseValueRejection(false);
    reg.setUseRANSACRejection(true);
    reg.setRANSACThreshold(0.20);

    pcl::KdTreeFLANN<PointT> tree_data;
    pcl::KdTreeFLANN<PointT> tree_model;

    tree_data.setInputCloud(cloud_src);
    tree_model.setInputCloud(cloud_dst);

    reg.setDataCloud  (&*cloud_src);
    reg.setModelCloud (&*cloud_dst);

    reg.setDataTree  (&tree_data);
    reg.setModelTree (&tree_model);

    reg.align();
    Eigen::Matrix4f corr_eigen_2 = reg.getFinalTransformation();

    pcl::transformPointCloud(*cloud_src, *cloud_src, corr_eigen_2);
    */

/*
bool RGBDFrame::get4RandomIndices(std::vector<int>& random_indices,
                                  unsigned int size)
{
  while (random_indices.size() < 4)
  {
    int n = rand() % size;

    bool exists = false;
    for (unsigned int i = 0; i < random_indices.size(); ++i)
    {
      if (random_indices[i] == n)
      {
        exists = true;
        break;
      }
    }

    if (!exists)
      random_indices.push_back(n);
  }

  return true;
}*/

/*
  // version 2 - use custom ranasc

  srand((unsigned)time(0)); // TODO - move
  int ransac_iterations = 0;

  while (ransac_iterations < ransac_max_iterations)
  {
    // select four random features
    std::vector<int> random_indices;
    RGBDFrame::get4RandomIndices(random_indices, good_matches.size());

    // *** estimate homography

    std::vector<cv::Point2f> initial_src_pts;
    std::vector<cv::Point2f> initial_dst_pts;

    for (unsigned int i = 0; i < random_indices.size(); ++i)
    {
      int random_index = random_indices[i];

      initial_src_pts.push_back(src_pts[random_index]);
      initial_dst_pts.push_back(dst_pts[random_index]);
    }

    cv::Mat h = cv::findHomography(initial_src_pts, initial_dst_pts);

    // **** find new inliers

    std::vector<cv::Point2f> h_src_pts;

    perspectiveTransform(src_pts, h_src_pts, h);

    inlier_matches.clear();

    for (unsigned int i = 0; i < good_matches.size(); ++i)
    {
      double dx = h_src_pts[i].x - dst_pts[i].x;
      double dy = h_src_pts[i].y - dst_pts[i].y;

      double d = dx*dx + dy*dy; //reprojection error

      if (d < eps_reproj_sq)
      {
        inlier_matches.push_back(good_matches[i]);
      }
    }

    inlier_ratio = (float)inlier_matches.size() / (float)good_matches.size();
    if (inlier_ratio > inlier_threshold)
    {
      converged = true;
      break;
    }

    ransac_iterations++;
  }


  if (converged)
    printf("\t[%d] %f\n", ransac_iterations, inlier_ratio);
  else
    printf("\tno ransac model\n");

*/

void RGBDFrame::simpleFilter()
{
  unsigned int w = 30;
  unsigned int dilate_size = 11;

  cv::Mat f_depth_img = cv::Mat(data.height, data.width, CV_32FC1);
  //cv::Mat e_depth_img = cv::Mat(data.height, data.width, CV_32FC1);

  for(unsigned int j = w; j < data.height-w; ++j)
  for(unsigned int i = w; i < data.width-w;  ++i)
  {
    if (i < w || i > data.width  - w || 
        j < w || j > data.height - w)
      f_depth_img.at<float>(j,i) = 0.0;
    else
    {
      unsigned int index = j * data.width + i;
      double z = data.points[index].z;

      if (isnan(z)) f_depth_img.at<float>(j,i) = 0.0;
      else f_depth_img.at<float>(j,i) = z;
    }
  }
/*
  cv::namedWindow("original", CV_WINDOW_NORMAL);
  cv::imshow ("original", f_depth_img);
  cv::waitKey (10);
*/

  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));
  cv::erode(f_depth_img, f_depth_img, element);

/*
  cv::namedWindow("erode", CV_WINDOW_NORMAL);
  cv::imshow ("erode", e_depth_img);
  cv::waitKey (10);
*/

  // back to point cloud

  double nan = std::numeric_limits<double>::quiet_NaN();
  
  for(unsigned int j = 0; j < data.height; ++j)
  for(unsigned int i = 0; i < data.width ; ++i)
  {
    unsigned int index = j * data.width + i;
    
    if(f_depth_img.at<float>(j,i) == 0)
    {
      data.points[index].x = nan;
      data.points[index].y = nan;
      data.points[index].z = nan;
    }
  }

}

} // namespace
