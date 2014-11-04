#include "ccny_rgbd/features/feature_detector.h"

namespace ccny_rgbd
{

FeatureDetector::FeatureDetector():
  smooth_(0),
  max_range_(10.0)
  //compute_descriptors_(false)
{

}

FeatureDetector::~FeatureDetector()
{

}

void FeatureDetector::findFeatures(RGBDFrame2& frame)
{
  cv::Mat * input_img = frame.getRGBImage();

  // convert from RGB to grayscale
  cv::Mat gray_img(input_img->rows, input_img->cols, CV_8UC1);
  cvtColor(*input_img, gray_img, CV_BGR2GRAY);

  // blur if needed
  if(smooth_ > 0)
  {
    int blur_size = smooth_*2 + 1;
    cv::GaussianBlur(gray_img, gray_img, cv::Size(blur_size, blur_size), 0);
  }

  findFeatures(frame, &gray_img);
  frame.constructFeatureCloud(max_range_);

  /*
  cv::namedWindow("Features", CV_WINDOW_NORMAL);
  cv::Mat kp_img(input_img->rows, input_img->cols, CV_8UC1);
  cv::drawKeypoints(*input_img, frame.keypoints, kp_img);
  cv::imshow ("features", kp_img);
  cv::waitKey (10);
  */
}

/*
bool FeatureDetector::testPixel(const cv::Mat& img,
                                const PointCloudT& cloud_in,
                                double z, int n_i, int n_j,
                                int& corr_index)
{

  int n_index = n_j*cloud_in.width + n_i;
  double n_z = cloud_in.points[n_index].z;

  double scaled_threshold = std::min(0.15, 0.03 * z * z);

  if (!isnan(n_z) && z - n_z > scaled_threshold)
  {
    corr_index = n_index;
    return true;
  }
  else return false;
}
*/
/*
void FeatureDetector::constructFeaturePointCloud(RGBDFrame2& frame, const cv::Mat * input_img)
{
  for (unsigned int i = 0; i < frame.keypoints.size(); ++i)
  {
    int ix = frame.keypoints[i].pt.x;
    int iy = frame.keypoints[i].pt.y;

    unsigned int index = iy * input_img->cols + ix;

    // **** filter using z data ****************************************
    // Neighbors := cross (window_*2+1, window*2 +1)
    // If any of the neighbors are closer and the z jump is big enough,
    // use them instead.

    double z = frame.data.points[index].z;

    if (!isnan(z) && z < max_range_)
    {
      int corr_index = index;

      for (int c = 1; c <= window_; ++c)
      {
        if(ix-c > 0)
          if (testPixel(*input_img, frame.data, z, ix-c, iy, corr_index)) break;
        if(ix+c < input_img->cols)
          if (testPixel(*input_img, frame.data, z, ix+c, iy, corr_index)) break;
        if(iy-c > 0)
          if (testPixel(*input_img, frame.data, z, ix, iy-c, corr_index)) break;
        if(iy+c < input_img->rows)
          if (testPixel(*input_img, frame.data, z, ix, iy+c, corr_index)) break;
      }

      PointFeature p;
      p.x = frame.data.points[corr_index].x;
      p.y = frame.data.points[corr_index].y;
      p.z = frame.data.points[corr_index].z;

      frame.rgb_features.points.push_back(p);
    }
  }

  frame.rgb_features.header = frame.data.header;
  frame.rgb_features.height = 1;
  frame.rgb_features.width = frame.rgb_features.points.size();
  frame.rgb_features.is_dense = true;
}
*/
} //namespace
