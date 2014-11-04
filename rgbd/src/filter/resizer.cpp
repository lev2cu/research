#include "ccny_rgbd/filter/resizer.h"

namespace ccny_rgbd
{

Resizer::Resizer(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  publish_qvga_(true),
  publish_qqvga_(true)
{
  ROS_INFO("Starting Resizer");

  width  = 640;
  height = 480;

  qwidth  = width/2;
  qheight = height/2;

  qqwidth  = width/4;
  qqheight = height/4;

  // **** publishers

  vga_pub_    = nh_.advertise<PointCloudT>(pub_vga_topic_, 1);
  qvga_pub_   = nh_.advertise<PointCloudT>(pub_qvga_topic_, 1);
  qqvga_pub_  = nh_.advertise<PointCloudT>(pub_qqvga_topic_, 1);

  // **** subscribers

  point_cloud_subscriber_ = nh_.subscribe<PointCloudT>(
    sub_topic_, 1, &Resizer::pointCloudCallback, this);
}

Resizer::~Resizer()
{
  ROS_INFO("Destroying Resizer");
}

void Resizer::pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr)
{
  struct timeval start, end;
  gettimeofday(&start, NULL);

  if (publish_qvga_)
  {
    PointCloudT::Ptr qvga_ = boost::make_shared<PointCloudT>();

    qvga_->points.resize(qwidth * qheight);
    qvga_->width = qwidth;
    qvga_->height = qheight;

    for (unsigned int i = 0; i < qwidth;  ++i)
    for (unsigned int j = 0; j < qheight; ++j)
    {
      unsigned int  index = (j*2)*width + i*2;
      unsigned int qindex = j*qwidth + i;

      qvga_->points[qindex] = cloud_in_ptr->points[index];
    }

    qvga_->header = cloud_in_ptr->header;

    qvga_pub_.publish(qvga_);
  }

  if (publish_qqvga_)
  {
    PointCloudT::Ptr qqvga_ = boost::make_shared<PointCloudT>();

    qqvga_->points.resize(qqwidth * qqheight);
    qqvga_->width = qqwidth;
    qqvga_->height = qqheight;

    for (unsigned int i = 0; i < qqwidth;  ++i)
    for (unsigned int j = 0; j < qqheight; ++j)
    {
      unsigned int   index = (j*4)*width + i*4;
      unsigned int qqindex = j*qqwidth + i;

      qqvga_->points[qqindex] = cloud_in_ptr->points[index];
    }

    qqvga_->header = cloud_in_ptr->header;

    qqvga_pub_.publish(qqvga_);
  }

  gettimeofday(&end, NULL);

  double callback_dur  = msDuration(start, end);

  printf("Resize: %.1f ms\n", callback_dur);
}

} //namespace ccny_rgbd

