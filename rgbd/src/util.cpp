#include "ccny_rgbd/util.h"

namespace ccny_rgbd
{

//float dist(const PointOrb& p1, const PointOrb& p2)
//{
//  return cv::Hamming()(p1.descriptor, p2.descriptor, 32);
//}
//
//float dist(const PointCanny& p1, const PointCanny& p2)
//{
//  double da =  p1.angle - p2.angle;
//
//  // FIXME nomralize
//
//  return da;
//}
//
//float dist(const PointSurf& p1, const PointSurf& p2)
//{
//  // FIXME implement
//
//  return 0;
//}

double msDuration (struct timeval start, struct timeval end)
{
    return ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
            (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
}

void getTfDifference(const tf::Transform& a, const tf::Transform b, double& dist, double& angle)
{
  tf::Transform motion = a.inverse() * b;
  dist = motion.getOrigin().length();
  float trace = motion.getBasis()[0][0] + motion.getBasis()[1][1] + motion.getBasis()[2][2];
  angle = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));
}

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
//  #if ROS_VERSION_MINOR >= 6
    MyMatrix btm;
//  #else
//    btMatrix3x3 btm;
//  #endif

  btm.setValue(trans(0,0),trans(0,1),trans(0,2),
            trans(1,0),trans(1,1),trans(1,2),
            trans(2,0),trans(2,1),trans(2,2));
  tf::Transform ret;
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

void getXYZRPY(const tf::Transform& t,
                     double& x,    double& y,     double& z,
                     double& roll, double& pitch, double& yaw)
{
  x = t.getOrigin().getX();
  y = t.getOrigin().getY();
  z = t.getOrigin().getZ();

//  #if ROS_VERSION_MINOR >= 6
    MyMatrix m(t.getRotation());
//  #else
//    btMatrix3x3 m(t.getRotation());
//  #endif

  m.getRPY(roll, pitch, yaw);
}

} //namespace ccny_rgbd
