#ifndef SIMPLE_SLAM_H_
#define SIMPLE_SLAM_H_

#include <cmath>
#include <Eigen/Dense>

// Simple 2D pose
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

inline double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0*M_PI;
  while (a <= -M_PI) a += 2.0*M_PI;
  return a;
}

inline Eigen::Matrix3d transform_from_pose(const Pose2D &p) {
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  double c = std::cos(p.theta), s = std::sin(p.theta);
  T(0,0)=c; T(0,1)=-s; T(0,2)=p.x;
  T(1,0)=s; T(1,1)=c;  T(1,2)=p.y;
  return T;
}

inline Pose2D pose_from_transform(const Eigen::Matrix3d &T) {
  Pose2D p;
  p.x = T(0,2);
  p.y = T(1,2);
  p.theta = std::atan2(T(1,0), T(0,0));
  return p;
}

#endif // SIMPLE_SLAM_H_
