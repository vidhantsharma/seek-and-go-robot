#include "simple_slam.hpp"
#include "simple_slam.h"

#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

/* ----------------------
   ICP2D implementation
   ---------------------- */
ICP2D::ICP2D()
: max_iterations_(20), max_correspondence_dist_(0.5), convergence_tol_(1e-4) {}

void ICP2D::setMaxIterations(int it) { max_iterations_ = it; }
void ICP2D::setMaxCorrespondenceDistance(double d) { max_correspondence_dist_ = d; }
void ICP2D::setConvergenceTolerance(double t) { convergence_tol_ = t; }

ICP2D::Result ICP2D::run(const std::vector<Eigen::Vector2d> &source_in,
                         const std::vector<Eigen::Vector2d> &target_in)
{
  Result res;
  if (source_in.empty() || target_in.empty()) {
    res.fitness = std::numeric_limits<double>::infinity();
    return res;
  }
  std::vector<Eigen::Vector2d> source = source_in;
  const auto &target = target_in;

  Eigen::Matrix3d T_acc = Eigen::Matrix3d::Identity();
  double prev_error = 1e12;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    std::vector<Eigen::Vector2d> P;
    std::vector<Eigen::Vector2d> Q;
    double sum_err = 0.0;

    for (const auto &p : source) {
      double best_d2 = max_correspondence_dist_*max_correspondence_dist_;
      int best_idx = -1;
      for (size_t j=0;j<target.size();++j) {
        double d2 = (p - target[j]).squaredNorm();
        if (d2 < best_d2) {
          best_d2 = d2;
          best_idx = (int)j;
        }
      }
      if (best_idx >= 0) {
        P.push_back(p);
        Q.push_back(target[best_idx]);
        sum_err += best_d2;
      }
    }

    if (P.size() < 3) {
      res.inliers = (int)P.size();
      res.fitness = (P.empty()? std::numeric_limits<double>::infinity() : sum_err / P.size());
      res.transform = T_acc;
      return res;
    }

    Eigen::Vector2d p_mean(0,0), q_mean(0,0);
    for (size_t i=0;i<P.size();++i) { p_mean += P[i]; q_mean += Q[i]; }
    p_mean /= (double)P.size();
    q_mean /= (double)Q.size();

    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    for (size_t i=0;i<P.size();++i) {
      Eigen::Vector2d p_hat = P[i] - p_mean;
      Eigen::Vector2d q_hat = Q[i] - q_mean;
      W += p_hat * q_hat.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    Eigen::Matrix2d R2 = V * U.transpose();
    if (R2.determinant() < 0) {
      V.col(1) *= -1;
      R2 = V * U.transpose();
    }
    Eigen::Vector2d t = q_mean - R2 * p_mean;

    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T(0,0)=R2(0,0); T(0,1)=R2(0,1); T(0,2)=t(0);
    T(1,0)=R2(1,0); T(1,1)=R2(1,1); T(1,2)=t(1);

    for (size_t i=0;i<source.size();++i) {
      Eigen::Vector3d v(source[i].x(), source[i].y(), 1.0);
      Eigen::Vector3d v2 = T * v;
      source[i].x() = v2(0);
      source[i].y() = v2(1);
    }

    T_acc = T * T_acc;

    double mean_err = sum_err / P.size();
    if (std::abs(prev_error - mean_err) < convergence_tol_) {
      res.inliers = (int)P.size();
      res.fitness = mean_err;
      res.transform = T_acc;
      return res;
    }
    prev_error = mean_err;
  }

  res.transform = T_acc;
  res.inliers = 0;
  res.fitness = prev_error;
  return res;
}

/* ----------------------
   OccupancyGridMap implementation
   ---------------------- */
OccupancyGridMap::OccupancyGridMap(double width_m, double height_m, double resolution)
: width_m_(width_m), height_m_(height_m), res_(resolution)
{
  w_ = std::max(1, (int)std::round(width_m_ / res_));
  h_ = std::max(1, (int)std::round(height_m_ / res_));
  log_odds_.assign(w_ * h_, 0.0);
  origin_x_ = -width_m_ / 2.0;
  origin_y_ = -height_m_ / 2.0;
  l_occ_ = 0.85;
  l_free_ = -0.4;
  l_min_ = -4.0;
  l_max_ = 4.0;
}

void OccupancyGridMap::clear() {
  std::fill(log_odds_.begin(), log_odds_.end(), 0.0);
}

bool OccupancyGridMap::worldToCell(double wx, double wy, int &cx, int &cy) const {
  double rx = (wx - origin_x_) / res_;
  double ry = (wy - origin_y_) / res_;
  cx = (int)std::floor(rx);
  cy = (int)std::floor(ry);
  if (cx < 0 || cy < 0 || cx >= w_ || cy >= h_) return false;
  return true;
}

void OccupancyGridMap::updateWithScan(const Pose2D &pose, const std::vector<Eigen::Vector2d> &endpoints, double /*max_range*/) {
  for (const auto &end : endpoints) {
    int x0, y0, x1, y1;
    if (!worldToCell(pose.x, pose.y, x0, y0)) continue;

    double end_x = end.x();
    double end_y = end.y();

    bool endpoint_inside = worldToCell(end_x, end_y, x1, y1);

    if (!endpoint_inside) {
      double dx = end_x - pose.x;
      double dy = end_y - pose.y;
      double lo = 0.0, hi = 1.0;
      for (int it = 0; it < 20; ++it) {
        double mid = 0.5*(lo + hi);
        double mx = pose.x + dx*mid;
        double my = pose.y + dy*mid;
        int tcx, tcy;
        if (worldToCell(mx, my, tcx, tcy)) lo = mid; else hi = mid;
      }
      double cx = pose.x + dx*lo;
      double cy = pose.y + dy*lo;
      if (!worldToCell(cx, cy, x1, y1)) continue;
    }

    std::vector<std::pair<int,int>> cells;
    bresenham(x0,y0,x1,y1,cells);
    for (size_t k=0; k+1 < cells.size(); ++k) {
      int idx = cells[k].second * w_ + cells[k].first;
      log_odds_[idx] += l_free_;
      if (log_odds_[idx] < l_min_) log_odds_[idx] = l_min_;
    }

    bool actual_hit = endpoint_inside; // conservative
    if (endpoint_inside && actual_hit && !cells.empty()) {
      int idx = cells.back().second * w_ + cells.back().first;
      log_odds_[idx] += l_occ_;
      if (log_odds_[idx] > l_max_) log_odds_[idx] = l_max_;
    } else {
      // do not mark occupied
    }
  }
}

nav_msgs::msg::OccupancyGrid OccupancyGridMap::toOccupancyGridMsg(const std::string &frame_id, rclcpp::Time t) const {
  nav_msgs::msg::OccupancyGrid g;
  g.header.stamp = t;
  g.header.frame_id = frame_id;
  g.info.map_load_time = t;
  g.info.resolution = res_;
  g.info.width = w_;
  g.info.height = h_;
  g.info.origin.position.x = origin_x_;
  g.info.origin.position.y = origin_y_;
  g.info.origin.position.z = 0.0;
  g.info.origin.orientation.w = 1.0;
  g.info.origin.orientation.x = 0.0;
  g.info.origin.orientation.y = 0.0;
  g.info.origin.orientation.z = 0.0;

  g.data.resize(w_ * h_);
  const double unknown_eps = 1e-6; // much smaller than 0.01
  for (int j = 0; j < h_; ++j) {
    for (int i = 0; i < w_; ++i) {
      int idx = j * w_ + i;
      double l = log_odds_[idx];

      // canonical sigmoid
      double p = 1.0 / (1.0 + std::exp(-l)); // p in (0,1)
      if (std::abs(l) < unknown_eps) {
        g.data[idx] = -1; // unknown
      } else {
        // map probability to [0..100]
        int val = (int)std::round(std::clamp(p * 100.0, 0.0, 100.0));
        g.data[idx] = static_cast<int8_t>(val);
      }
    }
  }
  return g;
}

void OccupancyGridMap::bresenham(int x0, int y0, int x1, int y1, std::vector<std::pair<int,int>> &cells) const {
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  while (true) {
    cells.emplace_back(x,y);
    if (x == x1 && y == y1) break;
    int e2 = 2*err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
  }
}

/* ----------------------
   SimpleSlamNode implementation
   ---------------------- */
SimpleSlamNode::SimpleSlamNode()
: Node("simple_2d_slam"),
  map_width_m_(40.0),
  map_height_m_(40.0),
  map_resolution_(0.05),
  scan_downsample_(2),
  icp_max_iterations_(25),
  icp_max_correspondence_dist_(0.5),
  icp_convergence_tol_(1e-4),
  odom_use_(true),
  have_last_scan_(false),
  have_odom_pose_(false)
{
  // declare parameters
  declare_parameter("map_width_m", map_width_m_);
  declare_parameter("map_height_m", map_height_m_);
  declare_parameter("map_resolution", map_resolution_);
  declare_parameter("scan_downsample", scan_downsample_);
  declare_parameter("icp_max_iterations", icp_max_iterations_);
  declare_parameter("icp_max_correspondence_dist", icp_max_correspondence_dist_);
  declare_parameter("icp_convergence_tol", icp_convergence_tol_);
  declare_parameter("odom_use", odom_use_);

  // read parameters (allow override from CLI / yaml)
  map_width_m_ = get_parameter("map_width_m").as_double();
  map_height_m_ = get_parameter("map_height_m").as_double();
  map_resolution_ = get_parameter("map_resolution").as_double();
  scan_downsample_ = get_parameter("scan_downsample").as_int();
  icp_max_iterations_ = get_parameter("icp_max_iterations").as_int();
  icp_max_correspondence_dist_ = get_parameter("icp_max_correspondence_dist").as_double();
  icp_convergence_tol_ = get_parameter("icp_convergence_tol").as_double();
  odom_use_ = get_parameter("odom_use").as_bool();

  gridmap_ = std::make_unique<OccupancyGridMap>(map_width_m_, map_height_m_, map_resolution_);

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_matched", 5);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/warehouse_robot/scan", rclcpp::SensorDataQoS(),
    std::bind(&SimpleSlamNode::scanCallback, this, std::placeholders::_1)
  );

  if (odom_use_) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&SimpleSlamNode::odomCallback, this, std::placeholders::_1)
    );
  }

  icp_.setMaxIterations(icp_max_iterations_);
  icp_.setMaxCorrespondenceDistance(icp_max_correspondence_dist_);
  icp_.setConvergenceTolerance(icp_convergence_tol_);

  map_pub_timer_ = create_wall_timer(500ms, std::bind(&SimpleSlamNode::publishMapAndTF, this));
  RCLCPP_INFO(get_logger(), "Simple 2D SLAM node started");
}

void SimpleSlamNode::initTF()
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void SimpleSlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_pose_.x = msg->pose.pose.position.x;
  odom_pose_.y = msg->pose.pose.position.y;
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  double siny = 2.0*(qw*qz + qx*qy);
  double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
  odom_pose_.theta = std::atan2(siny, cosy);
  have_odom_pose_ = true;
}

std::vector<Eigen::Vector2d> SimpleSlamNode::scanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr &scan, const Pose2D &pose_world) {
  std::vector<Eigen::Vector2d> pts;
  int n = (int)scan->ranges.size();
  for (int i=0;i<n;i += std::max(1, scan_downsample_)) {
    float r = scan->ranges[i];
    if (std::isnan(r) || std::isinf(r)) continue;
    if (r <= scan->range_min || r >= scan->range_max) continue;
    double angle = scan->angle_min + i*scan->angle_increment;
    double gx = pose_world.x + r * std::cos(pose_world.theta + angle);
    double gy = pose_world.y + r * std::sin(pose_world.theta + angle);
    pts.emplace_back(gx, gy);
  }
  return pts;
}

std::vector<Eigen::Vector2d> SimpleSlamNode::scanToPointsLocal(const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
  std::vector<Eigen::Vector2d> pts;
  int n = (int)scan->ranges.size();
  for (int i=0;i<n;i += std::max(1, scan_downsample_)) {
    float r = scan->ranges[i];
    if (std::isnan(r) || std::isinf(r)) continue;
    if (r <= scan->range_min || r >= scan->range_max) continue;
    double angle = scan->angle_min + i*scan->angle_increment;
    double lx = r * std::cos(angle);
    double ly = r * std::sin(angle);
    pts.emplace_back(lx, ly);
  }
  return pts;
}

std::vector<Eigen::Vector2d> SimpleSlamNode::transformPoints(const Eigen::Matrix3d &T, const std::vector<Eigen::Vector2d> &pts) {
  std::vector<Eigen::Vector2d> out;
  out.reserve(pts.size());
  for (const auto &p : pts) {
    Eigen::Vector3d v(p.x(), p.y(), 1.0);
    Eigen::Vector3d v2 = T * v;
    out.emplace_back(v2.x(), v2.y());
  }
  return out;
}

void SimpleSlamNode::publishMapAndTF() {
  rclcpp::Time now = this->get_clock()->now();
  auto map_msg = gridmap_->toOccupancyGridMsg("map", now);
  map_pub_->publish(map_msg);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "map";
  t.child_frame_id = "robot_footprint";
  t.transform.translation.x = global_pose_.x;
  t.transform.translation.y = global_pose_.y;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0,0,global_pose_.theta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  if (tf_broadcaster_) tf_broadcaster_->sendTransform(t);
}

void SimpleSlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  rclcpp::Time now = this->get_clock()->now();
  std::lock_guard<std::mutex> lock(scan_mutex_);

  auto scan_local_pts = scanToPointsLocal(scan);

  if (!have_last_scan_) {
    if (odom_use_ && have_odom_pose_) {
      global_pose_ = odom_pose_;
    } else {
      global_pose_.x = 0.0; global_pose_.y = 0.0; global_pose_.theta = 0.0;
    }
    Eigen::Matrix3d T0 = transform_from_pose(global_pose_);
    last_scan_points_ = transformPoints(T0, scan_local_pts);
    gridmap_->updateWithScan(global_pose_, last_scan_points_, scan->range_max);
    have_last_scan_ = true;

    auto scan_out = *scan;
    scan_out.header.stamp = now;
    scan_out.header.frame_id = "robot_footprint";
    scan_pub_->publish(scan_out);
    return;
  }

  Pose2D predicted_pose = global_pose_;
  if (odom_use_ && have_odom_pose_) {
    predicted_pose = odom_pose_;
  }

  Eigen::Matrix3d Tpred = transform_from_pose(predicted_pose);
  auto scan_world_initial = transformPoints(Tpred, scan_local_pts);

  ICP2D::Result icp_res = icp_.run(scan_world_initial, last_scan_points_);

  Eigen::Matrix3d T_new = icp_res.transform * Tpred;
  Pose2D new_pose = pose_from_transform(T_new);

  new_pose.theta = normalize_angle(new_pose.theta);
  global_pose_ = new_pose;

  Eigen::Matrix3d Tnew = transform_from_pose(global_pose_);
  auto scan_world = transformPoints(Tnew, scan_local_pts);
  gridmap_->updateWithScan(global_pose_, scan_world, scan->range_max);

  last_scan_points_.swap(scan_world);

  auto scan_out = *scan;
  scan_out.header.stamp = now;
  scan_out.header.frame_id = "robot_footprint";
  scan_pub_->publish(scan_out);
}
