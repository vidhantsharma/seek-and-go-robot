#ifndef SIMPLE_SLAM_HPP_
#define SIMPLE_SLAM_HPP_

#include "simple_slam_node.h"

#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

/* ----------------------
   ICP2D
   ---------------------- */
class ICP2D {
public:
  struct Result {
    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
    double fitness = std::numeric_limits<double>::infinity();
    int inliers = 0;
  };

  ICP2D();
  void setMaxIterations(int it);
  void setMaxCorrespondenceDistance(double d);
  void setConvergenceTolerance(double t);

  Result run(const std::vector<Eigen::Vector2d> &source,
             const std::vector<Eigen::Vector2d> &target);

private:
  int max_iterations_;
  double max_correspondence_dist_;
  double convergence_tol_;
};

/* ----------------------
   OccupancyGridMap
   ---------------------- */
class OccupancyGridMap {
public:
  OccupancyGridMap(double width_m, double height_m, double resolution);
  void clear();

  // world -> cell index
  bool worldToCell(double wx, double wy, int &cx, int &cy) const;

  // update with world endpoints
  void updateWithScan(const Pose2D &pose,
                      const std::vector<Eigen::Vector2d> &endpoints,
                      double max_range);

  nav_msgs::msg::OccupancyGrid toOccupancyGridMsg(const std::string &frame_id,
                                                  rclcpp::Time t) const;

private:
  double width_m_, height_m_, res_;
  int w_, h_;
  double origin_x_, origin_y_;
  std::vector<double> log_odds_;
  double l_occ_, l_free_, l_min_, l_max_;

  void bresenham(int x0, int y0, int x1, int y1, std::vector<std::pair<int,int>> &cells) const;
};

/* ----------------------
   SimpleSlamNode
   ---------------------- */
class SimpleSlamNode : public rclcpp::Node {
public:
  SimpleSlamNode();
  void initTF(); // call after making shared_ptr

private:
  // parameters
  double map_width_m_, map_height_m_, map_resolution_;
  int scan_downsample_;
  int icp_max_iterations_;
  double icp_max_correspondence_dist_;
  double icp_convergence_tol_;
  bool odom_use_;

  // map & algorithms
  std::unique_ptr<OccupancyGridMap> gridmap_;
  ICP2D icp_;

  // state
  Pose2D global_pose_; // map <-- base_link
  std::vector<Eigen::Vector2d> last_scan_points_;
  bool have_last_scan_;
  std::mutex scan_mutex_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr map_pub_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // odom buffer (simple)
  Pose2D odom_pose_;
  bool have_odom_pose_;

  // callbacks & helpers
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  std::vector<Eigen::Vector2d> scanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                                            const Pose2D &pose_world);
  std::vector<Eigen::Vector2d> scanToPointsLocal(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
  std::vector<Eigen::Vector2d> transformPoints(const Eigen::Matrix3d &T,
                                               const std::vector<Eigen::Vector2d> &pts);
  void publishMapAndTF();
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif // SIMPLE_SLAM_HPP_
