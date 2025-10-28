// warehouse_bridge.cpp
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class WarehouseBridge : public rclcpp::Node
{
public:
  WarehouseBridge()
  : Node("warehouse_bridge")
  {
    // subscribe to exact topic published by your gazebo plugin
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/warehouse_robot/scan", 10,
      std::bind(&WarehouseBridge::scan_callback, this, _1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/warehouse_robot/pointcloud", 10);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/warehouse_robot/odom", 10);

    // publish odom at 20 Hz so topic always exists
    odom_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&WarehouseBridge::publish_fake_odom, this));

    RCLCPP_INFO(this->get_logger(), "WarehouseBridge started: listening on /warehouse_robot/scan");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // convert LaserScan -> PointCloud2 (simple planar conversion, z=0)
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    std::size_t N = scan->ranges.size();
    cloud->header = scan->header;
    cloud->header.frame_id = scan->header.frame_id;  // keep same frame as laser
    cloud->height = 1;
    cloud->width = static_cast<uint32_t>(N);

    sensor_msgs::msg::PointField pf;
    cloud->fields.clear();
    // x
    pf.name = "x"; pf.offset = 0; pf.datatype = sensor_msgs::msg::PointField::FLOAT32; pf.count = 1;
    cloud->fields.push_back(pf);
    // y
    pf.name = "y"; pf.offset = 4; pf.datatype = sensor_msgs::msg::PointField::FLOAT32; pf.count = 1;
    cloud->fields.push_back(pf);
    // z
    pf.name = "z"; pf.offset = 8; pf.datatype = sensor_msgs::msg::PointField::FLOAT32; pf.count = 1;
    cloud->fields.push_back(pf);

    cloud->is_bigendian = false;
    cloud->point_step = 12; // 3 * 4 bytes (float32)
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->is_dense = false; // if any NaNs or infs, set false

    cloud->data.resize(cloud->row_step * cloud->height);

    // Fill using iterators
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    double angle = scan->angle_min;
    for (size_t i = 0; i < N; ++i, angle += scan->angle_increment) {
      float r = scan->ranges[i];
      if (std::isnan(r) || std::isinf(r) || r < scan->range_min || r > scan->range_max) {
        // mark as NaN
        *iter_x = std::numeric_limits<float>::quiet_NaN();
        *iter_y = std::numeric_limits<float>::quiet_NaN();
        *iter_z = std::numeric_limits<float>::quiet_NaN();
      } else {
        float x = r * std::cos(angle);
        float y = r * std::sin(angle);
        float z = 0.0f;
        *iter_x = x;
        *iter_y = y;
        *iter_z = z;
      }
      ++iter_x; ++iter_y; ++iter_z;
    }

    pc_pub_->publish(*cloud);
  }

  void publish_fake_odom()
  {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";                 // world frame
    msg.child_frame_id = "robot_footprint";       // matches your gazebo diff drive robotBaseFrame

    // For now we publish zeros; replace with real odom if you later compute it
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    // Covariances optional: leave zeros or set reasonable defaults
    // msg.pose.covariance = { ... };

    msg.twist.twist.linear.x = 0.0;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.angular.z = 0.0;

    odom_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WarehouseBridge>());
  rclcpp::shutdown();
  return 0;
}
