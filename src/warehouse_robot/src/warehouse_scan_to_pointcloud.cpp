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

using std::placeholders::_1;

class WarehouseScanToPointcloud : public rclcpp::Node
{
public:
  WarehouseScanToPointcloud()
  : Node("warehouse_scan_to_pointcloud")
  {
    // subscribe to exact topic published by your gazebo plugin
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/warehouse_robot/scan", 10,
      std::bind(&WarehouseScanToPointcloud::scan_callback, this, _1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/warehouse_robot/pointcloud", 10);

    RCLCPP_INFO(this->get_logger(), "WarehouseScanToPointcloud started: listening on /warehouse_robot/scan");
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

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WarehouseScanToPointcloud>());
  rclcpp::shutdown();
  return 0;
}
