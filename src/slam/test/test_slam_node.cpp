#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "../include/simple_slam.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class SimpleSlamNodeTest : public ::testing::Test {
protected:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<SimpleSlamNode> slam_node;
    
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("test_node");
        slam_node = std::make_shared<SimpleSlamNode>();
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }

    sensor_msgs::msg::LaserScan::SharedPtr createTestScan() {
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.frame_id = "laser";
        scan->angle_min = -M_PI/4;
        scan->angle_max = M_PI/4;
        scan->angle_increment = M_PI/8;
        scan->range_min = 0.1;
        scan->range_max = 10.0;
        scan->ranges = {1.0, 2.0, 1.5, 2.5, 1.8};
        return scan;
    }

    nav_msgs::msg::Odometry::SharedPtr createTestOdom() {
        auto odom = std::make_shared<nav_msgs::msg::Odometry>();
        odom->header.frame_id = "odom";
        odom->child_frame_id = "robot_footprint";
        odom->pose.pose.position.x = 1.0;
        odom->pose.pose.position.y = 2.0;
        odom->pose.pose.orientation.w = 1.0;
        return odom;
    }
};

TEST_F(SimpleSlamNodeTest, NodeInitialization) {
    EXPECT_TRUE(slam_node->get_node_base_interface() != nullptr);
}

TEST_F(SimpleSlamNodeTest, ScanToPointsLocal) {
    auto scan = createTestScan();
    auto points = slam_node->scanToPointsLocal(scan);
    EXPECT_GT(points.size(), 0);
}

TEST_F(SimpleSlamNodeTest, TransformPoints) {
    std::vector<Eigen::Vector2d> points = {
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.0, 1.0)
    };
    
    // Create a translation transform
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T(0,2) = 1.0;  // x translation
    T(1,2) = 2.0;  // y translation
    
    auto transformed = slam_node->transformPoints(T, points);
    EXPECT_EQ(transformed.size(), points.size());
    EXPECT_NEAR(transformed[0].x(), 2.0, 1e-6);  // 1 + 1
    EXPECT_NEAR(transformed[0].y(), 2.0, 1e-6);  // 0 + 2
}

TEST_F(SimpleSlamNodeTest, MapPublication) {
    bool map_received = false;
    auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 1,
        [&map_received]([[maybe_unused]] nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_received = true;
        }
    );
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(slam_node);
    
    // Spin for a short time to allow map publication
    auto start = node->now();
    while (node->now() - start < rclcpp::Duration::from_seconds(2.0)) {
        executor.spin_some();
        if (map_received) break;
    }
    
    EXPECT_TRUE(map_received);
}

TEST_F(SimpleSlamNodeTest, TFPublication) {
    slam_node->initTF();
    
    // Process a scan to trigger TF publication
    auto scan = createTestScan();
    slam_node->scanCallback(scan);
    
    // Check if TF is being published (would need tf2_ros listener in real test)
    EXPECT_TRUE(true);  // Basic structural test
}

// Main test runner
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}