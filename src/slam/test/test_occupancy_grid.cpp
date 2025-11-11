// test_occupancy_grid_map.cpp
#include <gtest/gtest.h>
#include "../include/simple_slam.hpp"
#include <cmath>

class OccupancyGridMapTest : public ::testing::Test {
protected:
    std::unique_ptr<OccupancyGridMap> map;
    
    void SetUp() override {
        // Create a 10x10 meter map with 0.1m resolution -> 100 x 100 cells
        map = std::make_unique<OccupancyGridMap>(10.0, 10.0, 0.1);
        map->clear();
    }

    void TearDown() override {
        map.reset();
    }
};

TEST_F(OccupancyGridMapTest, Initialization) {
    // width/height in cells = meters / resolution = 10.0 / 0.1 = 100
    EXPECT_EQ(map->getWidth(), 100);
    EXPECT_EQ(map->getHeight(), 100);
    EXPECT_NEAR(map->getResolution(), 0.1, 1e-6);
}

TEST_F(OccupancyGridMapTest, WorldToCell) {
    int cx, cy;
    // Center of map (0,0 in world coordinates) should map to middle cell
    ASSERT_TRUE(map->worldToCell(0.0, 0.0, cx, cy));
    EXPECT_EQ(cx, 50);  // middle cell x (0..99 -> center is 50)
    EXPECT_EQ(cy, 50);  // middle cell y

    // Out of bounds: 10.0 is outside since map extents are [-5.0, +5.0)
    EXPECT_FALSE(map->worldToCell(10.0, 10.0, cx, cy));
}

TEST_F(OccupancyGridMapTest, UpdateWithScan) {
    Pose2D robot_pose{0.0, 0.0, 0.0};

    std::vector<Eigen::Vector2d> endpoints = {
        {1.0, 0.0},  // 1m ahead
        {0.0, 1.0}   // 1m to the left
    };

    map->updateWithScan(robot_pose, endpoints, 5.0);
    auto grid_msg = map->toOccupancyGridMsg("map", rclcpp::Time());

    int cx1, cy1, cx2, cy2;
    ASSERT_TRUE(map->worldToCell(1.0, 0.0, cx1, cy1));
    ASSERT_TRUE(map->worldToCell(0.0, 1.0, cx2, cy2));

    int idx1 = cy1 * map->getWidth() + cx1;
    int idx2 = cy2 * map->getWidth() + cx2;

    // minimal checks: cells should be known and look occupied (>50)
    EXPECT_GT(static_cast<int>(grid_msg.data[idx1]), 50);
    EXPECT_GT(static_cast<int>(grid_msg.data[idx2]), 50);
}

TEST_F(OccupancyGridMapTest, Clear) {
    Pose2D robot_pose{0.0, 0.0, 0.0};
    std::vector<Eigen::Vector2d> endpoints = { {1.0, 0.0} };
    map->updateWithScan(robot_pose, endpoints, 5.0);

    map->clear();
    auto grid_msg = map->toOccupancyGridMsg("map", rclcpp::Time());
    for (const auto& cell : grid_msg.data) {
        EXPECT_EQ(cell, -1);
    }
}

TEST_F(OccupancyGridMapTest, Bresenham) {
    std::vector<std::pair<int,int>> cells;
    map->bresenham(0, 0, 3, 3, cells);

    // Should have 4 points for diagonal line (0,0),(1,1),(2,2),(3,3)
    EXPECT_EQ(cells.size(), 4);

    for (size_t i = 0; i < cells.size(); ++i) {
        EXPECT_EQ(cells[i].first, cells[i].second);  // x==y on diagonal
    }
}
