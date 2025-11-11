#include <gtest/gtest.h>
#include "../include/simple_slam.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class ICPTest : public ::testing::Test {
protected:
    ICP2D icp;
    void SetUp() override {
        icp.setMaxIterations(50);
        icp.setMaxCorrespondenceDistance(2.0);
        icp.setConvergenceTolerance(1e-4);
    }
};

TEST_F(ICPTest, EmptyPointClouds) {
    std::vector<Eigen::Vector2d> source, target;
    auto result = icp.run(source, target);
    EXPECT_TRUE(std::isinf(result.fitness));
    EXPECT_EQ(result.inliers, 0);
}

TEST_F(ICPTest, IdentityTransform) {
    std::vector<Eigen::Vector2d> source = {
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.0, 1.0),
        Eigen::Vector2d(-1.0, 0.0)
    };
    auto result = icp.run(source, source);
    EXPECT_NEAR(result.fitness, 0.0, 1e-5);
    EXPECT_EQ(result.inliers, 3);
    EXPECT_TRUE(result.transform.isApprox(Eigen::Matrix3d::Identity()));
}

TEST_F(ICPTest, TranslationOnly) {
    std::vector<Eigen::Vector2d> source = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.0, 1.0)
    };
    std::vector<Eigen::Vector2d> target = {
        Eigen::Vector2d(1.0, 1.0),
        Eigen::Vector2d(2.0, 1.0),
        Eigen::Vector2d(1.0, 2.0)
    };
    
    auto result = icp.run(source, target);
    EXPECT_NEAR(result.transform(0,2), 1.0, 1e-3); // x translation
    EXPECT_NEAR(result.transform(1,2), 1.0, 1e-3); // y translation
    EXPECT_GT(result.inliers, 0);
}

TEST_F(ICPTest, RotationOnly) {
    double angle = M_PI / 4; // 45 degrees

    // Use >=3 non-collinear points (more robust for ICP)
    std::vector<Eigen::Vector2d> source = {
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.0, 1.0),
        Eigen::Vector2d(-1.0, 0.0),
        Eigen::Vector2d(0.0, -1.0)
    };
    std::vector<Eigen::Vector2d> target;
    for (const auto& p : source) {
        target.emplace_back(
            p.x() * std::cos(angle) - p.y() * std::sin(angle),
            p.x() * std::sin(angle) + p.y() * std::cos(angle)
        );
    }

    auto result = icp.run(source, target);

    // require some correspondences found before asserting angle
    EXPECT_GT(result.inliers, 0);

    double found_angle = std::atan2(result.transform(1,0), result.transform(0,0));
    EXPECT_NEAR(found_angle, angle, 1e-2); // relaxed tolerance
}