#include "simple_slam.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSlamNode>();
    node->initTF();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

