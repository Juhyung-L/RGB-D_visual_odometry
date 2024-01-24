#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "visual_odometry/visual_odom_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<visual_odometry::VisualOdometryNode>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}