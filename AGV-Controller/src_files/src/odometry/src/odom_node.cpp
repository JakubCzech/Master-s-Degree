#include "odometry/odom.cpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Odometry>();
    RCLCPP_INFO(node->get_logger(), "Odom publisher node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
