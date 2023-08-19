#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>

class CloudMerger : public rclcpp::Node
{
  public:
    CloudMerger()
    : Node("cloud_merger")
    {
      subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/front_scanner/cloud", rclcpp::QoS(rclcpp::SensorDataQoS()),
          std::bind(&CloudMerger::cloud_1_callback, this, std::placeholders::_1));
      subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rear_scanner/cloud", rclcpp::QoS(rclcpp::SensorDataQoS()),
          std::bind(&CloudMerger::cloud_2_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("combined_cloud",  rclcpp::QoS(rclcpp::SensorDataQoS()));
      RCLCPP_INFO(this->get_logger(), "Init node cloud_merger");
    }

  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    void cloud_1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs)
	  {
	    pcl::fromROSMsg(*point_cloud2_msgs, *point_cloud1);
	    merge_and_publish();
	  }

	 void cloud_2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs)
	 {
	    pcl::fromROSMsg(*point_cloud2_msgs, *point_cloud2);
	    merge_and_publish();
	 }

	 void merge_and_publish()
	 {
	    sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*point_cloud1+*point_cloud2, msg);
	    publisher_->publish(msg);
	 }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_, subscription2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudMerger>();
  RCLCPP_INFO(node->get_logger(), "Cloud merger publisher node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
