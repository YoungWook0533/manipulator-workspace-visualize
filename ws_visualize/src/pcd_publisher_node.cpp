#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PCDPublisher : public rclcpp::Node
{
public:
  PCDPublisher() : Node("pcd_publisher_node")
  {
    /* ---------- ROS 파라미터 ---------- */
    this->declare_parameter<std::string>("pcd_path", "/home/yeonguk/mani_ws/src/txt_to_pcd/build/DBB.pcd");
    this->declare_parameter<std::string>("topic",    "/dbb_cloud");
    this->declare_parameter<double>("publish_rate",  0.05);      // [Hz]

    std::string pcd_path  = this->get_parameter("pcd_path").as_string();
    std::string topic     = this->get_parameter("topic").as_string();
    double      rate_hz   = this->get_parameter("publish_rate").as_double();

    /* ---------- PCD 로드 ---------- */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_path, *cloud) == -1)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to load PCD file: %s", pcd_path.c_str());
      rclcpp::shutdown();
      return;
    }
    pcl::toROSMsg(*cloud, ros_cloud_);
    ros_cloud_.header.frame_id = "base_footprint";

    /* ---------- 퍼블리셔 & 타이머 ---------- */
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();                     // <- Reliable 로 설정
    qos.durability_volatile();          // (센서 데이터니까 volatile 유지)

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&PCDPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points from '%s'. Publishing on '%s' at %.1f Hz.",
                cloud->size(), pcd_path.c_str(), topic.c_str(), rate_hz);
  }

private:
  void timerCallback()
  {
    ros_cloud_.header.stamp = this->now();
    pub_->publish(ros_cloud_);
  }

  sensor_msgs::msg::PointCloud2 ros_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDPublisher>());
  rclcpp::shutdown();
  return 0;
}
