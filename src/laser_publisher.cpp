#include <chrono>
#include <memory>
#include <vector>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LaserPublisher : public rclcpp::Node
{
public:
  LaserPublisher()
  : Node("laser_publisher")
  {
    publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LaserPublisher::publishScan, this));
    static_transformer = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    staticTransformer();
  }

private:
  void staticTransformer()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    static_transformer->sendTransform(t);
  }
  void publishScan()
  {
    auto now = this->get_clock()->now();

    sensor_msgs::msg::LaserScan laser_scan;
    laser_scan.header.stamp = now;
    laser_scan.header.frame_id = "base_link";

    laser_scan.angle_min = -M_PI;
    laser_scan.angle_max = M_PI;
    laser_scan.angle_increment = M_PI / 180.0; // 1 grado
    laser_scan.range_min = 0.1;
    laser_scan.range_max = 6.0;

    int num_rays = static_cast<int>((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1;
    laser_scan.ranges.resize(num_rays, std::numeric_limits<float>::infinity());

    laser_scan.ranges[num_rays/2] = 1.0;   
    laser_scan.ranges[num_rays/4] = 2.0; 
    laser_scan.ranges[3 * num_rays/4] = 2.0;  

    publisher->publish(laser_scan);
    RCLCPP_INFO(this->get_logger(), "Scan pusblished");
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transformer;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserPublisher>());
  rclcpp::shutdown();
  return 0;
}
