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
    obstacle_timer = this->create_wall_timer(std::chrono::seconds(7), [this]() { generateObstacles(20); });
  }

private:

  struct Obstacle
  {
    float x;
    float y;
    rclcpp::Time timestamp;
  };
  void generateObstacles(int num_obstacles)
  {
    rclcpp::Time now = this->now();
    for (int i = 0; i < num_obstacles; ++i)
    {
      float angle = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
      float distance = 2.0f + static_cast<float>(rand()) / RAND_MAX * 4.0f;
      float x = std::cos(angle) * distance;
      float y = std::sin(angle) * distance;
      obstacles.push_back({x, y, now});
    }
  }
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
  
    for (auto i = obstacles.begin(); i != obstacles.end(); )
    {
      if ((now - i->timestamp).seconds() > 10.0)
      {
        i = obstacles.erase(i);
      }
      else
      {
        ++i;
      }
    }
    sensor_msgs::msg::LaserScan laser_scan;
    laser_scan.header.stamp = now;
    laser_scan.header.frame_id = "base_link";

    laser_scan.angle_min = -M_PI;
    laser_scan.angle_max = M_PI;
    laser_scan.angle_increment = M_PI / 180.0; 
    laser_scan.range_min = 0.1;
    laser_scan.range_max = 6.0;

    int num_rays = static_cast<int>((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1;
    laser_scan.ranges.resize(num_rays, std::numeric_limits<float>::infinity());
  
    for (int i = 0; i < num_rays; ++i)
    {
      double angle = laser_scan.angle_min + i * laser_scan.angle_increment;
      double dist_x = std::cos(angle);
      double dist_y = std::sin(angle);
      double min_distance = laser_scan.range_max;

      for (const auto& obs : obstacles)
      {
        double obs_x = obs.x;
        double obs_y = obs.y;

        double projection = obs_x * dist_x + obs_y * dist_y;
        if (projection < 0) continue;

        double closest_x = projection * dist_x;
        double closest_y = projection * dist_y;
        double perp_dist = std::hypot(obs_x - closest_x, obs_y - closest_y);

        if (perp_dist < 0.2)
        {
          double dist_to_obs = std::hypot(obs_x, obs_y);
          if (dist_to_obs < min_distance)
            min_distance = dist_to_obs;
        }
      }

      if (min_distance < laser_scan.range_max)
      {
        double noise = 0.02 * ((rand() % 100) / 100.0 - 0.5);
        laser_scan.ranges[i] = static_cast<float>(min_distance + noise);
      }
    }
    publisher->publish(laser_scan);
    RCLCPP_INFO(this->get_logger(), "Scan pusblished");
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transformer;
  std::vector<Obstacle> obstacles;
  rclcpp::TimerBase::SharedPtr obstacle_timer;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserPublisher>());
  rclcpp::shutdown();
  return 0;
}
