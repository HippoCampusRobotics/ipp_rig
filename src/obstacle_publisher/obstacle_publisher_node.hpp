#pragma once

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/polygon.hpp>
#include <ipp/msg/polygons_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ipp {

class ObstaclePublisherNode : public rclcpp::Node {
 public:
  ObstaclePublisherNode();

 private:
  void OnTimer();
  std::string GetScenarioFilePath(const std::string &scenario_name) const;
  bool LoadObstacles(const std::string &file_path);

  std::vector<geometry_msgs::msg::Polygon> obstacles_;

  rclcpp::Publisher<ipp::msg::PolygonsStamped>::SharedPtr obstacles_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ipp
