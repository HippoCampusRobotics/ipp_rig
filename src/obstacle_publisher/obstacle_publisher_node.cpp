#include "obstacle_publisher_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ipp {

static constexpr double kDepth = -0.5;
static constexpr int kPublishRateMs = 100;

ObstaclePublisherNode::ObstaclePublisherNode() : Node("obstacle_publisher") {
  declare_parameter<std::string>("scenario", "");

  std::string scenario = get_parameter("scenario").as_string();
  if (!scenario.empty()) {
    std::string file_path = GetScenarioFilePath(scenario);
    if (!LoadObstacles(file_path)) {
      RCLCPP_FATAL(get_logger(), "Failed to load obstacles from '%s'.",
                   file_path.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu obstacle(s) from scenario '%s'.",
                obstacles_.size(), scenario.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "No scenario provided. Running without obstacles.");
  }

  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(1);
  obstacles_pub_ =
      create_publisher<ipp::msg::PolygonsStamped>("obstacles", qos);

  timer_ = create_timer(std::chrono::milliseconds(kPublishRateMs),
                        [this]() { OnTimer(); });
}

std::string ObstaclePublisherNode::GetScenarioFilePath(
    const std::string &scenario_name) const {
  std::string share_dir =
      ament_index_cpp::get_package_share_directory("ipp");
  return share_dir + "/config/" + scenario_name + ".yaml";
}

void ObstaclePublisherNode::OnTimer() {
  ipp::msg::PolygonsStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.polygons = obstacles_;
  obstacles_pub_->publish(msg);
}

bool ObstaclePublisherNode::LoadObstacles(const std::string &file_path) {
  YAML::Node node;
  try {
    node = YAML::LoadFile(file_path);
  } catch (const YAML::BadFile &) {
    RCLCPP_FATAL(get_logger(), "Could not open file '%s'.", file_path.c_str());
    return false;
  } catch (const YAML::ParserException &) {
    RCLCPP_FATAL(get_logger(), "Malformed YAML in '%s'.", file_path.c_str());
    return false;
  }

  obstacles_.clear();
  for (const YAML::Node &obstacle : node["obstacles"]) {
    geometry_msgs::msg::Polygon polygon;
    for (const YAML::Node &corner : obstacle["corners"]) {
      auto xy = corner.as<std::vector<double>>();
      geometry_msgs::msg::Point32 p;
      p.x = xy.at(0);
      p.y = xy.at(1);
      p.z = kDepth;
      polygon.points.push_back(p);
    }
    polygon.points.push_back(polygon.points.front());
    obstacles_.push_back(polygon);
  }
  return true;
}

}  // namespace ipp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ipp::ObstaclePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
