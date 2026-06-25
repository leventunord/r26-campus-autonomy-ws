#ifndef PEDESTRIAN_MPPI_CRITIC__DYNAMIC_OBSTACLE_CRITIC_HPP_
#define PEDESTRIAN_MPPI_CRITIC__DYNAMIC_OBSTACLE_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi::critics
{

class DynamicObstacleCritic : public CriticFunction
{
public:
  DynamicObstacleCritic() = default;
  ~DynamicObstacleCritic() override = default;

  void initialize() override;
  void score(CriticData & data) override;

private:
  void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Parameters
  int power_{1};
  float weight_{15.0f};
  float safe_distance_{1.0f};
  std::string topic_name_{"/pedestrian/predicted_poses"};

  // ROS Subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_predictions_;
  bool has_predictions_{false};
  std::mutex mutex_;
};

}  // namespace mppi::critics

#endif  // PEDESTRIAN_MPPI_CRITIC__DYNAMIC_OBSTACLE_CRITIC_HPP_
