#ifndef PEDESTRIAN_COSTMAP_LAYER__PEDESTRIAN_LAYER_HPP_
#define PEDESTRIAN_COSTMAP_LAYER__PEDESTRIAN_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace pedestrian_costmap_layer
{

class PedestrianLayer : public nav2_costmap_2d::Layer
{
public:
  PedestrianLayer();

  void onInitialize() override;

  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  void reset() override;
  bool isClearable() override;

private:
  struct MarkPoint
  {
    double x;
    double y;
    double radius;
    unsigned char cost;
  };

  void currentCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void predictedCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  void addPoseArrayToPoints(
    const geometry_msgs::msg::PoseArray::SharedPtr & msg,
    double base_radius,
    unsigned char base_cost,
    bool is_predicted,
    std::vector<MarkPoint> & points);

  void drawDisc(
    nav2_costmap_2d::Costmap2D & master_grid,
    const MarkPoint & point);

  bool enabled_;
  double current_radius_;
  double predicted_radius_;
  int current_cost_;
  int predicted_cost_;
  double transform_tolerance_;
  
  double time_decay_factor_;
  double radius_expansion_rate_;
  double msg_timeout_;

  std::string current_topic_;
  std::string predicted_topic_;
  std::string global_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr current_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr predicted_sub_;

  geometry_msgs::msg::PoseArray::SharedPtr latest_current_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_predicted_;
  rclcpp::Time last_msg_time_;

  std::vector<MarkPoint> points_to_apply_;
  std::mutex mutex_;
};

}  // namespace pedestrian_costmap_layer

#endif  // PEDESTRIAN_COSTMAP_LAYER__PEDESTRIAN_LAYER_HPP_
