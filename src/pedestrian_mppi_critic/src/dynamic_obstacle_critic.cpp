#include "pedestrian_mppi_critic/dynamic_obstacle_critic.hpp"

#include <cmath>
#include <algorithm>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi::critics
{

void DynamicObstacleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 15.0f);
  getParam(safe_distance_, "safe_distance", 1.0f);
  getParam(topic_name_, "topic_name", std::string("/pedestrian/predicted_poses"));

  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("LifecycleNode parent is null!");
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    topic_name_,
    qos,
    std::bind(&DynamicObstacleCritic::callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(
    logger_,
    "DynamicObstacleCritic: initialized. weight: %f, safe_distance: %f, topic: %s",
    weight_, safe_distance_, topic_name_.c_str());
}

void DynamicObstacleCritic::callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_predictions_ = msg;
  has_predictions_ = true;
}

void DynamicObstacleCritic::score(CriticData & data)
{
  if (!enabled_ || !has_predictions_ || weight_ == 0.0f) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_predictions_ || latest_predictions_->poses.empty()) {
    return;
  }

  // Determine frame transform if needed
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  std::string source_frame = latest_predictions_->header.frame_id;
  if (source_frame.empty()) {
    source_frame = "odom"; // fallback
  }

  bool need_transform = (global_frame != source_frame);
  geometry_msgs::msg::TransformStamped tf;
  if (need_transform) {
    try {
      tf = costmap_ros_->getTfBuffer()->lookupTransform(
        global_frame,
        source_frame,
        tf2::TimePointZero
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *parent_.lock()->get_clock(),
        2000,
        "DynamicObstacleCritic: cannot transform from [%s] to [%s]: %s",
        source_frame.c_str(),
        global_frame.c_str(),
        ex.what());
      return;
    }
  }

  struct PredPoint {
    double x;
    double y;
    double t;
  };

  std::vector<PredPoint> pred_points;
  pred_points.reserve(latest_predictions_->poses.size());

  for (const auto & pose : latest_predictions_->poses) {
    double x = pose.position.x;
    double y = pose.position.y;
    double t = pose.position.z; // Time encoded in z

    if (need_transform) {
      geometry_msgs::msg::PoseStamped in;
      geometry_msgs::msg::PoseStamped out;
      in.header.frame_id = source_frame;
      in.pose = pose;
      tf2::doTransform(in, out, tf);
      x = out.pose.position.x;
      y = out.pose.position.y;
    }

    pred_points.push_back({x, y, t});
  }

  const auto & trajectories = data.trajectories;
  size_t batch_size = trajectories.x.shape()[0];
  size_t time_steps = trajectories.x.shape()[1];
  double model_dt = data.model_dt;

  // Pre-associate predicted pedestrian points with trajectory timesteps
  // relevant_pedestrians_per_step[k] will contain all points whose time is within [k*dt - 0.25, k*dt + 0.25]
  std::vector<std::vector<PredPoint>> relevant_pedestrians_per_step(time_steps);
  for (size_t k = 0; k < time_steps; ++k) {
    double t_r = k * model_dt;
    for (const auto & p : pred_points) {
      if (std::abs(t_r - p.t) < 0.25) {
        relevant_pedestrians_per_step[k].push_back(p);
      }
    }
  }

  // Calculate scores
  for (size_t b = 0; b < batch_size; ++b) {
    float penalty = 0.0f;
    for (size_t k = 0; k < time_steps; ++k) {
      if (relevant_pedestrians_per_step[k].empty()) {
        continue;
      }

      double rx = trajectories.x(b, k);
      double ry = trajectories.y(b, k);

      for (const auto & p : relevant_pedestrians_per_step[k]) {
        double dx = rx - p.x;
        double dy = ry - p.y;
        double dist = std::hypot(dx, dy);

        if (dist < safe_distance_) {
          float ratio = (safe_distance_ - dist) / safe_distance_;
          penalty += std::pow(ratio, power_);
        }
      }
    }

    if (penalty > 0.0f) {
      data.costs[b] += weight_ * penalty;
    }
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicObstacleCritic, mppi::critics::CriticFunction)
