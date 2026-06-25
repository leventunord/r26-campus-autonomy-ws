#include "pedestrian_costmap_layer/pedestrian_layer.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pedestrian_costmap_layer
{

PedestrianLayer::PedestrianLayer()
: enabled_(true),
  current_radius_(0.60),
  predicted_radius_(0.40),
  current_cost_(254),
  predicted_cost_(200),
  transform_tolerance_(0.2),
  time_decay_factor_(0.4),
  radius_expansion_rate_(0.05),
  msg_timeout_(1.0),
  last_msg_time_(0)
{
}

void PedestrianLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("current_topic", rclcpp::ParameterValue(std::string("/pedestrian/current_poses")));
  declareParameter("predicted_topic", rclcpp::ParameterValue(std::string("/pedestrian/predicted_poses")));
  declareParameter("current_radius", rclcpp::ParameterValue(0.60));
  declareParameter("predicted_radius", rclcpp::ParameterValue(0.40));
  declareParameter("current_cost", rclcpp::ParameterValue(254));
  declareParameter("predicted_cost", rclcpp::ParameterValue(200));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  declareParameter("time_decay_factor", rclcpp::ParameterValue(0.5));
  declareParameter("radius_expansion_rate", rclcpp::ParameterValue(0.05));
  declareParameter("msg_timeout", rclcpp::ParameterValue(1.5));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".current_topic", current_topic_);
  node->get_parameter(name_ + ".predicted_topic", predicted_topic_);
  node->get_parameter(name_ + ".current_radius", current_radius_);
  node->get_parameter(name_ + ".predicted_radius", predicted_radius_);
  node->get_parameter(name_ + ".current_cost", current_cost_);
  node->get_parameter(name_ + ".predicted_cost", predicted_cost_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".time_decay_factor", time_decay_factor_);
  node->get_parameter(name_ + ".radius_expansion_rate", radius_expansion_rate_);
  node->get_parameter(name_ + ".msg_timeout", msg_timeout_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  last_msg_time_ = node->now();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  current_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    current_topic_,
    qos,
    std::bind(&PedestrianLayer::currentCallback, this, std::placeholders::_1));

  predicted_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    predicted_topic_,
    qos,
    std::bind(&PedestrianLayer::predictedCallback, this, std::placeholders::_1));

  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "PedestrianLayer initialized in frame [%s], current_topic=[%s], predicted_topic=[%s]",
    global_frame_.c_str(),
    current_topic_.c_str(),
    predicted_topic_.c_str());
}

void PedestrianLayer::currentCallback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_current_ = msg;
  if (auto node = node_.lock()) {
    last_msg_time_ = node->now();
  }
}

void PedestrianLayer::predictedCallback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_predicted_ = msg;
  if (auto node = node_.lock()) {
    last_msg_time_ = node->now();
  }
}

void PedestrianLayer::addPoseArrayToPoints(
  const geometry_msgs::msg::PoseArray::SharedPtr & msg,
  double base_radius,
  unsigned char base_cost,
  bool is_predicted,
  std::vector<MarkPoint> & points)
{
  if (!msg) {
    return;
  }

  std::string source_frame = msg->header.frame_id;
  if (source_frame.empty()) {
    source_frame = "base_link";
  }

  geometry_msgs::msg::TransformStamped tf;

  try {
    tf = tf_->lookupTransform(
      global_frame_,
      source_frame,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      2000,
      "PedestrianLayer cannot transform from [%s] to [%s]: %s",
      source_frame.c_str(),
      global_frame_.c_str(),
      ex.what());
    return;
  }

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped in;
    geometry_msgs::msg::PoseStamped out;

    in.header.frame_id = source_frame;
    in.header.stamp.sec = 0;
    in.header.stamp.nanosec = 0;
    in.pose = msg->poses[i];

    tf2::doTransform(in, out, tf);

    double r = base_radius;
    unsigned char c = base_cost;

    if (is_predicted) {
      // 从输入消息的 Z 坐标中提取预测时间 t
      double t = msg->poses[i].position.z;
      // 容错处理：如果上游未正确设置 z，则给一个默认值
      if (t <= 0.0) {
        t = 0.5;
      }
      
      // 代价按时间指数衰减
      double decayed_cost = static_cast<double>(base_cost) * std::exp(-time_decay_factor_ * t);
      c = static_cast<unsigned char>(std::clamp(static_cast<int>(decayed_cost), 0, 254));
      
      // 半径随时间线性扩大，表示不确定性增加
      r = base_radius + radius_expansion_rate_ * t;
      
      // 如果经过衰减后代价值太低（<5），就没有必要再画了，以节省计算资源
      if (c < 5) {
        continue;
      }
    }

    MarkPoint p;
    p.x = out.pose.position.x;
    p.y = out.pose.position.y;
    p.radius = r;
    p.cost = c;

    points.push_back(p);
  }
}

void PedestrianLayer::updateBounds(
  double /*robot_x*/,
  double /*robot_y*/,
  double /*robot_yaw*/,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto node = node_.lock();
  if (!node) {
    return;
  }

  // 超时保护机制：防止因为上游节点崩溃而残留永远不清除的障碍物
  if ((node->now() - last_msg_time_).seconds() > msg_timeout_) {
    latest_current_.reset();
    latest_predicted_.reset();
  }

  points_to_apply_.clear();

  addPoseArrayToPoints(
    latest_predicted_,
    predicted_radius_,
    static_cast<unsigned char>(std::clamp(predicted_cost_, 0, 254)),
    true, // is_predicted
    points_to_apply_);

  addPoseArrayToPoints(
    latest_current_,
    current_radius_,
    static_cast<unsigned char>(std::clamp(current_cost_, 0, 254)),
    false, // is_predicted
    points_to_apply_);

  for (const auto & p : points_to_apply_) {
    *min_x = std::min(*min_x, p.x - p.radius);
    *min_y = std::min(*min_y, p.y - p.radius);
    *max_x = std::max(*max_x, p.x + p.radius);
    *max_y = std::max(*max_y, p.y + p.radius);
  }
}

void PedestrianLayer::drawDisc(
  nav2_costmap_2d::Costmap2D & master_grid,
  const MarkPoint & point)
{
  unsigned int center_mx;
  unsigned int center_my;

  if (!master_grid.worldToMap(point.x, point.y, center_mx, center_my)) {
    return;
  }

  const double resolution = master_grid.getResolution();
  const int radius_cells = static_cast<int>(std::ceil(point.radius / resolution));

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double dist = std::sqrt(
        std::pow(dx * resolution, 2.0) +
        std::pow(dy * resolution, 2.0));

      if (dist > point.radius) {
        continue;
      }

      const int mx_i = static_cast<int>(center_mx) + dx;
      const int my_i = static_cast<int>(center_my) + dy;

      if (mx_i < 0 || my_i < 0) {
        continue;
      }

      const unsigned int mx = static_cast<unsigned int>(mx_i);
      const unsigned int my = static_cast<unsigned int>(my_i);

      if (mx >= master_grid.getSizeInCellsX() ||
          my >= master_grid.getSizeInCellsY()) {
        continue;
      }

      const unsigned char old_cost = master_grid.getCost(mx, my);
      const unsigned char new_cost = std::max(old_cost, point.cost);

      master_grid.setCost(mx, my, new_cost);
    }
  }
}

void PedestrianLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/,
  int /*min_j*/,
  int /*max_i*/,
  int /*max_j*/)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & point : points_to_apply_) {
    drawDisc(master_grid, point);
  }
}

void PedestrianLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_to_apply_.clear();
  latest_current_.reset();
  latest_predicted_.reset();
  if (auto node = node_.lock()) {
    last_msg_time_ = node->now();
  }
  current_ = true;
}

bool PedestrianLayer::isClearable()
{
  return false;
}

}  // namespace pedestrian_costmap_layer

PLUGINLIB_EXPORT_CLASS(
  pedestrian_costmap_layer::PedestrianLayer,
  nav2_costmap_2d::Layer)
