/**
 * @file position_fusion_node.cpp
 * @brief PCL-based pedestrian position fusion node.
 *
 * Pipeline (per YOLO direction):
 *   1. Extract points within the angular sector indicated by YOLO
 *   2. RANSAC plane fitting → remove ground points
 *   3. Euclidean clustering on remaining "floating" points
 *   4. 3D bounding-box rule filter → keep only pedestrian-sized clusters
 *   5. Publish the centroid of each valid cluster as a detected pedestrian pose
 *
 * Input topics:
 *   - /yolo/person_directions  (visualization_msgs/MarkerArray)
 *   - /lidar_points            (sensor_msgs/PointCloud2)
 *
 * Output topics:
 *   - /pedestrian/detected_poses    (geometry_msgs/PoseArray)
 *   - /pedestrian/position_markers  (visualization_msgs/MarkerArray)
 */

#include <cmath>
#include <deque>
#include <memory>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

// ─── Utility ───

static inline double angle_diff(double a, double b) {
    double d = a - b;
    d = std::fmod(d + M_PI, 2.0 * M_PI);
    if (d < 0.0) d += 2.0 * M_PI;
    return d - M_PI;
}

// ─── Stamped cloud buffer entry ───

struct StampedCloud {
    double stamp_sec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

// ─── Detection result ───

struct DetectionResult {
    double x, y, z;
    double yaw;
    int num_points;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Node
// ═══════════════════════════════════════════════════════════════════════════════

class PositionFusionNode : public rclcpp::Node {
public:
    PositionFusionNode() : Node("pedestrian_position_fusion_node") {
        // ── Topic parameters ──
        this->declare_parameter("directions_topic", std::string("/yolo/person_directions"));
        this->declare_parameter("lidar_topic",      std::string("/lidar_points"));
        this->declare_parameter("output_frame",     std::string("base_link"));
        this->declare_parameter("lidar_frame",      std::string("hesai_lidar"));

        // ── Static transform: hesai_lidar → base_link ──
        this->declare_parameter("lidar_to_base_x",   0.34058);
        this->declare_parameter("lidar_to_base_y",   0.0);
        this->declare_parameter("lidar_to_base_z",   0.3465);
        this->declare_parameter("lidar_to_base_yaw", M_PI / 2.0);

        // ── Range filtering ──
        this->declare_parameter("min_range", 0.3);
        this->declare_parameter("max_range", 15.0);

        // ── Angular gating ──
        this->declare_parameter("sector_half_angle", 0.25);  // ~14° half-width of search sector

        // ── RANSAC ground removal ──
        this->declare_parameter("ransac_distance_threshold", 0.15);  // metres
        this->declare_parameter("ransac_max_iterations", 200);

        // ── Euclidean clustering ──
        this->declare_parameter("cluster_tolerance", 0.25);  // metres
        this->declare_parameter("cluster_min_points", 5);
        this->declare_parameter("cluster_max_points", 2000);

        // ── Pedestrian bounding-box rules ──
        this->declare_parameter("ped_min_width",  0.15);   // metres (x-y extent)
        this->declare_parameter("ped_max_width",  1.2);
        this->declare_parameter("ped_min_height", 0.3);    // metres (z extent)
        this->declare_parameter("ped_max_height", 2.2);

        // ── Output NMS ──
        this->declare_parameter("nms_distance", 0.6);

        // ── Read parameters ──
        directions_topic_ = this->get_parameter("directions_topic").as_string();
        lidar_topic_      = this->get_parameter("lidar_topic").as_string();
        output_frame_     = this->get_parameter("output_frame").as_string();
        lidar_frame_      = this->get_parameter("lidar_frame").as_string();

        lidar_to_base_x_   = this->get_parameter("lidar_to_base_x").as_double();
        lidar_to_base_y_   = this->get_parameter("lidar_to_base_y").as_double();
        lidar_to_base_z_   = this->get_parameter("lidar_to_base_z").as_double();
        lidar_to_base_yaw_ = this->get_parameter("lidar_to_base_yaw").as_double();

        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();

        sector_half_angle_ = this->get_parameter("sector_half_angle").as_double();

        ransac_dist_thresh_ = this->get_parameter("ransac_distance_threshold").as_double();
        ransac_max_iter_    = this->get_parameter("ransac_max_iterations").as_int();

        cluster_tolerance_  = this->get_parameter("cluster_tolerance").as_double();
        cluster_min_points_ = static_cast<int>(this->get_parameter("cluster_min_points").as_int());
        cluster_max_points_ = static_cast<int>(this->get_parameter("cluster_max_points").as_int());

        ped_min_width_  = this->get_parameter("ped_min_width").as_double();
        ped_max_width_  = this->get_parameter("ped_max_width").as_double();
        ped_min_height_ = this->get_parameter("ped_min_height").as_double();
        ped_max_height_ = this->get_parameter("ped_max_height").as_double();

        nms_distance_ = this->get_parameter("nms_distance").as_double();

        // ── Subscriptions ──
        auto sensor_qos = rclcpp::QoS(2).reliable();
        auto normal_qos = rclcpp::QoS(10);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, sensor_qos,
            std::bind(&PositionFusionNode::cloud_callback, this, std::placeholders::_1));

        dir_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            directions_topic_, normal_qos,
            std::bind(&PositionFusionNode::direction_callback, this, std::placeholders::_1));

        // ── Publishers ──
        pose_pub_   = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/pedestrian/detected_poses", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pedestrian/position_markers", 10);

        RCLCPP_INFO(this->get_logger(),
            "PositionFusionNode (PCL) started: %s + %s → /pedestrian/detected_poses",
            directions_topic_.c_str(), lidar_topic_.c_str());
    }

private:
    // ─────────────────────── Point Cloud Handling ───────────────────────

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Transform from lidar frame to base_link if needed
        bool needs_transform = (msg->header.frame_id == lidar_frame_ &&
                                lidar_frame_ != output_frame_);

        // Range filter + transform in one pass
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        filtered->reserve(pcl_cloud->size());

        const double cos_yaw = std::cos(lidar_to_base_yaw_);
        const double sin_yaw = std::sin(lidar_to_base_yaw_);

        for (const auto& pt : *pcl_cloud) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;

            double r = std::hypot(pt.x, pt.y);
            if (r < min_range_ || r > max_range_)
                continue;

            pcl::PointXYZ out;
            if (needs_transform) {
                out.x = static_cast<float>(cos_yaw * pt.x - sin_yaw * pt.y + lidar_to_base_x_);
                out.y = static_cast<float>(sin_yaw * pt.x + cos_yaw * pt.y + lidar_to_base_y_);
                out.z = static_cast<float>(pt.z + lidar_to_base_z_);
            } else {
                out = pt;
            }
            filtered->push_back(out);
        }

        double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        cloud_buffer_.push_back({stamp_sec, filtered});
        if (cloud_buffer_.size() > 20) {
            cloud_buffer_.pop_front();
        }
    }

    // ─────────────────── Direction (MarkerArray) Handling ───────────────────

    void direction_callback(
        const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        if (msg->markers.empty()) return;

        // Find first valid timestamp
        double stamp_sec = 0.0;
        for (const auto& m : msg->markers) {
            if (m.header.stamp.sec != 0 || m.header.stamp.nanosec != 0) {
                stamp_sec = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
                break;
            }
        }
        if (stamp_sec == 0.0) return;

        // Find closest cloud in time
        pcl::PointCloud<pcl::PointXYZ>::Ptr best_cloud = nullptr;
        double min_diff = std::numeric_limits<double>::infinity();

        for (const auto& entry : cloud_buffer_) {
            double diff = std::abs(entry.stamp_sec - stamp_sec);
            if (diff < min_diff) {
                min_diff   = diff;
                best_cloud = entry.cloud;
            }
        }

        if (!best_cloud || min_diff > 0.2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "No matching cloud (time diff: %.3f s). Skipping.", min_diff);
            return;
        }

        // Extract yaw values from person_direction markers
        std::vector<double> yaw_list;
        for (const auto& marker : msg->markers) {
            if (marker.ns == "robot_front") continue;
            if (marker.action == visualization_msgs::msg::Marker::DELETEALL) continue;
            if (marker.ns != "person_direction") continue;

            double qz = marker.pose.orientation.z;
            double qw = marker.pose.orientation.w;
            double yaw = 2.0 * std::atan2(qz, qw);
            yaw_list.push_back(yaw);
        }

        // Build the output stamp from the first valid person_direction marker
        builtin_interfaces::msg::Time out_stamp;
        for (const auto& m : msg->markers) {
            if (m.ns != "robot_front" &&
                m.action != visualization_msgs::msg::Marker::DELETEALL) {
                out_stamp = m.header.stamp;
                break;
            }
        }

        if (yaw_list.empty()) {
            // No person — publish empty PoseArray
            auto empty = geometry_msgs::msg::PoseArray();
            empty.header.frame_id = output_frame_;
            empty.header.stamp    = out_stamp;
            pose_pub_->publish(empty);

            // Clear previous sphere markers
            auto empty_markers = visualization_msgs::msg::MarkerArray();
            visualization_msgs::msg::Marker del;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            del.ns     = "pedestrian_pos";
            empty_markers.markers.push_back(del);
            marker_pub_->publish(empty_markers);

            return;
        }

        // ── Per-direction fusion ──
        std::vector<DetectionResult> raw_results;
        for (double yaw : yaw_list) {
            auto result = fuse_single_direction(best_cloud, yaw);
            if (result.has_value()) {
                raw_results.push_back(result.value());
            }
        }

        // ── NMS ──
        auto final_results = nms(raw_results);

        // ── Publish PoseArray ──
        auto pose_array = geometry_msgs::msg::PoseArray();
        pose_array.header.frame_id = output_frame_;
        pose_array.header.stamp    = out_stamp;

        for (const auto& det : final_results) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = det.x;
            pose.position.y = det.y;
            pose.position.z = std::max(0.0, det.z);
            pose.orientation.z = std::sin(det.yaw / 2.0);
            pose.orientation.w = std::cos(det.yaw / 2.0);
            pose_array.poses.push_back(pose);
        }
        pose_pub_->publish(pose_array);

        // ── Publish sphere markers ──
        auto marker_array = visualization_msgs::msg::MarkerArray();

        // Delete all previous markers
        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        del.ns     = "pedestrian_pos";
        marker_array.markers.push_back(del);

        for (size_t i = 0; i < final_results.size(); ++i) {
            const auto& det = final_results[i];
            visualization_msgs::msg::Marker sphere;
            sphere.header   = pose_array.header;
            sphere.ns       = "pedestrian_pos";
            sphere.id       = static_cast<int>(i);
            sphere.type     = visualization_msgs::msg::Marker::SPHERE;
            sphere.action   = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x    = det.x;
            sphere.pose.position.y    = det.y;
            sphere.pose.position.z    = std::max(0.0, det.z);
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = 0.4;
            sphere.scale.y = 0.4;
            sphere.scale.z = 0.4;
            sphere.color.a = 0.85f;
            sphere.color.r = 0.2f;
            sphere.color.g = 0.4f;
            sphere.color.b = 1.0f;
            marker_array.markers.push_back(sphere);
        }
        marker_pub_->publish(marker_array);
    }

    // ─────────────────── Core Fusion Pipeline ───────────────────

    std::optional<DetectionResult> fuse_single_direction(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double yaw_guess)
    {
        // ── Step 1: Extract points within the angular sector ──
        auto sector_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (const auto& pt : *cloud) {
            double pt_yaw = std::atan2(pt.y, pt.x);
            if (std::abs(angle_diff(pt_yaw, yaw_guess)) <= sector_half_angle_) {
                sector_cloud->push_back(pt);
            }
        }

        if (sector_cloud->size() < static_cast<size_t>(cluster_min_points_)) {
            return std::nullopt;
        }

        // ── Step 2: RANSAC plane fitting → remove ground ──
        auto non_ground = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if (sector_cloud->size() >= 10) {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_dist_thresh_);
            seg.setMaxIterations(ransac_max_iter_);
            seg.setInputCloud(sector_cloud);
            seg.segment(*inliers, *coefficients);

            if (!inliers->indices.empty()) {
                // Extract non-ground (invert selection)
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(sector_cloud);
                extract.setIndices(inliers);
                extract.setNegative(true);  // keep points NOT on the plane
                extract.filter(*non_ground);
            } else {
                // No plane found — use all points
                *non_ground = *sector_cloud;
            }
        } else {
            *non_ground = *sector_cloud;
        }

        if (non_ground->size() < static_cast<size_t>(cluster_min_points_)) {
            return std::nullopt;
        }

        // ── Step 3: Euclidean clustering ──
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(non_ground);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_min_points_);
        ec.setMaxClusterSize(cluster_max_points_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(non_ground);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
            return std::nullopt;
        }

        // ── Step 4: Filter clusters by pedestrian bounding-box rules ──
        struct ClusterCandidate {
            double cx, cy, cz;       // centroid
            double yaw;              // direction from origin
            double dist;             // horizontal distance
            int    n_points;
        };

        std::vector<ClusterCandidate> valid_candidates;

        for (const auto& indices : cluster_indices) {
            // Compute 3D bounding box
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::lowest();

            double sum_x = 0, sum_y = 0, sum_z = 0;
            int n = static_cast<int>(indices.indices.size());

            for (int idx : indices.indices) {
                const auto& pt = (*non_ground)[idx];
                sum_x += pt.x;
                sum_y += pt.y;
                sum_z += pt.z;
                min_x = std::min(min_x, pt.x);
                max_x = std::max(max_x, pt.x);
                min_y = std::min(min_y, pt.y);
                max_y = std::max(max_y, pt.y);
                min_z = std::min(min_z, pt.z);
                max_z = std::max(max_z, pt.z);
            }

            double dx = max_x - min_x;
            double dy = max_y - min_y;
            double dz = max_z - min_z;
            double width = std::max(dx, dy);  // horizontal extent

            // Pedestrian size rules
            if (width < ped_min_width_ || width > ped_max_width_) continue;
            if (dz    < ped_min_height_ || dz   > ped_max_height_) continue;

            double cx = sum_x / n;
            double cy = sum_y / n;
            double cz = sum_z / n;
            double c_yaw = std::atan2(cy, cx);
            double c_dist = std::hypot(cx, cy);

            valid_candidates.push_back({cx, cy, cz, c_yaw, c_dist, n});
        }

        if (valid_candidates.empty()) {
            return std::nullopt;
        }

        // Choose the candidate whose direction best matches YOLO's yaw guess
        // (tie-break by distance — prefer closer)
        auto best = std::min_element(valid_candidates.begin(), valid_candidates.end(),
            [&](const ClusterCandidate& a, const ClusterCandidate& b) {
                double sa = std::abs(angle_diff(a.yaw, yaw_guess)) * 5.0 + a.dist * 0.1;
                double sb = std::abs(angle_diff(b.yaw, yaw_guess)) * 5.0 + b.dist * 0.1;
                return sa < sb;
            });

        return DetectionResult{
            best->cx, best->cy, best->cz,
            best->yaw, best->n_points
        };
    }

    // ─────────────────── NMS De-duplication ───────────────────

    std::vector<DetectionResult> nms(std::vector<DetectionResult>& results) {
        if (results.size() <= 1) return results;

        // Sort by distance (closer first)
        std::sort(results.begin(), results.end(),
            [](const DetectionResult& a, const DetectionResult& b) {
                return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
            });

        std::vector<DetectionResult> keep;
        for (const auto& candidate : results) {
            double c_yaw = std::atan2(candidate.y, candidate.x);
            bool is_dup = false;

            for (const auto& kept : keep) {
                double k_yaw = std::atan2(kept.y, kept.x);
                double dist = std::hypot(candidate.x - kept.x, candidate.y - kept.y);
                double a_diff = std::abs(angle_diff(c_yaw, k_yaw));

                if (dist < nms_distance_ || a_diff < sector_half_angle_) {
                    is_dup = true;
                    break;
                }
            }

            if (!is_dup) {
                keep.push_back(candidate);
            }
        }
        return keep;
    }

    // ─────────────────── Member Variables ───────────────────

    // Parameters
    std::string directions_topic_;
    std::string lidar_topic_;
    std::string output_frame_;
    std::string lidar_frame_;

    double lidar_to_base_x_, lidar_to_base_y_, lidar_to_base_z_, lidar_to_base_yaw_;
    double min_range_, max_range_;
    double sector_half_angle_;
    double ransac_dist_thresh_;
    int    ransac_max_iter_;
    double cluster_tolerance_;
    int    cluster_min_points_, cluster_max_points_;
    double ped_min_width_, ped_max_width_;
    double ped_min_height_, ped_max_height_;
    double nms_distance_;

    // Cloud buffer
    std::deque<StampedCloud> cloud_buffer_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr          cloud_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr   dir_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr             pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr      marker_pub_;
};

// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
