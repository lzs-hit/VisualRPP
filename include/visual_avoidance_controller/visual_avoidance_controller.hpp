// Copyright 2026, lzs. Apache-2.0
//
// VisualAvoidanceController: a Nav2 controller plugin that wraps the stock
// RegulatedPurePursuitController and injects visual-based reactive obstacle
// avoidance. The avoidance algorithm and state machine are ported from the
// senior-team's `avoid_zjkpkg1/src/pidf.cpp` (Cone_Judgement + Cone_Judgement_qr
// + avoidObstacle / avoidObstacle_right / calDx / calDy).

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"

namespace visual_avoidance_controller
{

// Single obstacle expressed in the robot base_link frame (x = right, y = forward).
struct ObstaclePoint
{
  double x;
  double y;
  ObstaclePoint(double x_ = 0.0, double y_ = 1e3) : x(x_), y(y_) {}
};

// Three-state machine, following the spirit of the senior code's
// state machine (NORMAL / AVOIDING / RECOVERY).
enum class State
{
  NORMAL = 0,    // delegate to RegulatedPurePursuitController
  AVOIDING = 1,  // compute avoidance cmd_vel locally
  RECOVERY = 2,  // half-speed delegate to RPP, wait for obstacle to clear
};

class VisualAvoidanceController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  VisualAvoidanceController() = default;
  ~VisualAvoidanceController() override = default;

  // nav2_core::Controller interface (matches RegulatedPurePursuitController).
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  // ROS plumbing.
  void obstacleCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Pure helpers (no ROS) -- unit-testable.
  bool inConeZone(const ObstaclePoint & p) const;

  // Left-turn avoidance target (ported from senior avoidObstacle / calDx / calDy).
  // Rotates the obstacle vector by +90 deg — goes around on the obstacle's LEFT.
  ObstaclePoint computeAvoidanceTarget(const ObstaclePoint & p) const;

  // Right-turn avoidance target (ported from senior avoidObstacle_right).
  // Rotates the obstacle vector by -90 deg — goes around on the obstacle's RIGHT.
  // Uses 0.8×R scaling to make tighter right turns (matches senior code).
  ObstaclePoint computeAvoidanceTargetRight(const ObstaclePoint & p) const;

  // Issues the avoidance cmd_vel during the AVOIDING state.
  geometry_msgs::msg::TwistStamped computeAvoidanceCommand(
    const geometry_msgs::msg::PoseStamped & pose);

  // ----- members -----

  // Subscription + node handle cache.
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;

  // Tunable parameters (loaded in configure()).
  std::string obstacle_topic_;
  double cone_zone_a_;
  double cone_zone_b_;
  double cone_zone_c_;
  double avoid_R_;              // far-field avoidance radius
  double avoid_bigR_;           // near-field avoidance radius (r < 0.6m)
  double y_threshold_;          // front cutoff (meters)
  double offset_x_first_half_;  // turn threshold early in run (default -0.12)
  double offset_x_second_half_; // turn threshold later in run  (default -0.08)
  double half_transition_distance_;  // distance (meters) to switch first→second half (default 3.0)
  double recovery_duration_;    // seconds, after obstacle clears
  double avoid_linear_vel_;     // m/s while in AVOIDING
  double max_angular_vel_;      // rad/s clamp while AVOIDING
  double avoidance_steering_gain_;  // P-gain for avoidance bearing (default 2.0)
  double path_bias_weight_;        // RPP path bias during AVOIDING (default 0.3)
  double avoidance_cooldown_duration_;  // cooldown after avoidance (s, default 0.5)
  double recovery_speed_ratio_;     // speed multiplier in RECOVERY (default 0.5)
  double recovery_steering_boost_; // steering amplification during RECOVERY (default 1.5)
  double near_threshold_distance_;  // r < this uses bigR else R (default 0.6)
  double right_turn_radius_scale_;  // right-turn radius scaling (default 0.8)
  bool   enable_visual_avoidance_;

  // Cached state across ticks (avoids allocation in hot path).
  std::vector<ObstaclePoint> latest_obstacles_;
  std::mutex obstacle_mutex_;

  State state_;
  ObstaclePoint current_target_;
  rclcpp::Time recovery_start_time_;
  rclcpp::Time last_avoidance_end_time_;  // when last AVOIDING->RECOVERY transition happened

  // Tracks which direction we chose for the current avoidance episode.
  // true = left-turn (computeAvoidanceTarget), false = right-turn (computeAvoidanceTargetRight).
  bool turn_left_;

  // Odom-based half-track: first_half vs second_half of the run.
  // Matches senior's time-based <3000ms / ≥3000ms switching, but uses distance
  // for robustness against varying speeds.
  double odom_distance_travelled_ = 0.0;
  double last_pose_x_ = 0.0;
  double last_pose_y_ = 0.0;
  bool   pose_initialized_ = false;
};

}  // namespace visual_avoidance_controller
