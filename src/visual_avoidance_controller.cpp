// Copyright 2026, lzs. Apache-2.0
//
// VisualAvoidanceController implementation.
// Port of the senior team's Cone_Judgement + Cone_Judgement_qr +
// avoidObstacle / avoidObstacle_right logic from
//   /root/Car/.../VehicleCompetiyion/FindQRAndLinesAndPstopAndLines/
//   LineFolowControl/avoid_zjkpkg1/src/pidf.cpp
// into a Nav2 controller plugin that wraps
// nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController.

#include "visual_avoidance_controller/visual_avoidance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace visual_avoidance_controller
{

using nav2_util::declare_parameter_if_not_declared;


void VisualAvoidanceController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 1. Configure the parent (RegulatedPurePursuitController). This loads
  //    lookahead_dist, costmap regulation, etc. and creates the parent's
  //    publishers, dynamic-params handler, etc.
  RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error(
      "VisualAvoidanceController::configure failed to lock parent node");
  }

  node_ = parent;
  plugin_name_ = name;
  clock_ = node->get_clock();

  // 2. Declare and read OUR parameters (all prefixed by the plugin name so
  //    nav2 puts them under the controller's namespace in yaml).
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_topic",
    rclcpp::ParameterValue(std::string("/obstacle_info")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cone_zone_a", rclcpp::ParameterValue(75.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cone_zone_b", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cone_zone_c", rclcpp::ParameterValue(12.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".avoid_R", rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".avoid_bigR", rclcpp::ParameterValue(0.45));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".y_threshold", rclcpp::ParameterValue(0.75));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".offset_x_first_half", rclcpp::ParameterValue(-0.12));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".offset_x_second_half", rclcpp::ParameterValue(-0.08));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".half_transition_distance", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_duration", rclcpp::ParameterValue(0.58));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".avoid_linear_vel", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".avoidance_steering_gain", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_bias_weight", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".avoidance_cooldown_duration", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_speed_ratio", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_steering_boost", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".near_threshold_distance", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".right_turn_radius_scale", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_visual_avoidance", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name_ + ".obstacle_topic", obstacle_topic_);
  node->get_parameter(plugin_name_ + ".cone_zone_a", cone_zone_a_);
  node->get_parameter(plugin_name_ + ".cone_zone_b", cone_zone_b_);
  node->get_parameter(plugin_name_ + ".cone_zone_c", cone_zone_c_);
  node->get_parameter(plugin_name_ + ".avoid_R", avoid_R_);
  node->get_parameter(plugin_name_ + ".avoid_bigR", avoid_bigR_);
  node->get_parameter(plugin_name_ + ".y_threshold", y_threshold_);
  node->get_parameter(plugin_name_ + ".offset_x_first_half", offset_x_first_half_);
  node->get_parameter(plugin_name_ + ".offset_x_second_half", offset_x_second_half_);
  node->get_parameter(plugin_name_ + ".half_transition_distance", half_transition_distance_);
  node->get_parameter(plugin_name_ + ".recovery_duration", recovery_duration_);
  node->get_parameter(plugin_name_ + ".avoid_linear_vel", avoid_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".avoidance_steering_gain", avoidance_steering_gain_);
  node->get_parameter(plugin_name_ + ".path_bias_weight", path_bias_weight_);
  node->get_parameter(plugin_name_ + ".avoidance_cooldown_duration", avoidance_cooldown_duration_);
  node->get_parameter(plugin_name_ + ".recovery_speed_ratio", recovery_speed_ratio_);
  node->get_parameter(plugin_name_ + ".recovery_steering_boost", recovery_steering_boost_);
  node->get_parameter(plugin_name_ + ".near_threshold_distance", near_threshold_distance_);
  node->get_parameter(plugin_name_ + ".right_turn_radius_scale", right_turn_radius_scale_);
  node->get_parameter(plugin_name_ + ".enable_visual_avoidance", enable_visual_avoidance_);

  // 3. Subscribe to obstacle topics. PoseArray with each pose.position.{x,y}
  //    being the obstacle coordinates in the configured frame (typically base_link).
  obstacle_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    obstacle_topic_, 5,
    std::bind(&VisualAvoidanceController::obstacleCallback, this, std::placeholders::_1));

  state_ = State::NORMAL;
  turn_left_ = true;
  last_avoidance_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  RCLCPP_INFO(
    node->get_logger(),
    "VisualAvoidanceController '%s' ready: obstacle_topic='%s', "
    "cone=[a=%.1f, b=%.1f, c=%.1f], R=%.2f, bigR=%.2f, y_th=%.2f, "
    "offsets=[%.2f,%.2f] half_trans=%.2fm steer_gain=%.1f recov_ratio=%.2f enable=%s",
    plugin_name_.c_str(), obstacle_topic_.c_str(),
    cone_zone_a_, cone_zone_b_, cone_zone_c_,
    avoid_R_, avoid_bigR_, y_threshold_,
    offset_x_first_half_, offset_x_second_half_,
    half_transition_distance_, avoidance_steering_gain_,
    recovery_speed_ratio_,
    enable_visual_avoidance_ ? "true" : "false");
}

void VisualAvoidanceController::cleanup()
{
  obstacle_sub_.reset();
  RegulatedPurePursuitController::cleanup();
}

void VisualAvoidanceController::activate() {}
void VisualAvoidanceController::deactivate() {}

void VisualAvoidanceController::setPlan(const nav_msgs::msg::Path & path)
{
  // Make sure the parent's plan is fresh; this resets RPP's internal cached plan
  // and goal pose. Reset our state and odom tracking on every plan change
  // so the new run starts clean.
  RegulatedPurePursuitController::setPlan(path);
  state_ = State::NORMAL;
  recovery_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Reset first-half / second-half tracking for the new plan.
  odom_distance_travelled_ = 0.0;
  pose_initialized_ = false;
}

void VisualAvoidanceController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  RegulatedPurePursuitController::setSpeedLimit(speed_limit, percentage);
}

void VisualAvoidanceController::obstacleCallback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  // Cache the latest obstacle list. The control loop reads it under a short mutex.
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  latest_obstacles_.clear();
  latest_obstacles_.reserve(msg->poses.size());
  for (const auto & p : msg->poses) {
    // Defensive: only keep finite points.
    if (std::isfinite(p.position.x) && std::isfinite(p.position.y)) {
      latest_obstacles_.emplace_back(p.position.x, p.position.y);
    }
  }
}

bool VisualAvoidanceController::inConeZone(const ObstaclePoint & p) const
{
  // Cone-zone check, ported from the senior's Cone_Judgement:
  //   -a*x + b*y + c < 0   (left cone boundary)
  //   a*x + b*y + c  < 0   (right cone boundary)
  //   y > y_threshold      (front cutoff)
  const double left = -cone_zone_a_ * p.x + cone_zone_b_ * p.y + cone_zone_c_;
  const double right = cone_zone_a_ * p.x + cone_zone_b_ * p.y + cone_zone_c_;
  return (left >= 0.0) && (right >= 0.0) && (p.y <= y_threshold_);
}

ObstaclePoint VisualAvoidanceController::computeAvoidanceTarget(
  const ObstaclePoint & p) const
{
  // Port of senior avoidObstacle() / calDx() / calDy():
  //
  //   r = hypot(x, y)
  //   R_use = (r < 0.6) ? bigR : R
  //   dx = -y / r * R_use
  //   dy =  x / r * R_use
  //   target = (x + dx, y + dy)
  //
  // This is a LEFT-turn avoidance: the target sits R_use meters off the
  // obstacle, rotated +90 deg (counter-clockwise). The car will steer
  // LEFT to go around the obstacle on its left side.
  const double r = std::hypot(p.x, p.y);
  const double R_use = (r < near_threshold_distance_) ? avoid_bigR_ : avoid_R_;
  if (r < 1e-6) {
    // Degenerate: obstacle on top of us. Push straight forward.
    return ObstaclePoint(p.x, p.y + R_use);
  }
  const double dx = -p.y / r * R_use;
  const double dy =  p.x / r * R_use;
  return ObstaclePoint(p.x + dx, p.y + dy);
}

ObstaclePoint VisualAvoidanceController::computeAvoidanceTargetRight(
  const ObstaclePoint & p) const
{
  // Port of senior avoidObstacle_right() / calDx() / calDy():
  //
  //   r = hypot(x, y)
  //   R_use = (r < 0.6) ? bigR * 0.8 : R * 0.8   ← 0.8× scaling for tighter right turn
  //   target = (x - calDx, y - calDy)               ← subtract instead of add
  //
  // This is a RIGHT-turn avoidance: the target sits closer to the obstacle
  // (0.8× radius) and rotated -90 deg (clockwise). The car will steer
  // RIGHT to go around the obstacle on its right side.
  // Uses the same 0.8× scaling as the senior code's avoidObstacle_right.
  const double r = std::hypot(p.x, p.y);
  // Right turns use 0.8×R — tighter, matches senior avoidObstacle_right
  const double R_use = (r < near_threshold_distance_) ? avoid_bigR_ * right_turn_radius_scale_ : avoid_R_ * right_turn_radius_scale_;
  if (r < 1e-6) {
    return ObstaclePoint(p.x, p.y + R_use);
  }
  // calDx = -y/r * R_use,  calDy = x/r * R_use   (same as left-turn helper)
  const double cal_dx = -p.y / r * R_use;
  const double cal_dy =  p.x / r * R_use;
  // avoidObstacle_right: target = (x - calDx, y - calDy)
  return ObstaclePoint(p.x - cal_dx, p.y - cal_dy);
}

geometry_msgs::msg::TwistStamped VisualAvoidanceController::computeAvoidanceCommand(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.header.stamp = clock_->now();

  // Steer toward the avoidance target using a simple P-controller on the
  // bearing (atan2 of target in robot base_link frame).
  //   Forward  (+x)  -> bearing = 0    -> omega = 0
  //   Left     (+y)  -> bearing > 0    -> omega > 0  (turn left)
  //   Right    (-y)  -> bearing < 0    -> omega < 0  (turn right)
  const double bearing = std::atan2(current_target_.y, current_target_.x);
  const double K = avoidance_steering_gain_;        // proportional gain for avoidance steering
  double omega = K * bearing;
  omega = std::clamp(omega, -max_angular_vel_, max_angular_vel_);

  cmd_vel.twist.linear.x = avoid_linear_vel_;
  cmd_vel.twist.angular.z = omega;
  return cmd_vel;
}

geometry_msgs::msg::TwistStamped VisualAvoidanceController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // If visual avoidance is disabled, just delegate.
  if (!enable_visual_avoidance_) {
    return RegulatedPurePursuitController::computeVelocityCommands(
      pose, velocity, goal_checker);
  }

  // ── Update odom-based first-half / second-half tracking ─────────────
  // Port of the senior's time-based <3000ms / ≥3000ms switching, but
  // using cumulative distance travelled for robustness against varying speeds.
  if (pose_initialized_) {
    const double dx = pose.pose.position.x - last_pose_x_;
    const double dy = pose.pose.position.y - last_pose_y_;
    odom_distance_travelled_ += std::hypot(dx, dy);
  }
  last_pose_x_ = pose.pose.position.x;
  last_pose_y_ = pose.pose.position.y;
  pose_initialized_ = true;

  // Determine which offset threshold to use based on current phase.
  const double offset_threshold = (odom_distance_travelled_ < half_transition_distance_)
    ? offset_x_first_half_    // e.g. -0.12 in first half
    : offset_x_second_half_;  // e.g. -0.08 in second half

  // Snapshot the obstacle list under the mutex.
  std::vector<ObstaclePoint> obstacles;
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles = latest_obstacles_;
  }

  auto node = node_.lock();
  if (!node) {
    return RegulatedPurePursuitController::computeVelocityCommands(
      pose, velocity, goal_checker);
  }

  const rclcpp::Time now = clock_->now();

  // Find the nearest (smallest front distance) obstacle in our local frame.
  ObstaclePoint nearest(0.0, std::numeric_limits<double>::max());
  bool have_obstacle = false;
  if (!obstacles.empty()) {
    std::sort(obstacles.begin(), obstacles.end(),
      [](const ObstaclePoint & a, const ObstaclePoint & b) {
        return a.y < b.y;
      });
    nearest = obstacles[0];
    have_obstacle = true;
  }

  // ────────────────────────── State machine ──────────────────────────
  switch (state_) {
    case State::NORMAL:
      if (have_obstacle && inConeZone(nearest)
          && (now - last_avoidance_end_time_).seconds() > avoidance_cooldown_duration_) {
        state_ = State::AVOIDING;
        
        // ── Adaptive direction selection (ported from senior Cone_Judgement) ──
        // First half:  obstacle.x > offset_x_first_half  → left,  else → right
        // Second half: obstacle.x > offset_x_second_half → left,  else → right
        if (nearest.x > offset_threshold) {
          current_target_ = computeAvoidanceTarget(nearest);
          turn_left_ = true;
          RCLCPP_INFO_THROTTLE(
            node->get_logger(), *clock_, 500,
            "[VisualAvoidance] NORMAL -> AVOIDING (LEFT), nearest=(%.2f,%.2f) "
            "target=(%.2f,%.2f) odom=%.2fm offset=%.2f",
            nearest.x, nearest.y, current_target_.x, current_target_.y,
            odom_distance_travelled_, offset_threshold);
        } else {
          current_target_ = computeAvoidanceTargetRight(nearest);
          turn_left_ = false;
          RCLCPP_INFO_THROTTLE(
            node->get_logger(), *clock_, 500,
            "[VisualAvoidance] NORMAL -> AVOIDING (RIGHT), nearest=(%.2f,%.2f) "
            "target=(%.2f,%.2f) odom=%.2fm offset=%.2f",
            nearest.x, nearest.y, current_target_.x, current_target_.y,
            odom_distance_travelled_, offset_threshold);
        }
      }
      break;

    case State::AVOIDING:
      if (!have_obstacle || !inConeZone(nearest)) {
        state_ = State::RECOVERY;
        recovery_start_time_ = now;
        last_avoidance_end_time_ = now;
        RCLCPP_INFO_THROTTLE(
          node->get_logger(), *clock_, 500,
          "[VisualAvoidance] AVOIDING -> RECOVERY (cone cleared)");
      } else {
        if (turn_left_) {
          current_target_ = computeAvoidanceTarget(nearest);
        } else {
          current_target_ = computeAvoidanceTargetRight(nearest);
        }
      }
      break;

    case State::RECOVERY:
      if (have_obstacle && inConeZone(nearest)) {
        state_ = State::AVOIDING;
        if (nearest.x > offset_threshold) {
          current_target_ = computeAvoidanceTarget(nearest);
          turn_left_ = true;
        } else {
          current_target_ = computeAvoidanceTargetRight(nearest);
          turn_left_ = false;
        }
        RCLCPP_INFO_THROTTLE(
          node->get_logger(), *clock_, 500,
          "[VisualAvoidance] RECOVERY -> AVOIDING (cone re-entered, %s)",
          turn_left_ ? "LEFT" : "RIGHT");
      } else if (!have_obstacle &&
                 (now - recovery_start_time_).seconds() > recovery_duration_)
      {
        state_ = State::NORMAL;
        RCLCPP_INFO_THROTTLE(
          node->get_logger(), *clock_, 500,
          "[VisualAvoidance] RECOVERY -> NORMAL (clean for %.2fs)",
          recovery_duration_);
      }
      break;
  }

  // ────────────────────────── Emit by state ──────────────────────────
  switch (state_) {
    case State::NORMAL:
      return RegulatedPurePursuitController::computeVelocityCommands(
        pose, velocity, goal_checker);

    case State::AVOIDING: {
      // Blend avoidance with RPP path-following to prevent path drift
      auto avoid_cmd = computeAvoidanceCommand(pose);
      auto rpp_cmd = RegulatedPurePursuitController::computeVelocityCommands(
        pose, velocity, goal_checker);
      avoid_cmd.twist.linear.x = path_bias_weight_ * rpp_cmd.twist.linear.x
                               + (1.0 - path_bias_weight_) * avoid_cmd.twist.linear.x;
      avoid_cmd.twist.angular.z = path_bias_weight_ * rpp_cmd.twist.angular.z
                                + (1.0 - path_bias_weight_) * avoid_cmd.twist.angular.z;
      return avoid_cmd;
    }
    case State::RECOVERY: {
      // Run RPP normally but at half speed while we wait for the recovery
      // timeout.
      auto cmd = RegulatedPurePursuitController::computeVelocityCommands(
        pose, velocity, goal_checker);
      cmd.twist.linear.x *= recovery_speed_ratio_;
      cmd.twist.angular.z *= recovery_steering_boost_;
      return cmd;
    }
  }

  // Unreachable: keep the compiler happy.
  return RegulatedPurePursuitController::computeVelocityCommands(
    pose, velocity, goal_checker);
}

}  // namespace visual_avoidance_controller

PLUGINLIB_EXPORT_CLASS(
  visual_avoidance_controller::VisualAvoidanceController,
  nav2_core::Controller)
