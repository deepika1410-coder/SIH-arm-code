// Copyright 2026 Robotics Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rover_arm_bringup/rover_arm_coordinator.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace rover_arm_bringup
{

std::string to_string(CoordinatorState state)
{
  switch (state) {
    case CoordinatorState::NAVIGATING:
      return "NAVIGATING";
    case CoordinatorState::OBSTACLE_HALT:
      return "OBSTACLE_HALT";
    case CoordinatorState::PICKING:
      return "PICKING";
    case CoordinatorState::RESUME:
      return "RESUME";
    case CoordinatorState::MANUAL_OVERRIDE:
      return "MANUAL_OVERRIDE";
    default:
      return "UNKNOWN";
  }
}

RoverArmCoordinator::RoverArmCoordinator(const rclcpp::NodeOptions & options)
: Node("rover_arm_coordinator_node", options)
{
  // ---------------------------------------------------------------------
  // Parameter declaration with sane defaults and basic descriptions.
  // ---------------------------------------------------------------------
  rcl_interfaces::msg::ParameterDescriptor speed_desc;
  speed_desc.description = "Forward navigation speed published to /cmd_vel [m/s]";
  target_speed_ = this->declare_parameter<double>("target_speed", 0.2, speed_desc);

  rcl_interfaces::msg::ParameterDescriptor threshold_desc;
  threshold_desc.description =
    "Obstacle distance threshold [m]; halt when range <= this value";
  obstacle_threshold_ =
    this->declare_parameter<double>("obstacle_threshold", 0.15, threshold_desc);

  rcl_interfaces::msg::ParameterDescriptor resume_delay_desc;
  resume_delay_desc.description =
    "Settle delay [s] after a successful pick before resuming navigation";
  resume_delay_sec_ =
    this->declare_parameter<double>("resume_delay_sec", 1.0, resume_delay_desc);

  rcl_interfaces::msg::ParameterDescriptor loop_period_desc;
  loop_period_desc.description = "Control loop period [s] (default 0.05 -> 20 Hz)";
  control_loop_period_sec_ =
    this->declare_parameter<double>("control_loop_period_sec", 0.05, loop_period_desc);

  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  range_topic_ = this->declare_parameter<std::string>("range_topic", "/ultrasonic/range");
  arm_service_name_ =
    this->declare_parameter<std::string>("arm_service_name", "/arm/execute_pick");
  state_topic_ = this->declare_parameter<std::string>("state_topic", "/rover_arm/state");
  manual_override_service_name_ = this->declare_parameter<std::string>(
    "manual_override_service_name", "/rover_arm/set_manual_override");

  if (target_speed_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "target_speed (%.3f) should be positive; forcing to 0.2 m/s", target_speed_);
    target_speed_ = 0.2;
  }
  if (obstacle_threshold_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "obstacle_threshold (%.3f) should be positive; forcing to 0.15 m", obstacle_threshold_);
    obstacle_threshold_ = 0.15;
  }

  // ---------------------------------------------------------------------
  // Publishers / Subscribers / Service client
  // ---------------------------------------------------------------------
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // Latched-like QoS (transient local, depth 1) so a dashboard connecting
  // mid-run immediately gets the current FSM state instead of waiting for
  // the next transition.
  rclcpp::QoS state_qos(1);
  state_qos.transient_local();
  state_pub_ = this->create_publisher<std_msgs::msg::String>(state_topic_, state_qos);

  auto sensor_qos = rclcpp::SensorDataQoS();
  range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    range_topic_, sensor_qos, std::bind(&RoverArmCoordinator::range_callback, this, _1));

  // Dedicated re-entrant callback group so the service response callback
  // is never blocked behind the periodic control-loop timer callback.
  client_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  arm_pick_client_ = this->create_client<std_srvs::srv::Trigger>(
    arm_service_name_, rmw_qos_profile_services_default, client_callback_group_);

  manual_override_srv_ = this->create_service<std_srvs::srv::SetBool>(
    manual_override_service_name_,
    std::bind(
      &RoverArmCoordinator::set_manual_override_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // ---------------------------------------------------------------------
  // Timers
  // ---------------------------------------------------------------------
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(control_loop_period_sec_),
    std::bind(&RoverArmCoordinator::control_loop, this));

  RCLCPP_INFO(
    get_logger(),
    "rover_arm_coordinator_node initialized | target_speed=%.2f m/s | "
    "obstacle_threshold=%.2f m | resume_delay=%.2f s | cmd_vel='%s' | "
    "range='%s' | arm_service='%s'",
    target_speed_, obstacle_threshold_, resume_delay_sec_,
    cmd_vel_topic_.c_str(), range_topic_.c_str(), arm_service_name_.c_str());
  RCLCPP_INFO(get_logger(), "Entering initial state: %s", to_string(state_).c_str());

  // Publish the initial state immediately (transient-local QoS keeps it
  // available for any dashboard/rosbridge client that connects afterward).
  std_msgs::msg::String initial_state_msg;
  initial_state_msg.data = to_string(state_);
  state_pub_->publish(initial_state_msg);
}

void RoverArmCoordinator::range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  last_range_m_ = static_cast<double>(msg->range);

  // Only NAVIGATING cares about new obstacle detections; once we are
  // halting/picking/resuming, further range updates are informational only
  // until we are back to NAVIGATING.
  if (get_state() == CoordinatorState::NAVIGATING &&
    last_range_m_ <= obstacle_threshold_)
  {
    RCLCPP_WARN(
      get_logger(),
      "Obstacle detected at %.3f m (threshold %.3f m). Halting rover.",
      last_range_m_, obstacle_threshold_);
    set_state(CoordinatorState::OBSTACLE_HALT);
  }
}

void RoverArmCoordinator::control_loop()
{
  switch (get_state()) {
    case CoordinatorState::NAVIGATING:
      publish_velocity(target_speed_);
      break;

    case CoordinatorState::OBSTACLE_HALT:
      publish_velocity(0.0);
      call_arm_pick_service();
      break;

    case CoordinatorState::PICKING:
      // Rover remains stationary while the arm executes; nothing to publish
      // beyond holding zero velocity defensively.
      publish_velocity(0.0);
      break;

    case CoordinatorState::RESUME:
      // Stay stopped during the settle delay; resume_timer_callback()
      // performs the transition back to NAVIGATING.
      publish_velocity(0.0);
      break;

    case CoordinatorState::MANUAL_OVERRIDE:
      // Intentionally publish nothing here: an external teleop client
      // (e.g. the mobile dashboard) owns /cmd_vel while this is active.
      break;
  }
}

void RoverArmCoordinator::call_arm_pick_service()
{
  if (arm_request_in_flight_) {
    // Guard against re-issuing the request on every control_loop tick
    // while OBSTACLE_HALT -> PICKING is still resolving.
    return;
  }

  if (!arm_pick_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 2000,
      "Waiting for arm service '%s' to become available...",
      arm_service_name_.c_str());
    return;
  }

  arm_request_in_flight_ = true;
  set_state(CoordinatorState::PICKING);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  RCLCPP_INFO(
    get_logger(), "Calling arm service '%s' to execute pick-and-place...",
    arm_service_name_.c_str());

  arm_pick_client_->async_send_request(
    request,
    std::bind(&RoverArmCoordinator::arm_response_callback, this, _1));
}

void RoverArmCoordinator::arm_response_callback(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  arm_request_in_flight_ = false;

  std_srvs::srv::Trigger::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Arm service call threw an exception: %s. Retrying halt cycle.",
      e.what());
    set_state(CoordinatorState::OBSTACLE_HALT);
    return;
  }

  if (response->success) {
    RCLCPP_INFO(
      get_logger(), "Arm pick-and-place succeeded: '%s'. Resuming in %.2f s.",
      response->message.c_str(), resume_delay_sec_);
    set_state(CoordinatorState::RESUME);

    resume_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(resume_delay_sec_),
      std::bind(&RoverArmCoordinator::resume_timer_callback, this));
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Arm pick-and-place reported failure: '%s'. Retrying halt cycle.",
      response->message.c_str());
    set_state(CoordinatorState::OBSTACLE_HALT);
  }
}

void RoverArmCoordinator::resume_timer_callback()
{
  // One-shot behavior: cancel immediately so this timer does not fire again.
  if (resume_timer_) {
    resume_timer_->cancel();
    resume_timer_.reset();
  }

  RCLCPP_INFO(get_logger(), "Resume delay elapsed. Returning to NAVIGATING.");
  set_state(CoordinatorState::NAVIGATING);
}

void RoverArmCoordinator::set_manual_override_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    RCLCPP_WARN(
      get_logger(),
      "Manual override ENABLED. Autonomous /cmd_vel publishing paused; "
      "external client now owns velocity commands.");
    set_state(CoordinatorState::MANUAL_OVERRIDE);
    response->success = true;
    response->message = "Manual override enabled";
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Manual override DISABLED. Publishing zero velocity and resuming "
      "autonomous navigation.");
    // Defensive stop before handing control back to the FSM, so any
    // in-flight manual command doesn't linger as the last published value.
    publish_velocity(0.0);
    set_state(CoordinatorState::NAVIGATING);
    response->success = true;
    response->message = "Manual override disabled; resumed NAVIGATING";
  }
}

void RoverArmCoordinator::set_state(CoordinatorState new_state)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != new_state) {
      RCLCPP_INFO(
        get_logger(), "FSM transition: %s -> %s",
        to_string(state_).c_str(), to_string(new_state).c_str());
      state_ = new_state;
      changed = true;
    }
  }

  // Publish outside the lock to avoid holding state_mutex_ during I/O.
  if (changed && state_pub_) {
    std_msgs::msg::String msg;
    msg.data = to_string(new_state);
    state_pub_->publish(msg);
  }
}

CoordinatorState RoverArmCoordinator::get_state()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

void RoverArmCoordinator::publish_velocity(double linear_x)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  cmd_vel_pub_->publish(msg);
}

}  // namespace rover_arm_bringup
