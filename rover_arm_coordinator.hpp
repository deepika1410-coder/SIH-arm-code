// Copyright 2026 Robotics Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef ROVER_ARM_BRINGUP__ROVER_ARM_COORDINATOR_HPP_
#define ROVER_ARM_BRINGUP__ROVER_ARM_COORDINATOR_HPP_

#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rover_arm_bringup
{

/// @brief Finite State Machine states governing rover/arm coordination.
enum class CoordinatorState
{
  NAVIGATING,      ///< Rover drives forward, monitoring the ultrasonic sensor.
  OBSTACLE_HALT,    ///< Obstacle detected; rover stopped, arm service being invoked.
  PICKING,          ///< Waiting asynchronously for the arm pick-and-place task.
  RESUME,           ///< Arm task complete; post-completion settle delay before resuming.
  MANUAL_OVERRIDE   ///< Autonomous publishing paused; an external client (e.g. a
                     ///< teleop dashboard) owns /cmd_vel directly.
};

/// @brief Converts a CoordinatorState to a human-readable string for logging.
std::string to_string(CoordinatorState state);

/// @class RoverArmCoordinator
/// @brief Coordinates a differential-drive rover with a 3-DOF robotic arm.
///
/// The node runs a non-blocking Finite State Machine driven by a wall timer.
/// While NAVIGATING it publishes a constant forward velocity on /cmd_vel and
/// listens to /ultrasonic/range. When an obstacle closer than
/// `obstacle_threshold` is detected, the rover is halted and an asynchronous
/// service call is issued to /arm/execute_pick. Once the arm reports success,
/// the node waits `resume_delay_sec` seconds before resuming navigation.
class RoverArmCoordinator : public rclcpp::Node
{
public:
  /// @brief Construct the coordinator node and declare/read all parameters.
  explicit RoverArmCoordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------

  /// @brief Subscription callback for the ultrasonic range sensor.
  /// @param msg Incoming Range message.
  void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);

  /// @brief Main control loop, invoked periodically by control_timer_.
  /// Publishes /cmd_vel according to the current FSM state and drives
  /// state transitions that do not depend on external async results.
  void control_loop();

  /// @brief Callback invoked when the /arm/execute_pick service response
  /// arrives. Transitions PICKING -> RESUME on success, or logs a warning
  /// and retries the halt/pick cycle on failure.
  void arm_response_callback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  /// @brief One-shot timer callback that ends the RESUME settle delay and
  /// transitions the FSM back to NAVIGATING.
  void resume_timer_callback();

  /// @brief Service callback toggling manual override on/off. When enabled,
  /// the FSM transitions to MANUAL_OVERRIDE and control_loop() stops
  /// publishing /cmd_vel entirely, so an external teleop client (e.g. the
  /// mobile dashboard) has sole ownership of the topic. When disabled, the
  /// FSM resets safely back to NAVIGATING.
  void set_manual_override_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // ---------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------

  /// @brief Thread-safe state transition with logging.
  void set_state(CoordinatorState new_state);

  /// @brief Thread-safe state read.
  CoordinatorState get_state();

  /// @brief Publishes a Twist message with the given linear.x, zero elsewhere.
  void publish_velocity(double linear_x);

  /// @brief Issues the asynchronous call to /arm/execute_pick, guarding
  /// against duplicate in-flight requests.
  void call_arm_pick_service();

  // ---------------------------------------------------------------------
  // ROS interfaces
  // ---------------------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_pick_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr manual_override_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr resume_timer_;

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  // ---------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------
  double target_speed_{0.2};             ///< [m/s] Forward navigation speed.
  double obstacle_threshold_{0.15};      ///< [m] Range at/below which we halt.
  double resume_delay_sec_{1.0};         ///< [s] Settle delay before resuming.
  double control_loop_period_sec_{0.05}; ///< [s] Control loop period (20 Hz).
  std::string arm_service_name_{"/arm/execute_pick"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string range_topic_{"/ultrasonic/range"};
  std::string state_topic_{"/rover_arm/state"};
  std::string manual_override_service_name_{"/rover_arm/set_manual_override"};

  // ---------------------------------------------------------------------
  // FSM / runtime state
  // ---------------------------------------------------------------------
  std::mutex state_mutex_;
  CoordinatorState state_{CoordinatorState::NAVIGATING};
  bool arm_request_in_flight_{false};
  double last_range_m_{std::numeric_limits<double>::infinity()};
};

}  // namespace rover_arm_bringup

#endif  // ROVER_ARM_BRINGUP__ROVER_ARM_COORDINATOR_HPP_
