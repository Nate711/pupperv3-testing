#include "motor_controller_generic_node.hpp"

#include <cassert>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include <atomic>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pupperv3 {

template <int K_SERVOS>
MotorControllerNode<K_SERVOS>::MotorControllerNode(
    float rate, std::unique_ptr<MotorController<K_SERVOS>> motor_controller,
    const std::array<std::string, K_SERVOS>& joint_names, const ActuatorVector& default_position)
    : Node("motor_controller_node"),
      publish_rate_(rate),
      motor_controller_(std::move(motor_controller)),
      default_position_(default_position),
      stop_(false) {
  // CAN interface setup
  motor_controller_->begin();
  SPDLOG_INFO("Began motor controller.");

  // Joint State joint_state_message_ setup
  for (std::string joint_name : joint_names) {
    joint_state_message_.name.push_back(joint_name);
    joint_state_message_.position.push_back(0.0);
    joint_state_message_.velocity.push_back(0.0);
    joint_state_message_.effort.push_back(0.0);
  }

  // Node setup
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
  timer_ = this->create_wall_timer(rclcpp::WallRate(publish_rate_).period(),
                                   std::bind(&MotorControllerNode::publish_callback, this));

  subscriber_ = this->create_subscription<pupper_interfaces::msg::JointCommand>(
      "/joint_commands", rclcpp::SensorDataQoS(),
      std::bind(&MotorControllerNode::joint_command_callback, this, _1));
}

template <int K_SERVOS>
MotorControllerNode<K_SERVOS>::~MotorControllerNode() {
  SPDLOG_INFO("Destroying motor controller node...");
  stop_ = true;
  startup_thread_.join();
  SPDLOG_INFO("Finished motor controller node destructor. Destroying members...");
}

template <int K_SERVOS>
void MotorControllerNode<K_SERVOS>::startup() {
  RCLCPP_INFO(this->get_logger(), "Beginning startup thread..");
  startup_thread_ = std::thread([this]() { this->startup_thread_fn(); });
}

template <int K_SERVOS>
void MotorControllerNode<K_SERVOS>::startup_thread_fn() {
  // Start watchdog with period 0.5x max timeout
  watchdog_timer_ =
      this->create_wall_timer(rclcpp::WallRate(1e6 / kWatchDogWarningUS * 2.0).period(),
                              std::bind(&MotorControllerNode::watchdog_callback, this));
  motor_controller_->calibrate_motors(stop_);
  motor_controller_->blocking_move(stop_, 750.0, 20000.0, 15, default_position_);
}
/*
 * TODO: add velocity, feedforward, kp, kd
 */
template <int K_SERVOS>
void MotorControllerNode<K_SERVOS>::joint_command_callback(
    pupper_interfaces::msg::JointCommand joint_command) {
  RCLCPP_INFO(this->get_logger(), "GOT JOINT COMMAND");
  latest_joint_command_ = joint_command;
}

template <int K_SERVOS>
void MotorControllerNode<K_SERVOS>::publish_callback() {
  // Must receive a joint command message to make latest_joint_command_ non-zero
  if (latest_joint_command_.position_target.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No joint command received. Skipping control");
  } else if (latest_joint_command_.position_target.size() != K_SERVOS) {
    RCLCPP_WARN(this->get_logger(), "Joint command vector size != K_SERVOS");
  } else if (motor_controller_->is_calibrated()) {
    ActuatorVector motor_position_targets;
    for (size_t i = 0; i < latest_joint_command_.position_target.size(); i++) {
      motor_position_targets(i) = latest_joint_command_.position_target[i];
    }
    motor_controller_->position_control(motor_position_targets);
  } else {
    RCLCPP_WARN(this->get_logger(), "Motor controller not calibrated");
  }

  RCLCPP_INFO(this->get_logger(), "Publishing joint states");
  joint_state_message_.header.stamp = now();
  auto positions = motor_controller_->actuator_positions();
  auto velocities = motor_controller_->actuator_velocities();
  auto efforts = motor_controller_->actuator_efforts();

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  SPDLOG_INFO("Positions: {}", positions.transpose().format(CleanFmt));

  for (int i = 0; i < positions.size(); i++) {
    joint_state_message_.position.at(i) = positions(i);
    joint_state_message_.velocity.at(i) = velocities(i);
    // TODO: make Nm
    joint_state_message_.effort.at(i) = efforts(i);
  }
  publisher_->publish(joint_state_message_);
}

template <int K_SERVOS>
void MotorControllerNode<K_SERVOS>::watchdog_callback() {
  auto latency = motor_controller_->micros_since_last_read();
  int max_latency = latency.maxCoeff();
  int max_idx;
  latency.maxCoeff(&max_idx);

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  SPDLOG_INFO("Microseconds since last read: {}", latency.transpose().format(CleanFmt));
  if (max_latency > kWatchDogWarningUS) {
    SPDLOG_WARN("Watchdog WARNING: Latency on motor {}: {} us. Max allowed: {} us", max_idx,
                max_latency, kWatchDogTimeoutUS);
  }
  if (max_latency > kWatchDogTimeoutUS) {
    SPDLOG_ERROR("Watchdog TRIGGER: Latency on motor {}: {} us. Max allowed: {} us", max_idx,
                 max_latency, kWatchDogTimeoutUS);
    throw WatchdogTriggered("Watchdog triggered");
  }
}

template class MotorControllerNode<3>;
template class MotorControllerNode<6>;
template class MotorControllerNode<12>;
}  // namespace pupperv3
