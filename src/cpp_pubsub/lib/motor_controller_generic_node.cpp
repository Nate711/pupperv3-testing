// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "motor_controller_generic_node.hpp"

#include <atomic>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pupperv3 {

template <int K_SERVOS>
MotorControllerNode<K_SERVOS>::MotorControllerNode(
    float rate, std::unique_ptr<MotorController<K_SERVOS>> motor_controller)
    : Node("motor_controller_node"),
      publish_rate_(rate),
      motor_controller_(std::move(motor_controller)) {
  // CAN interface setup
  motor_controller_->begin();
  std::cout << "Initialized motor controller." << std::endl;

  // Joint State joint_state_message_ setup
  for (std::string joint_name : joint_names_) {
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
  std::cout << "Destroying motor controller node..." << std::endl;
  stop_ = true;
  calibration_thread_.join();
  std::cout << "Finished destroying motor controller." << std::endl;
}

template <int K_SERVOS>
void MotorControllerNode<K_SERVOS>::startup() {
  RCLCPP_INFO(this->get_logger(), "Beginning startup thread");
  calibration_thread_ = std::thread([this]() { this->startup_thread_fn(); });
}

template <int K_SERVOS>
void MotorControllerNode<K_SERVOS>::startup_thread_fn() {
  motor_controller_->calibrate_motors(stop_);
  // ActuatorVector command = {0, 0, 1.0}; // hard coding 3 is bad
  // motor_controller_->blocking_move(stop_, 750.0, 20000.0, 15, command);
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
  // TODO: add watchdog
  // if (latest_joint_command_.position_target.size() == 0) {
  //   RCLCPP_WARN(this->get_logger(), "No joint command received. Skipping control");
  // } else if (motor_controller_->is_calibrated()) {
  //   auto motor_position_targets = split_vector(latest_joint_command_.position_target);
  //   RCLCPP_INFO(this->get_logger(), "Commanding %lu motors on %lu buses.",
  //               latest_joint_command_.position_target.size(), motor_position_targets.size());
  //   motor_controller_->position_control(motor_position_targets);
  // }

  // RCLCPP_INFO(this->get_logger(), "Publishing joint states");
  // joint_state_message_.header.stamp = now();
  // auto positions = motor_controller_->actuator_positions();
  // auto velocities = motor_controller_->actuator_velocities();
  // auto efforts = motor_controller_->actuator_efforts();

  // for (size_t i = 0; i < positions.size() * K_SERVOS; i++) {
  //   joint_state_message_.position.at(i) = positions(i);
  //   joint_state_message_.velocity.at(i) = velocities(i);
  //   // TODO: make Nm
  //   joint_state_message_.effort.at(i) = efforts(i);
  // }
  // publisher_->publish(joint_state_message_);
}
template class MotorControllerNode<3>;
template class MotorControllerNode<6>;
template class MotorControllerNode<12>;
}  // namespace pupperv3
