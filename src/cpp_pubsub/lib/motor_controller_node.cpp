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

#include "motor_controller_node.hpp"

#include <atomic>

using namespace std::chrono_literals;
using std::placeholders::_1;

MotorControllerNode::MotorControllerNode(float rate, float position_kp, uint8_t speed_kp,
                                         float max_speed)
    : Node("motor_controller_node"),
      publish_rate_(rate),
      motor_controller_(position_kp, speed_kp, max_speed, kMotorConnections) {
  // CAN interface setup
  motor_controller_.begin();
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

MotorControllerNode::~MotorControllerNode() {
  std::cout << "Destroying motor controller node..." << std::endl;
  stop_ = true;
  calibration_thread_.join();
  std::cout << "Finished destroying motor controller." << std::endl;
}

void MotorControllerNode::startup() {
  RCLCPP_INFO(this->get_logger(), "Beginning startup thread");
  calibration_thread_ = std::thread([this]() { this->startup_thread_fn(); });
}

void MotorControllerNode::startup_thread_fn() {
  motor_controller_.calibrate_motors(stop_);
  MotorController<K_SERVOS_PER_CHANNEL>::ActuatorMatrix<float> command = {{0, 0, 1.0, 0, 0, -1.0},
                                                                          {0, 0, 1.0, 0, 0, -1.0}};
  motor_controller_.blocking_move(stop_, 750.0, 20000.0, 10, command);
}
/*
 * TODO: add velocity, feedforward, kp, kd
 */
void MotorControllerNode::joint_command_callback(
    pupper_interfaces::msg::JointCommand joint_command) {
  RCLCPP_INFO(this->get_logger(), "GOT JOINT COMMAND");
  latest_joint_command_ = joint_command;
}

std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> MotorControllerNode::split_vector(
    std::vector<double> vector) {
  MotorController<K_SERVOS_PER_CHANNEL>::ActuatorMatrix<float> data;
  for (size_t i = 0; i < vector.size() / K_SERVOS_PER_CHANNEL; i++) {
    std::array<float, K_SERVOS_PER_CHANNEL> section;
    for (size_t j = 0; j < K_SERVOS_PER_CHANNEL; j++) {
      section.at(j) = vector.at(j + K_SERVOS_PER_CHANNEL * i);
    }
    data.push_back(section);
  }
  return data;
}

void MotorControllerNode::publish_callback() {
  // Must receive a joint command message to make latest_joint_command_ non-zero
  // TODO: add watchdog
  if (latest_joint_command_.position_target.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No joint command received. Skipping control");
    return;
  }

  if (motor_controller_.is_calibrated()) {
    auto motor_position_targets = split_vector(latest_joint_command_.position_target);
    RCLCPP_INFO(this->get_logger(), "Commanding %lu motors on %lu buses.",
                latest_joint_command_.position_target.size(), motor_position_targets.size());
    motor_controller_.position_control(motor_position_targets);

    RCLCPP_INFO(this->get_logger(), "Publishing joint states");
    joint_state_message_.header.stamp = now();
    auto positions = motor_controller_.actuator_positions();
    auto velocities = motor_controller_.actuator_velocities();
    auto efforts = motor_controller_.actuator_efforts();

    for (size_t i = 0; i < positions.size() * K_SERVOS_PER_CHANNEL; i++) {
      int bus_id = i / K_SERVOS_PER_CHANNEL;
      int motor_id = i % K_SERVOS_PER_CHANNEL;
      joint_state_message_.position.at(i) = positions.at(bus_id).at(motor_id);
      joint_state_message_.velocity.at(i) = velocities.at(bus_id).at(motor_id);
      // TODO: make Nm
      joint_state_message_.effort.at(i) = efforts.at(bus_id).at(motor_id);
    }
    publisher_->publish(joint_state_message_);
  }
}