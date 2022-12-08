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

using namespace std::chrono_literals;
using std::placeholders::_1;

#define M_DEG_TO_RAD 0.01745329252
#define M_GEAR_RATIO 0.10

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

MotorControllerNode::~MotorControllerNode() {}

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
  std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> data;
  assert(vector.size() % K_SERVOS_PER_CHANNEL == 0);
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
  auto motor_position_targets = split_vector(latest_joint_command_.position_target);
  RCLCPP_INFO(this->get_logger(), "Commanding %lu motors on %lu buses.",
              latest_joint_command_.position_target.size(), motor_position_targets.size());
  // RCLCPP_INFO(this->get_logger(), motor_position_targets);
  motor_controller_.position_control(motor_position_targets);

  RCLCPP_INFO(this->get_logger(), "Publishing joint states");
  auto latest_data = motor_controller_.motor_data_safe();
  joint_state_message_.header.stamp = now();

  for (size_t i = 0; i < joint_state_message_.velocity.size(); i++) {
    int bus_id = i / K_SERVOS_PER_CHANNEL;
    int motor_id = i % K_SERVOS_PER_CHANNEL;
    joint_state_message_.position.at(i) = latest_data.at(bus_id).at(motor_id).common.output_rads;
    joint_state_message_.velocity.at(i) =
        latest_data.at(bus_id).at(motor_id).common.velocity_rads * M_GEAR_RATIO;

    // TODO: make Nm
    joint_state_message_.effort.at(i) = latest_data.at(bus_id).at(motor_id).common.current;
  }
  // RCLCPP_INFO(this->get_logger(), "Publishing joint state");
  publisher_->publish(joint_state_message_);
}