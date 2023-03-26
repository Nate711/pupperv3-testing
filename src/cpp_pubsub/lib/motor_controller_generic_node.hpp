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

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "motor_controller_generic.hpp"
#include "motor_interface_generic.hpp"

namespace pupperv3 {

template <int K_SERVOS>
class MotorControllerNode : public rclcpp::Node {
 public:
  using ActuatorVector = typename MotorController<K_SERVOS>::ActuatorVector;
  MotorControllerNode(float rate, std::unique_ptr<MotorController<K_SERVOS>> motor_controller,
                      const std::array<std::string, K_SERVOS>& joint_names,
                      const ActuatorVector& default_position);
  ~MotorControllerNode();
  void startup();

 private:
  void startup_thread_fn();
  void publish_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr subscriber_;
  sensor_msgs::msg::JointState joint_state_message_;
  void joint_command_callback(pupper_interfaces::msg::JointCommand joint_command);

  // TODO: make parameter
  //   const std::string joint_names_[12] = {
  //       "leg_front_r_1", "leg_front_r_2", "leg_front_r_3", "leg_front_l_1",
  //       "leg_front_l_2", "leg_front_l_3", "leg_back_r_1",  "leg_back_r_2",
  //       "leg_back_r_3",  "leg_back_l_1",  "leg_back_l_2",  "leg_back_l_3",
  //   };
  // const std::string joint_names_[K_SERVOS] = {"leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
  //                                             "leg_front_l_1", "leg_front_l_2", "leg_front_l_3"};
  // const std::string joint_names_[K_SERVOS] = {"leg_front_r_1", "leg_front_r_2", "leg_front_r_3"};
  float publish_rate_;
  std::unique_ptr<MotorController<K_SERVOS>> motor_controller_;
  ActuatorVector default_position_;
  pupper_interfaces::msg::JointCommand latest_joint_command_;
  std::thread calibration_thread_;
  std::atomic<bool> stop_;
};
}  // namespace pupperv3