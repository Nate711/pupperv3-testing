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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "motor_controller_generic_node.hpp"

constexpr int K_SERVOS = 3;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  float publish_rate = 500;
  if (argc > 1) {
    publish_rate = std::stof(argv[1]);
  }
  std::cout << "Joint state publish_rate: " << publish_rate << std::endl;

  // Create config
  pupperv3::MotorID id1{pupperv3::CANChannel::CAN0, 1};
  pupperv3::MotorID id2{pupperv3::CANChannel::CAN0, 2};
  pupperv3::MotorID id3{pupperv3::CANChannel::CAN0, 3};
  pupperv3::ActuatorConfiguration actuator_config{{id1, id2, id3}};

  auto interface = std::make_unique<pupperv3::MotorInterface>(actuator_config);

  // 20000 was on the brink of being too stiff
  // 10000 was on the soft side and robot would fall backwards often
  float position_kp = 15000.0;  // 15000 is good default. units rotor deg/s per output rad
  uint8_t speed_kp = 5;         // 5 is good default. units A/rotor deg/s
  float max_speed = 5000;       // rotor deg/s
  using ActuatorVector = pupperv3::MotorController<K_SERVOS>::ActuatorVector;
  ActuatorVector endstop_positions_degs = {-135, 90, 68};
  ActuatorVector calibration_directions = {-1, 1, 1};
  std::array<std::string, K_SERVOS> joint_names = {
      "leg_front_r_1", "leg_front_r_2", "leg_front_r_3"};

  ActuatorVector default_position = {0, 0, 1.0};

  auto controller = std::make_unique<pupperv3::MotorController<K_SERVOS>>(
      position_kp, speed_kp, max_speed, endstop_positions_degs, calibration_directions,
      std::move(interface));
  auto node = std::make_shared<pupperv3::MotorControllerNode<K_SERVOS>>(
      publish_rate, std::move(controller), joint_names, default_position);

  node->startup();
  rclcpp::spin(node);
  std::cout << "rclcpp done spinning" << std::endl;
  rclcpp::shutdown();
  std::cout << "rclcpp shutdown" << std::endl;
  return 0;
}
