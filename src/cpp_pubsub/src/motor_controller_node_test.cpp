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
#include "motor_controller_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  float rate = 500;
  if (argc > 1) {
    rate = std::stof(argv[1]);
  }
  std::cout << "Joint state publish rate: " << rate << std::endl;

  float position_kp = 10000.0;  // 10000 is good default. units rotor deg/s per output rad
  uint8_t speed_kp = 5;       // 5 is good default. units A/rotor deg/s
  float max_speed = 5000;     // rotor deg/s

  auto node = std::make_shared<MotorControllerNode>(rate, position_kp, speed_kp, max_speed);
  node->startup();
  rclcpp::spin(node);
  std::cout << "rclcpp done spinning" << std::endl;
  rclcpp::shutdown();
  std::cout << "rclcpp shutdown" << std::endl;
  return 0;
}
