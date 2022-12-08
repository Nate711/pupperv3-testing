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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include "motor_controller_node.hpp"

std::atomic_bool quit(false);

void sigint_handler(sig_atomic_t signal) {
  quit.store(true);
  printf("Caught signal %d\n", signal);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  float rate = 500;
  if (argc > 1) {
    rate = std::stof(argv[1]);
  }
  std::cout << "Rate: " << rate << std::endl;

  float position_kp = 100;  // 10000 is good default. units rotor deg/s per output rad
  uint8_t speed_kp = 5;       // 5 is good default. units A/rotor deg/s
  float max_speed = 5000;     // rotor deg/s

  rclcpp::spin(
      std::make_shared<MotorControllerNode>(/*rate=*/rate, position_kp, speed_kp, max_speed, quit));
  rclcpp::shutdown();
  return 0;
}
