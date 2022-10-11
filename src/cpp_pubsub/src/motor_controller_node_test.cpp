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

#include <rclcpp/rclcpp.hpp>
#include "motor_controller_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    float rate = 500;
    if (argc > 1)
    {
        rate = std::stof(argv[1]);
    }
    cout << "Rate: " << rate << endl;

    float position_kp = 50; // 50 is good default
    uint8_t speed_kp = 1; // 1 is good default
    float max_speed = 5000;

    rclcpp::spin(std::make_shared<MotorControllerNode>(/*rate=*/rate, position_kp, speed_kp, max_speed));
    rclcpp::shutdown();
    return 0;
}
