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
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_interface.hpp"

#include "motor_controller_node.hpp"

using namespace std::chrono_literals;

#define M_DEG_TO_RAD 0.01745329252
#define M_GEAR_RATIO 0.10

MotorControllerNode::MotorControllerNode(float rate,
                                         float position_kp,
                                         uint8_t speed_kp,
                                         float max_speed)
    : Node("motor_controller_node"),
      publish_rate_(rate),
      motor_controller_(position_kp, speed_kp, max_speed, kMotorConnections)
{
    // CAN interface setup
    motor_controller_.begin();
    std::cout << "Initialized motor controller." << std::endl;

    // Joint State joint_state_message_ setup
    for (std::string joint_name : joint_names_)
    {
        joint_state_message_.name.push_back(joint_name);
        joint_state_message_.position.push_back(0.0);
        joint_state_message_.velocity.push_back(0.0);
        joint_state_message_.effort.push_back(0.0);
    }

    // Node setup
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    timer_ = this->create_wall_timer(
        rclcpp::WallRate(publish_rate_).period(),
        std::bind(&MotorControllerNode::publish_callback, this));
}
MotorControllerNode::~MotorControllerNode()
{
}

void MotorControllerNode::publish_callback()
{
    motor_controller_.run({{0, 0, 0, 0, 0, 0}});

    auto latest_data = motor_controller_.motor_data_copy();
    for (int bus = 0; bus < 1; bus++)
    {
        for (int servo = 0; servo < K_SERVOS_PER_CHANNEL; servo++)
        {
            std::cout << latest_data.at(bus).at(servo).common.multi_loop_angle << "\t";
        }
    }
    std::cout << std::endl;
    joint_state_message_.header.stamp = now();
    joint_state_message_.position.at(0) = latest_data.at(0).at(0).common.multi_loop_angle * M_DEG_TO_RAD * M_GEAR_RATIO;
    joint_state_message_.position.at(1) = latest_data.at(0).at(1).common.multi_loop_angle * M_DEG_TO_RAD * M_GEAR_RATIO;
    joint_state_message_.position.at(2) = latest_data.at(0).at(2).common.multi_loop_angle * M_DEG_TO_RAD * M_GEAR_RATIO;
    joint_state_message_.position.at(3) = latest_data.at(0).at(3).common.multi_loop_angle * M_DEG_TO_RAD * M_GEAR_RATIO;
    joint_state_message_.position.at(4) = latest_data.at(0).at(4).common.multi_loop_angle * M_DEG_TO_RAD * M_GEAR_RATIO;
    joint_state_message_.position.at(5) = latest_data.at(0).at(5).common.multi_loop_angle * M_DEG_TO_RAD * M_GEAR_RATIO;

    // RCLCPP_INFO(this->get_logger(), "Publishing joint state");
    publisher_->publish(joint_state_message_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    float rate = 500;
    if (argc > 1)
    {
        rate = std::stof(argv[1]);
    }
    cout << "Rate: " << rate << endl;

    float position_kp = 50;
    uint8_t speed_kp = 0;
    float max_speed = 5000;

    rclcpp::spin(std::make_shared<MotorControllerNode>(/*rate=*/rate, position_kp, speed_kp, max_speed));
    rclcpp::shutdown();
    return 0;
}
