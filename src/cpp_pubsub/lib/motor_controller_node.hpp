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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>

#include "motor_controller.hpp"

#define K_SERVOS_PER_CHANNEL 6

const std::vector<CANChannel> kMotorConnections = {CANChannel::CAN0, CANChannel::CAN1};

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode(float rate,
                        float position_kp,
                        uint8_t speed_kp,
                        float max_speed);
    ~MotorControllerNode();
    static std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> split_vector(std::vector<double> vector);

private:
    void publish_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr subscriber_;
    sensor_msgs::msg::JointState joint_state_message_;
    void joint_command_callback(pupper_interfaces::msg::JointCommand joint_command);
    // static std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> split_vector(std::vector<double> vector);

    const std::string joint_names_[12] = {
        "leg_front_r_1",
        "leg_front_r_2",
        "leg_front_r_3",
        "leg_front_l_1",
        "leg_front_l_2",
        "leg_front_l_3",
        "leg_back_r_1",
        "leg_back_r_2",
        "leg_back_r_3",
        "leg_back_l_1",
        "leg_back_l_2",
        "leg_back_l_3",
    };
    const float publish_rate_;
    MotorController<K_SERVOS_PER_CHANNEL> motor_controller_;
    pupper_interfaces::msg::JointCommand latest_joint_command_;
};