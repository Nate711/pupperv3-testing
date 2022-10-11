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
using std::placeholders::_1;

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

    subscriber_ = this->create_subscription<pupper_interfaces::msg::JointCommand>(
        "/joint_commands",
        rclcpp::SensorDataQoS(),
        std::bind(&MotorControllerNode::joint_command_callback, this, _1));
}

MotorControllerNode::~MotorControllerNode()
{
}

/*
 * TODO: add velocity, feedforward, kp, kd
 */
void MotorControllerNode::joint_command_callback(pupper_interfaces::msg::JointCommand joint_command)
{
    RCLCPP_INFO(this->get_logger(), "GOT JOINT COMMAND");
    latest_joint_command_ = joint_command;
}

std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> MotorControllerNode::split_vector(
    std::vector<double> vector)
{
    std::vector<std::array<float, K_SERVOS_PER_CHANNEL>> data;
    for (size_t i = 0; i < vector.size() / K_SERVOS_PER_CHANNEL; i++)
    {
        std::array<float, K_SERVOS_PER_CHANNEL> section;
        for (size_t j = 0; j < K_SERVOS_PER_CHANNEL; j++)
        {
            section.at(j) = vector.at(j + K_SERVOS_PER_CHANNEL * i);
        }
        data.push_back(section);
    }
    return data;
}

void MotorControllerNode::publish_callback()
{
    // must be running to get motor feedback
    motor_controller_.run(split_vector(latest_joint_command_.position_target));

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