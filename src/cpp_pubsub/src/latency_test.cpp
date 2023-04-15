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
#include "pupper_interfaces/msg/joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class LatencyTestNode : public rclcpp::Node {
 public:
  LatencyTestNode()
      : Node("LatencyTestNode"), received_joint_state_(false), received_joint_command_(false) {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&LatencyTestNode::joint_state_callback, this, std::placeholders::_1));
    joint_command_sub_ = this->create_subscription<pupper_interfaces::msg::JointCommand>(
        "/joint_commands", rclcpp::SensorDataQoS(),
        std::bind(&LatencyTestNode::joint_command_callback, this, std::placeholders::_1));
  }

 private:
  void joint_command_callback(pupper_interfaces::msg::JointCommand joint_command) {
    if (!received_joint_command_) {
      first_joint_command_time_ = this->get_clock()->now();
      received_joint_command_ = true;
      RCLCPP_INFO(this->get_logger(), "GOT JOINT COMMAND");
    }
  }

  void joint_state_callback(sensor_msgs::msg::JointState joint_state) {
    if (!received_joint_state_) {
      first_joint_state_time_ = this->get_clock()->now();
      received_joint_state_ = true;
      RCLCPP_INFO(this->get_logger(), "GOT JOINT STATE");
      auto latency = first_joint_state_time_ - first_joint_command_time_;
      double latency_s = static_cast<double>(latency.nanoseconds());
      RCLCPP_INFO(this->get_logger(), "latency (s): %f", latency_s);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  bool received_joint_state_;
  rclcpp::Time first_joint_state_time_;

  rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr joint_command_sub_;
  bool received_joint_command_;
  rclcpp::Time first_joint_command_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LatencyTestNode>();
  rclcpp::spin(node);
  std::cout << "rclcpp done spinning" << std::endl;
  rclcpp::shutdown();
  std::cout << "rclcpp shutdown" << std::endl;
  return 0;
}
