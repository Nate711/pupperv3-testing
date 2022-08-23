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

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PupperJointStatePublisher : public rclcpp::Node
{
public:
    PupperJointStatePublisher(float rate)
        : Node("pupper_joint_state_publisher"), publish_rate(rate)
    {
        // Node setup
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
        timer_ = this->create_wall_timer(
            rclcpp::WallRate(publish_rate).period(),
            std::bind(&PupperJointStatePublisher::publish_callback, this));

        // Joint State message setup
        for (std::string joint_name : joint_names)
        {
            message.name.push_back(joint_name);
            message.position.push_back(0.0);
            message.velocity.push_back(0.0);
            message.effort.push_back(0.0);
        }

        // CAN interface setup
        motor_interface_ = std::make_unique<MotorInterface>(kMotorConnections, kBitrate);
        motor_interface_->initialize_canbuses();
        motor_interface_->initialize_motors(); // not needed if you just want angles
        motor_interface_->start_read_threads();
    }
    ~PupperJointStatePublisher()
    {
    }

private:
    void publish_callback()
    {
        motor_interface_->request_multi_angle(CANChannel::CAN0, 1);
        motor_interface_->request_multi_angle(CANChannel::CAN0, 2);
        motor_interface_->request_multi_angle(CANChannel::CAN0, 3);
        motor_interface_->request_multi_angle(CANChannel::CAN1, 1);
        motor_interface_->request_multi_angle(CANChannel::CAN1, 2);
        motor_interface_->request_multi_angle(CANChannel::CAN1, 3);

        auto latest_data = motor_interface_->latest_data();
        for (int bus = 0; bus < 2; bus++)
        {
            for (int servo = 0; servo < 3; servo++)
            {
                std::cout << latest_data.at(bus).at(servo).multi_angle << "\t";
            }
        }
        std::cout << std::endl;
        message.header.stamp = now();
        message.position.at(0) = latest_data.at(0).at(1).multi_angle;

        // RCLCPP_INFO(this->get_logger(), "Publishing joint state");
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    sensor_msgs::msg::JointState message;
    const std::string joint_names[12] = {
        "leg_front_r_1",
        "leg_front_r_2",
        "leg_front_r_3",
        "leg_back_r_1",
        "leg_back_r_2",
        "leg_back_r_3",
        "leg_front_l_1",
        "leg_front_l_2",
        "leg_front_l_3",
        "leg_back_l_1",
        "leg_back_l_2",
        "leg_back_l_3",
    };
    const float publish_rate;

    std::vector<CANChannel> kMotorConnections = {CANChannel::CAN0, CANChannel::CAN1};
    int kBitrate = 1000000;
    std::unique_ptr<MotorInterface> motor_interface_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PupperJointStatePublisher>(/*rate=*/500.0));
    rclcpp::shutdown();
    return 0;
}
