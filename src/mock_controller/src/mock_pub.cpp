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
#include <iostream>
#include "math.h"

#include <rclcpp/rclcpp.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>

#define K_SERVOS_PER_CHANNEL 6

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MockController : public rclcpp::Node
{
public:
    MockController(float rate)
        : Node("mock_controller"), publish_rate(rate)
    {
        // Node setup
        publisher_ = this->create_publisher<pupper_interfaces::msg::JointCommand>("/joint_commands", 1);
        timer_ = this->create_wall_timer(
            rclcpp::WallRate(publish_rate).period(),
            std::bind(&MockController::publish_callback, this));

        message.name = {
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
        message.kp = std::vector<double>(12, 5.0);
        message.kd = std::vector<double>(12, 0.5);
        message.position_target = std::vector<double>(12, 0.0);
        message.velocity_target = std::vector<double>(12, 0.0);
        message.feedforward_torque = std::vector<double>(12, 0.0);

        start_ = now();
    }
    ~MockController()
    {
    }

private:
    void publish_callback()
    {
        message.header.stamp = now();
        
        float freq = 3.0;
        float phase = ((now() - start_).nanoseconds() / 1.0e9) * 2 * M_PI * freq;
        float amp = 1.0;
        std::cout << "phase: " << phase << std::endl;
        message.position_target.at(0) = std::sin(phase) * amp;
        message.position_target.at(3) = std::sin(phase) * amp;
        message.position_target.at(6) = -std::sin(phase) * amp;
        message.position_target.at(9) = -std::sin(phase) * amp;

        // RCLCPP_INFO(this->get_logger(), "Publishing joint state");
        publisher_->publish(message);
    }
    rclcpp::Time start_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<pupper_interfaces::msg::JointCommand>::SharedPtr publisher_;

    pupper_interfaces::msg::JointCommand message;
    const float publish_rate;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    float rate = 200;
    if (argc > 1)
    {
        rate = std::stof(argv[1]);
    }
    std::cout << "Rate: " << rate << std::endl;

    rclcpp::spin(std::make_shared<MockController>(/*rate=*/rate));
    rclcpp::shutdown();
    return 0;
}
