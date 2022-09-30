#ifndef BASIC_SIM_NODE
#define BASIC_SIM_NODE

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>

// #include "basic_sim.hpp"
#include "robot_emulator.hpp"
#include "actuator_model.hpp"

class BasicSimNode : public rclcpp::Node
{
public:
    BasicSimNode(bool fixed_base,
                 float timestep,
                 ActuatorParams actuator_params,
                 float publish_rate);

private:
    void publish_callback();
    void joint_command_callback(const pupper_interfaces::msg::JointCommand &msg);
    void step_and_render();

    RobotEmulator robot_emulator_;
    std::thread render_thread_;

    // Render timer
    const float kRenderRate = 60.0;
    rclcpp::TimerBase::SharedPtr render_timer_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const float publish_rate_;
    sensor_msgs::msg::JointState joint_state_message_;

    // Subscriber
    rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr subscription_;
    const int kSubscriberHistory = 1;

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
};

#endif