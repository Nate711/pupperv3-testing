#ifndef BASIC_SIM_NODE
#define BASIC_SIM_NODE

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "basic_sim.hpp"

class BasicSimNode : public rclcpp::Node
{
public:
    BasicSimNode(bool fixed_base, float publish_rate);

private:
    void publish_callback();
    void actuator_control_callback(const std_msgs::msg::String &msg);
    void render();
    void single_step();
    void render_thread();

    BasicSim basic_sim_;
    std::thread render_thread_;
    
    // Physics step timer
    const float kPhysicsRate = 500.0;
    rclcpp::TimerBase::SharedPtr physics_timer_;
    const float kRenderRate = 60.0;
    rclcpp::TimerBase::SharedPtr render_timer_;

    // Publisher 
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const float publish_rate_;
    sensor_msgs::msg::JointState joint_state_message_;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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