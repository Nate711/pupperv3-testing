#ifndef H_MUJOCO_NODE
#define H_MUJOCO_NODE

#include "mujoco_core.hpp"
#include "actuator_model.hpp"
#include <thread>
#include <mutex>

#include <sensor_msgs/msg/joint_state.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class MujocoNode : public rclcpp::Node
{
private:
    void joint_state_publish_callback();
    void joint_command_callback(const pupper_interfaces::msg::JointCommand &msg);
    void step();
    void step_thread();
    void step_and_render_thread();
    void blocking_step_render();
    void render();
    void render_loop();
    void publish_clock();

    std::thread step_thread_;
    std::thread render_thread_;

    // TODO: allow any simulator
    MujocoCore core_;
    // TODO: allow any actuator model
    // much of the same logic should be shared
    // regardless of actuator model and simulator
    ActuatorModel actuator_model_;
    std::vector<double> actuator_torques_;

    // Physics timer
    rclcpp::TimerBase::SharedPtr physics_timer_;

    // Render timer
    const float kRenderRate = 60.0;
    rclcpp::TimerBase::SharedPtr render_timer_;

    // Joint State Publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const float publish_rate_;
    sensor_msgs::msg::JointState joint_state_message_;
    // For publishing joint states
    std::vector<std::string> joint_names_;

    // Clock publisher
    // Publishes every physics step
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

    // Subscriber
    rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr subscription_;
    const int kSubscriberHistory = 1;
    pupper_interfaces::msg::JointCommand latest_msg_;

    int n_actuators_;
    std::mutex core_lock_;
    std::mutex msg_lock_;
    std::atomic<bool> stop_ = false;

    rclcpp::Time start_;

public:
    MujocoNode(const char *model_xml,
               bool floating_base,
               float timestep,
               std::vector<std::string> joint_names,
               ActuatorModel actuator_model,
               float publish_rate);
    void step_and_render_loop_spinsome();
};

#endif