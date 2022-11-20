#ifndef H_MUJOCO_NODE
#define H_MUJOCO_NODE

#include "actuator_model.hpp"
#include <thread>
#include <mutex>
#include <chrono>
#include "mujoco_core_interactive.hpp"
#include <memory>

#include <sensor_msgs/msg/joint_state.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class MujocoInteractiveNode : public rclcpp::Node
{
public:
    MujocoInteractiveNode(std::vector<std::string> joint_names,
                          std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models);

private:
    void joint_state_publish_callback();
    void joint_command_callback(const pupper_interfaces::msg::JointCommand &msg);

    // Underlying simulator
    std::shared_ptr<MujocoCoreInteractive> core_interactive_;

    // Actuator models
    std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models_;
    std::vector<double> actuator_torques_;

    // Joint State Publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;
    sensor_msgs::msg::JointState joint_state_message_;
    std::vector<std::string> joint_names_;

    // Body state publisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> body_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped body_tf_;

    // Clock publisher
    // Publishes every physics step
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

    // Subscriber
    rclcpp::Subscription<pupper_interfaces::msg::JointCommand>::SharedPtr subscription_;
    pupper_interfaces::msg::JointCommand latest_msg_;

    // Step rate tracker
    bool pub_step_rate_;
    rclcpp::Publisher<builtin_interfaces::msg::Duration>::SharedPtr sim_rate_tracker_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_sim_step_;

    int n_actuators_;
    std::mutex core_lock_;
    std::mutex msg_lock_;
    std::atomic<bool> stop_ = false;
};

#endif