#include "mujoco_interactive_node.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <mutex>
#include <memory>
#include <chrono>

#define duration_ns(t) std::chrono::duration_cast<std::chrono::nanoseconds>(t).count()

using std::placeholders::_1;

/*TODO: make core use correct timestep?*/

MujocoInteractiveNode::MujocoInteractiveNode(std::vector<std::string> joint_names,
                                             std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models) : Node("mujoco_node"),
                                                                                                                     actuator_models_(actuator_models)
{
    this->declare_parameter<float>("publish_rate", 500.0);
    this->declare_parameter<float>("sim_step_rate", 250.0);
    this->declare_parameter<float>("sim_render_rate", 30.0);
    this->declare_parameter<std::string>("model_xml", "");
    this->declare_parameter<bool>("floating_base", false);
    this->declare_parameter<float>("timestep", 0.004);
    this->declare_parameter<bool>("publish_step_rate", false);

    std::string model_xml = this->get_parameter("model_xml").as_string();
    bool floating_base = this->get_parameter("floating_base").as_bool();
    float timestep = this->get_parameter("timestep").as_double();
    float publish_rate = this->get_parameter("publish_rate").as_double();
    float sim_step_rate = this->get_parameter("sim_step_rate").as_double();
    float sim_render_rate = this->get_parameter("sim_render_rate").as_double();
    pub_step_rate_ = this->get_parameter("publish_step_rate").as_bool();

    core_interactive_ = std::make_shared<MujocoCoreInteractive>();//model_xml.c_str(), floating_base, timestep);

    n_actuators_ = core_interactive_->n_actuators();
    RCLCPP_INFO(this->get_logger(), "n_actuators= %d", n_actuators_);

    // initialize torques
    actuator_torques_ = std::vector<double>(n_actuators_, 0.0);

    // initialize joint message
    joint_state_message_.name = joint_names;
    joint_state_message_.position = std::vector<double>(n_actuators_, 0.0);
    joint_state_message_.velocity = std::vector<double>(n_actuators_, 0.0);
    joint_state_message_.effort = std::vector<double>(n_actuators_, 0.0);

    // initialize robot state tf2 message
    body_tf_.header.frame_id = "world";
    body_tf_.child_frame_id = "base_link";

    // initialize command message
    latest_msg_.kp = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.kd = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.position_target = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.velocity_target = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.feedforward_torque = std::vector<double>(n_actuators_, 0.0);

    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    joint_state_pub_timer_ = this->create_wall_timer(
        rclcpp::WallRate(publish_rate).period(),
        std::bind(&MujocoInteractiveNode::joint_state_publish_callback, this));

    body_tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<pupper_interfaces::msg::JointCommand>(
        "/joint_commands",
        rclcpp::SensorDataQoS(),
        std::bind(&MujocoInteractiveNode::joint_command_callback, this, _1));
}

void MujocoInteractiveNode::joint_state_publish_callback()
{
    RCLCPP_INFO(this->get_logger(), "pub joint state @ %f", core_interactive_->sim_time());

    joint_state_message_.header.stamp = this->get_clock()->now();
    joint_state_message_.position = core_interactive_->actuator_positions(); // slower than mem copy?
    joint_state_message_.velocity = core_interactive_->actuator_velocities();
    joint_state_message_.effort = core_interactive_->actuator_efforts();
    publisher_->publish(joint_state_message_);

    body_tf_.header.stamp = this->get_clock()->now();
    auto body_pos = core_interactive_->base_position();
    auto body_quat = core_interactive_->base_orientation();
    body_tf_.transform.translation.x = body_pos.at(0);
    body_tf_.transform.translation.y = body_pos.at(1);
    body_tf_.transform.translation.z = body_pos.at(2);
    body_tf_.transform.rotation.w = body_quat.at(0);
    body_tf_.transform.rotation.x = body_quat.at(1);
    body_tf_.transform.rotation.y = body_quat.at(2);
    body_tf_.transform.rotation.z = body_quat.at(3);
    body_tf_broadcaster_->sendTransform(body_tf_);
}

void MujocoInteractiveNode::joint_command_callback(const pupper_interfaces::msg::JointCommand &msg)
{
    RCLCPP_INFO(this->get_logger(), "recv joint command @ sim time %f", core_interactive_->sim_time());
    latest_msg_ = msg;
    // std::cout << "pos target: " << msg.position_target << std::endl;
}
