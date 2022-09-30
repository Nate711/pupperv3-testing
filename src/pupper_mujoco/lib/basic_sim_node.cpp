#include "basic_sim_node.hpp"
#include "actuator_model.hpp"
#include "constants.hpp"
#include "utils.hpp"

using std::placeholders::_1;

BasicSimNode::BasicSimNode(bool fixed_base,
                           float timestep,
                           ActuatorParams actuator_params,
                           float publish_rate) : Node("robot_emulator_node"),
                                                 robot_emulator_(fixed_base, timestep, actuator_params),
                                                 publish_rate_(publish_rate)
{
    // TODO: doesn't make sense that changing urdf logic is out here
    if (fixed_base)
    {
        robot_emulator_.initialize("/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_fixed_base.xml");
    }
    else
    {
        robot_emulator_.initialize("/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_floating_base.xml");
    }
    // Initialize persistent joint state message
    for (std::string joint_name : joint_names_)
    {
        joint_state_message_.name.push_back(joint_name);
        joint_state_message_.position.push_back(0.0);
        joint_state_message_.velocity.push_back(0.0);
        joint_state_message_.effort.push_back(0.0);
    }

    render_timer_ = this->create_wall_timer(
        rclcpp::WallRate(kRenderRate).period(),
        std::bind(&BasicSimNode::step_and_render, this));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    timer_ = this->create_wall_timer(
        rclcpp::WallRate(publish_rate_).period(),
        std::bind(&BasicSimNode::publish_callback, this));

    subscription_ = this->create_subscription<pupper_interfaces::msg::JointCommand>(
        "/joint_commands",
        kSubscriberHistory,
        std::bind(&BasicSimNode::joint_command_callback, this, _1));
}

void BasicSimNode::step_and_render()
{
    robot_emulator_.step_and_render();
}

void BasicSimNode::publish_callback()
{
    joint_state_message_.header.stamp = now();
    for (size_t i = 0; i < robot_emulator_.actuator_positions().size(); i++)
    {
        joint_state_message_.position.at(i) = robot_emulator_.actuator_positions().at(i);
        joint_state_message_.velocity.at(i) = robot_emulator_.actuator_velocities().at(i);
    }

    // Causes double free or corruption when ctrl-c physics step
    // And segmentation fault on rendering
    // std::copy(std::begin(robot_emulator_.actuator_positions()),
    //           std::end(robot_emulator_.actuator_positions()),
    //           std::begin(joint_state_message_.position));
    // std::copy(std::begin(robot_emulator_.actuator_velocities()),
    //           std::end(robot_emulator_.actuator_velocities()),
    //           std::begin(joint_state_message_.velocity));

    publisher_->publish(joint_state_message_);
}

void BasicSimNode::joint_command_callback(const pupper_interfaces::msg::JointCommand &msg)
{
    std::cout << "pos target: " << msg.position_target << std::endl;
    robot_emulator_.command_actuator_torques(msg.kp,
                                             msg.kd,
                                             msg.position_target,
                                             msg.velocity_target,
                                             msg.feedforward_torque);
}
