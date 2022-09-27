#include "basic_sim_node.hpp"

using std::placeholders::_1;

BasicSimNode::BasicSimNode(bool fixed_base, float publish_rate) : Node("basic_sim_node"),
                                                                  basic_sim_(fixed_base),
                                                                  publish_rate_(publish_rate)
{
    if (fixed_base)
    {
        basic_sim_.initialize("/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_fixed_base.xml");
    }
    else
    {
        basic_sim_.initialize("/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_floating_base.xml");
    }
    // Initialize persistent joint state message
    for (std::string joint_name : joint_names_)
    {
        joint_state_message_.name.push_back(joint_name);
        joint_state_message_.position.push_back(0.0);
        joint_state_message_.velocity.push_back(0.0);
        joint_state_message_.effort.push_back(0.0);
    }

    physics_timer_ = this->create_wall_timer(
        rclcpp::WallRate(kPhysicsRate).period(),
        std::bind(&BasicSimNode::single_step, this));

    render_timer_ = this->create_wall_timer(
        rclcpp::WallRate(kRenderRate).period(),
        std::bind(&BasicSimNode::render, this));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    timer_ = this->create_wall_timer(
        rclcpp::WallRate(publish_rate_).period(),
        std::bind(&BasicSimNode::publish_callback, this));

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/actuator_commands",
        kSubscriberHistory,
        std::bind(&BasicSimNode::actuator_control_callback, this, _1));
}

void BasicSimNode::render_thread()
{
    while (!basic_sim_.should_close())
    {
        basic_sim_.step_and_render();
    }
}

void BasicSimNode::render()
{
    basic_sim_.render();
}

void BasicSimNode::single_step()
{
    // debug only
    basic_sim_.set_actuator_torques({
        std::sin(basic_sim_.sim_time() * 2),
        0.0,
        0.0,
        std::cos(basic_sim_.sim_time() * 2),
        0.0,
        0.0,
        std::sin(basic_sim_.sim_time() * 2),
        0.0,
        0.0,
        std::cos(basic_sim_.sim_time() * 2),
        0.0,
        0.0,
    });

    basic_sim_.single_step();
}

void BasicSimNode::publish_callback()
{
    joint_state_message_.header.stamp = now();
    for (int i = 0; i < basic_sim_.actuator_positions().size(); i++)
    {
        joint_state_message_.position.at(i) = basic_sim_.actuator_positions().at(i);
        joint_state_message_.velocity.at(i) = basic_sim_.actuator_velocities().at(i);
    }

    // Causes double free or corruption when ctrl-c physics step
    // And segmentation fault on rendering
    // std::copy(std::begin(basic_sim_.actuator_positions()),
    //           std::end(basic_sim_.actuator_positions()),
    //           std::begin(joint_state_message_.position));
    // std::copy(std::begin(basic_sim_.actuator_velocities()),
    //           std::end(basic_sim_.actuator_velocities()),
    //           std::begin(joint_state_message_.velocity));

    publisher_->publish(joint_state_message_);
}

void BasicSimNode::actuator_control_callback(const std_msgs::msg::String &msg)
{
    basic_sim_.set_actuator_torques({
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    });
}

int main(int argc, char *argv[])
{
    /*
     * spin is a single-threaded executor
     * one callback for publishing at 500hz?
     * one callback for subscribing to control messages
     */
    rclcpp::init(argc, argv);
    bool fixed_base = true;
    float publish_rate = 50.0;
    rclcpp::spin(std::make_shared<BasicSimNode>(/*fixed_base=*/fixed_base, /*publish_rate=*/publish_rate));
    rclcpp::shutdown();
    return 0;
}
