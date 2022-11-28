#include "mujoco_interactive_node.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <mutex>
#include <memory>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define duration_ns(t) std::chrono::duration_cast<std::chrono::nanoseconds>(t).count()

using std::placeholders::_1;

/*TODO: make core use correct timestep?*/

MujocoInteractiveNode::MujocoInteractiveNode(std::vector<std::string> joint_names,
                                             std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models) : Node("mujoco_node"),
                                                                                                                     actuator_models_(actuator_models)
{
    std::string share_dir = ament_index_cpp::get_package_share_directory("pupper_mujoco");
    std::string default_urdf = share_dir + "/src/urdf/pupper_v3_fixed_base.xml";

    this->declare_parameter<std::string>("model_xml", default_urdf);
    this->declare_parameter<float>("timestep", 0.004); // TODO: currently unused
    this->declare_parameter<float>("publish_rate", 500.0);

    std::string model_xml = this->get_parameter("model_xml").as_string();
    float timestep = this->get_parameter("timestep").as_double();
    float publish_rate = this->get_parameter("publish_rate").as_double();

    // Set up mujoco simulation
    mujoco_interactive::init();
    mju::strcpy_arr(mujoco_interactive::filename, model_xml.c_str());
    mujoco_interactive::settings.loadrequest = 1;
    mujoco_interactive::loadmodel();

    n_actuators_ = mujoco_interactive::n_actuators();
    RCLCPP_INFO(this->get_logger(), "n_actuators = %d", n_actuators_);

    // Prepare node data
    allocate_messages(joint_names, n_actuators_);

    // Start pub subs
    make_pub_subs(publish_rate);
}

// TODO: put this code into mujoco_core_interactive.hpp
void MujocoInteractiveNode::gui_loop()
{
    while (mujoco_interactive::gui_is_alive())
    {
        mujoco_interactive::render_block();
    }
    mujoco_interactive::settings.exitrequest = 1;
    simulate_thread_->join();
    mujoco_interactive::free_data();
}

void MujocoInteractiveNode::make_pub_subs(const float publish_rate)
{
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

void MujocoInteractiveNode::allocate_messages(const std::vector<std::string> &joint_names, const unsigned int &n_actuators)
{
    // initialize torques
    actuator_torques_ = std::vector<double>(n_actuators, 0.0);

    // initialize joint message
    joint_state_message_.name = joint_names;
    joint_state_message_.position = std::vector<double>(n_actuators, 0.0);
    joint_state_message_.velocity = std::vector<double>(n_actuators, 0.0);
    joint_state_message_.effort = std::vector<double>(n_actuators, 0.0);

    // initialize robot state tf2 message
    body_tf_.header.frame_id = "world";
    body_tf_.child_frame_id = "base_link";

    // initialize command message
    latest_msg_.kp = std::vector<double>(n_actuators, 0.0);
    latest_msg_.kd = std::vector<double>(n_actuators, 0.0);
    latest_msg_.position_target = std::vector<double>(n_actuators, 0.0);
    latest_msg_.velocity_target = std::vector<double>(n_actuators, 0.0);
    latest_msg_.feedforward_torque = std::vector<double>(n_actuators, 0.0);
}

void MujocoInteractiveNode::start_simulation()
{
    simulate_thread_ = std::make_unique<std::thread>(mujoco_interactive::simulate);
}

void MujocoInteractiveNode::joint_state_publish_callback()
{
    RCLCPP_INFO(this->get_logger(), "pub joint state @ sim time: %f", mujoco_interactive::sim_time());

    joint_state_message_.header.stamp = this->get_clock()->now();
    joint_state_message_.position = mujoco_interactive::actuator_positions(); // slower than mem copy?
    joint_state_message_.velocity = mujoco_interactive::actuator_velocities();
    joint_state_message_.effort = mujoco_interactive::actuator_efforts();
    publisher_->publish(joint_state_message_);

    body_tf_.header.stamp = this->get_clock()->now();
    auto body_pos = mujoco_interactive::base_position();
    auto body_quat = mujoco_interactive::base_orientation();
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
    RCLCPP_INFO(this->get_logger(), "recv joint command @ sim time %f", mujoco_interactive::sim_time());
    latest_msg_ = msg;
    // std::cout << "pos target: " << msg.position_target << std::endl;
}
