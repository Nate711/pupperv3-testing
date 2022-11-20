#include "mujoco_node.hpp"
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

MujocoNode::MujocoNode(std::vector<std::string> joint_names,
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

    core_ = std::make_unique<MujocoCore>(model_xml.c_str(), floating_base, timestep);

    n_actuators_ = core_->n_actuators();
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
    timer_ = this->create_wall_timer(
        rclcpp::WallRate(publish_rate).period(),
        std::bind(&MujocoNode::joint_state_publish_callback, this));

    body_tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<pupper_interfaces::msg::JointCommand>(
        "/joint_commands",
        rclcpp::SensorDataQoS(),
        std::bind(&MujocoNode::joint_command_callback, this, _1));

    // OPTION 1 - timer blocking step-render (single-thread)
    // Works 100% except can't receive new commands faster than the rate
    // at which this function is called.
    // Setting to 60hz runs in real time, 500hz runs quite slow
    // render_timer_ = this->create_wall_timer(
    //     rclcpp::WallRate(kRenderRate).period(),
    //     std::bind(&MujocoNode::blocking_step_render, this));

    // OPTION 2 - rendering and stepping together in seperate thread (1 extra thread)
    // NOTE: before testing, put back the mutex locks
    // FAILS - black screen
    // render_thread_ = std::thread(
    //     std::bind(&MujocoNode::step_and_render_thread, this));

    // OPTION 3 - step and render on timers (single-thread)
    // WORKS
    // Seems to have good real time factor, perhaps 0.9
    physics_timer_ = this->create_wall_timer(
        rclcpp::WallRate(sim_step_rate).period(),
        std::bind(&MujocoNode::step, this));

    render_timer_ = this->create_wall_timer(
        rclcpp::WallRate(sim_render_rate).period(),
        std::bind(&MujocoNode::render, this));

    // OPTION 4 - step and render each on different threads (2 extra threads)
    // FAILS: black screen
    // NOTE: before testing, put back the mutex locks
    // step_thread_ = std::thread(
    //     std::bind(&MujocoNode::step_thread, this));

    // render_thread_ = std::thread(
    //     std::bind(&MujocoNode::render_loop, this));

    // OPTION 5 - call spin_some between every physics step
    // code is in the test file.
    // FAILS: commands and states not published and received at fast enough rate
    start_ = this->get_clock()->now();
    last_sim_step_ = std::chrono::high_resolution_clock::now();

    if (pub_step_rate_)
    {
        sim_rate_tracker_ = this->create_publisher<builtin_interfaces::msg::Duration>("/sim_step_period", rclcpp::SensorDataQoS());
    }
}

void MujocoNode::step()
{
    rosgraph_msgs::msg::Clock msg;
    msg.clock = rclcpp::Time(core_->sim_time() * 1e9, RCL_ROS_TIME);
    clock_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "step physics @ sim time %f", core_->sim_time());
    auto pos = core_->actuator_positions();
    auto vel = core_->actuator_velocities();
    for (int i = 0; i < n_actuators_; i++)
    {
        actuator_torques_.at(i) = actuator_models_.at(i)->run(latest_msg_.kp.at(i),
                                                              latest_msg_.kd.at(i),
                                                              pos.at(i),
                                                              latest_msg_.position_target.at(i),
                                                              vel.at(i),
                                                              latest_msg_.velocity_target.at(i),
                                                              latest_msg_.feedforward_torque.at(i));
    }
    core_->set_actuator_torques(actuator_torques_);
    core_->single_step(); // todo split across step1 and step2. otherwise not taking advantage of new state

    if (pub_step_rate_)
    {
        auto step_ts = std::chrono::high_resolution_clock::now();
        auto delta = duration_ns(step_ts - last_sim_step_);
        last_sim_step_ = step_ts;
        builtin_interfaces::msg::Duration d;
        d.sec = delta / 1000000000;
        d.nanosec = delta % 1000000000;
        sim_rate_tracker_->publish(d);
    }
}

void MujocoNode::blocking_step_render()
{
    auto simstart = core_->sim_time();
    while (core_->sim_time() - simstart < 1.0 / kRenderRate)
    {
        /**************/
        std::this_thread::yield(); // try to let commands process
        /**************/
        step();
    }
    core_->render();
}

void MujocoNode::render()
{
    RCLCPP_INFO(this->get_logger(), "render @ sim time %f", core_->sim_time());
    core_->render();
}

// did not fix corruption and leaves black screen
void MujocoNode::render_loop()
{
    while (!core_->should_close())
    {
        core_->render();
    }
    stop_ = true;
    step_thread_.join();
}

/**************
 * No segfaults, but black screen
 * RVIZ fails
 ***************/
void MujocoNode::step_and_render_thread()
{
    while (!core_->should_close())
    {
        auto simstart = core_->sim_time();
        while (core_->sim_time() - simstart < 1.0 / kRenderRate)
        {
            /**************/
            std::this_thread::yield(); // try to let commands process
            /**************/
            step();
        }
        core_->render();
    }
}

/**************
 *
 ***************/
void MujocoNode::step_and_render_loop_spinsome()
{
    while (!core_->should_close())
    {
        auto simstart = core_->sim_time();
        while (core_->sim_time() - simstart < 1.0 / kRenderRate)
        {
            /**************/
            // try
            // {
            rclcpp::spin_some(this->get_node_base_interface());
            // }
            // catch (const rclcpp::exceptions::RCLError &e)
            // {
            //     RCLCPP_ERROR(
            //         this->get_logger(),
            //         "unexpectedly failed with %s",
            //         e.what());
            // }
            /**************/
            step();
        }
        core_->render();
    }
}

void MujocoNode::step_thread()
{
    while (!stop_)
    {
        auto simstart = core_->sim_time();
        while (core_->sim_time() - simstart < 1.0 / kRenderRate)
        {
            std::this_thread::yield(); // try to let commands process
            step();
        }
    }
}

void MujocoNode::joint_state_publish_callback()
{
    RCLCPP_INFO(this->get_logger(), "pub joint state @ %f", core_->sim_time());

    joint_state_message_.header.stamp = this->get_clock()->now();
    joint_state_message_.position = core_->actuator_positions(); // slower than mem copy?
    joint_state_message_.velocity = core_->actuator_velocities();
    joint_state_message_.effort = core_->actuator_efforts();
    publisher_->publish(joint_state_message_);

    body_tf_.header.stamp = this->get_clock()->now();
    auto body_pos = core_->base_position();
    auto body_quat = core_->base_orientation();
    body_tf_.transform.translation.x = body_pos.at(0);
    body_tf_.transform.translation.y = body_pos.at(1);
    body_tf_.transform.translation.z = body_pos.at(2);
    body_tf_.transform.rotation.w = body_quat.at(0);
    body_tf_.transform.rotation.x = body_quat.at(1);
    body_tf_.transform.rotation.y = body_quat.at(2);
    body_tf_.transform.rotation.z = body_quat.at(3);
    body_tf_broadcaster_->sendTransform(body_tf_);
}

void MujocoNode::joint_command_callback(const pupper_interfaces::msg::JointCommand &msg)
{
    RCLCPP_INFO(this->get_logger(), "recv joint command @ sim time %f", core_->sim_time());
    latest_msg_ = msg;
    // std::cout << "pos target: " << msg.position_target << std::endl;
}
