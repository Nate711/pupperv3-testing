#include "mujoco_node.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <pupper_interfaces/msg/joint_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <mutex>
#include <memory>

using std::placeholders::_1;

#define MUJOCO_NODE_DEBUG

MujocoNode::MujocoNode(const char *model_xml,
                       bool floating_base,
                       float timestep,
                       float sim_step_rate,
                       std::vector<std::string> joint_names,
                       std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models,
                       float publish_rate) : Node("mujoco_node"),
                                             core_(model_xml,
                                                   floating_base,
                                                   timestep),
                                             actuator_models_(actuator_models),
                                             sim_step_rate_(sim_step_rate),
                                             publish_rate_(publish_rate)
{
    n_actuators_ = core_.n_actuators();
#ifdef MUJOCO_NODE_DEBUG
    std::cout << "n_actuators: " << n_actuators_ << std::endl;
#endif

    // initialize torques
    actuator_torques_ = std::vector<double>(n_actuators_, 0.0);

    // initialize joint message
    joint_state_message_.name = joint_names;
    joint_state_message_.position = std::vector<double>(n_actuators_, 0.0);
    joint_state_message_.velocity = std::vector<double>(n_actuators_, 0.0);
    joint_state_message_.effort = std::vector<double>(n_actuators_, 0.0);

    // initialize command message
    latest_msg_.kp = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.kd = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.position_target = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.velocity_target = std::vector<double>(n_actuators_, 0.0);
    latest_msg_.feedforward_torque = std::vector<double>(n_actuators_, 0.0);

    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    timer_ = this->create_wall_timer(
        rclcpp::WallRate(publish_rate_).period(),
        std::bind(&MujocoNode::joint_state_publish_callback, this));

    subscription_ = this->create_subscription<pupper_interfaces::msg::JointCommand>(
        "/joint_commands",
        kSubscriberHistory,
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
        rclcpp::WallRate(sim_step_rate_).period(),
        std::bind(&MujocoNode::step, this));

    render_timer_ = this->create_wall_timer(
        rclcpp::WallRate(kRenderRate).period(),
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
    start_ = now();
}

void MujocoNode::step()
{
    rosgraph_msgs::msg::Clock msg;
    msg.clock = rclcpp::Time(core_.sim_time() * 1e9, RCL_ROS_TIME);
    clock_publisher_->publish(msg);
#ifdef MUJOCO_NODE_DEBUG
    std::cout << "step (sim time): " << core_.sim_time() << std::endl;
#endif
    for (int i = 0; i < n_actuators_; i++)
    {
        auto pos = core_.actuator_positions();
        auto vel = core_.actuator_velocities();
        actuator_torques_.at(i) = actuator_models_.at(i)->run(latest_msg_.kp.at(i),
                                                              latest_msg_.kd.at(i),
                                                              pos.at(i),
                                                              latest_msg_.position_target.at(i),
                                                              vel.at(i),
                                                              latest_msg_.velocity_target.at(i),
                                                              latest_msg_.feedforward_torque.at(i));
    }
    core_.set_actuator_torques(actuator_torques_);
    core_.single_step(); // todo split across step1 and step2. otherwise not taking advantage of new state
}

void MujocoNode::blocking_step_render()
{
    auto simstart = core_.sim_time();
    while (core_.sim_time() - simstart < 1.0 / kRenderRate)
    {
        /**************/
        std::this_thread::yield(); // try to let commands process
        /**************/
        step();
    }
    core_.render();
}

void MujocoNode::render()
{
#ifdef MUJOCO_NODE_DEBUG
    std::cout << "render (sim time): " << core_.sim_time() << std::endl;
#endif
    core_.render();
}

// did not fix corruption and leaves black screen
void MujocoNode::render_loop()
{
    while (!core_.should_close())
    {
        core_.render();
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
    while (!core_.should_close())
    {
        auto simstart = core_.sim_time();
        while (core_.sim_time() - simstart < 1.0 / kRenderRate)
        {
            /**************/
            std::this_thread::yield(); // try to let commands process
            /**************/
            step();
        }
        core_.render();
    }
}

/**************
 *
 ***************/
void MujocoNode::step_and_render_loop_spinsome()
{
    while (!core_.should_close())
    {
        auto simstart = core_.sim_time();
        while (core_.sim_time() - simstart < 1.0 / kRenderRate)
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
        core_.render();
    }
}

void MujocoNode::step_thread()
{
    while (!stop_)
    {
        auto simstart = core_.sim_time();
        while (core_.sim_time() - simstart < 1.0 / kRenderRate)
        {
            std::this_thread::yield(); // try to let commands process
            step();
        }
    }
}

void MujocoNode::joint_state_publish_callback()
{
#ifdef MUJOCO_NODE_DEBUG
    std::cout << "joint state (wall clock): " << (now() - start_).nanoseconds() / 1.0e9 << std::endl;
#endif
    joint_state_message_.header.stamp = now();
    joint_state_message_.position = core_.actuator_positions(); // slower than mem copy?
    joint_state_message_.velocity = core_.actuator_velocities();

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

void MujocoNode::joint_command_callback(const pupper_interfaces::msg::JointCommand &msg)
{
#ifdef MUJOCO_NODE_DEBUG
    std::cout << "joint command (wall clock): " << (now() - start_).nanoseconds() / 1.0e9 << std::endl;
#endif
    latest_msg_ = msg;
    // std::cout << "pos target: " << msg.position_target << std::endl;
}
