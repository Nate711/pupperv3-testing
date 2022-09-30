#include "robot_emulator.hpp"
#include <iostream>
#include "utils.hpp"

// #define ROBOT_EMULATOR_DEBUG

RobotEmulator::RobotEmulator(bool fixed_base,
                             float timestep,
                             ActuatorParams actuator_params) : basic_sim_(fixed_base, timestep)
{
    for (size_t i = 0; i < N_ACTUATORS; i++)
    {
        actuator_models_.push_back(ActuatorModel(actuator_params));
    }
}

/*
 * TODO: make actuator models be a class that has an array for every param
 * better memory localization or whatever
 */
void RobotEmulator::command_actuator_torques(std::array<double, N_ACTUATORS> kps,
                                             std::array<double, N_ACTUATORS> kds,
                                             std::array<double, N_ACTUATORS> position_targets,
                                             std::array<double, N_ACTUATORS> velocity_targets,
                                             std::array<double, N_ACTUATORS> feedforward_torques)
{
    kps_ = kps;
    kds_ = kds;
    position_targets_ = position_targets;
    velocity_targets_ = velocity_targets;
    feedforward_torques_ = feedforward_torques;
}

/*******************************************************/
/************ Expose underlying sim functions **********/
/**********    TODO: Refactor to be less stupid ********/
/*******************************************************/

void RobotEmulator::initialize(const char *model_name)
{
    basic_sim_.initialize(model_name);
}
void RobotEmulator::single_step()
{
    auto pos = actuator_positions();
    auto vel = actuator_velocities();

    std::array<double, N_ACTUATORS> actual_torques;
    for (size_t i = 0; i < N_ACTUATORS; i++)
    {
        // stupid for loops
        actuator_models_.at(i).set_kp(kps_.at(i));
        actuator_models_.at(i).set_kd(kds_.at(i));
        actual_torques.at(i) = actuator_models_.at(i).run(/*position=*/pos.at(i),
                                                          /*position_target=*/position_targets_.at(i),
                                                          /*velocity=*/vel.at(i),
                                                          /*velocity_target=*/velocity_targets_.at(i),
                                                          /*feedforwward_torque=*/feedforward_torques_.at(i));
    }
#ifdef ROBOT_EMULATOR_DEBUG
    std::cout << std::setprecision(5);
    std::cout << "sim time: " << basic_sim_.sim_time() << std::endl;
    std::cout << "kp: " << kps_ << std::endl;
    std::cout << "act torq: " << actual_torques << std::endl;
    std::cout << "act pos: " << pos << std::endl;
    std::cout << "act vel: " << vel << std::endl;
#endif

    basic_sim_.set_actuator_torques(actual_torques);
    basic_sim_.single_step();
}
void RobotEmulator::step_and_render()
{
    auto simstart = basic_sim_.sim_time();
    while (basic_sim_.sim_time() - simstart < 1.0 / 60.0)
    {
        single_step();
    }
    render();
}
void RobotEmulator::render()
{
    basic_sim_.render();
}
bool RobotEmulator::should_close()
{
    return basic_sim_.should_close();
}
double RobotEmulator::sim_time() const
{
    return basic_sim_.sim_time();
}
std::array<double, N_ACTUATORS> RobotEmulator::actuator_positions() const
{
    return basic_sim_.actuator_positions();
}
std::array<double, N_ACTUATORS> RobotEmulator::actuator_velocities() const
{
    return basic_sim_.actuator_velocities();
}
std::array<double, 3> RobotEmulator::base_position() const
{
    return basic_sim_.base_position();
}
std::array<double, 4> RobotEmulator::base_orientation() const
{
    return basic_sim_.base_orientation();
}
std::array<double, 3> RobotEmulator::base_velocity() const
{
    return basic_sim_.base_velocity();
}
std::array<double, 3> RobotEmulator::base_angular_velocity() const
{
    return basic_sim_.base_angular_velocity();
}