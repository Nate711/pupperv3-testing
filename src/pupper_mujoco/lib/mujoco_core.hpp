#ifndef MUJOCO_CORE
#define MUJOCO_CORE

#include <vector>
#include <array>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "simulator_interface.hpp"
#include <mutex>

// Not a real class because mujoco is written in a c style
// that requires global objects because it only uses
// void* function callbacks
// Specific to PupperV3
class MujocoCore : public SimulatorInterface
{

public:
    MujocoCore(const char *model_file,
               bool floating_base,
               float timestep);
    ~MujocoCore();
    void single_step(); // run in child thread
    void render(); // run in main thread
    bool should_close();
    void set_actuator_torques(std::vector<double> torques);
    double sim_time();
    int n_actuators();
    std::vector<double> actuator_positions();
    std::vector<double> actuator_velocities();
    std::array<double, 3> base_position();
    std::array<double, 4> base_orientation();
    std::array<double, 3> base_velocity();
    std::array<double, 3> base_angular_velocity();

private:
    void initialize(const char *model_name);

    GLFWwindow *window_;
    mjvOption opt_;  // visualization options
    mjrContext con_; // custom GPU context

    const bool floating_base_;
    const float timestep_;
    int n_actuators_;
    int n_coordinates;

    const int kOrientationVars = 4;
    const int kPositionVars = 3;
    const int kBaseVelocityVars = 6;

    std::vector<double> positions_;
    std::vector<double> velocities_;

    std::mutex protect_model_data_;

    // std::vector<double> actuator_torques_; // pass to actuator model?
};

#endif