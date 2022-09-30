#ifndef MUJOCO_BASIC_SIM
#define MUJOCO_BASIC_SIM

#include <array>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "actuator_model.hpp"
#include "constants.hpp"


// Not a real class because mujoco is written in a c style
// that requires global objects because it only uses
// void* function callbacks
// Specific to PupperV3
class BasicSim
{

public:
    BasicSim(bool fixed_base, float timestep);
    ~BasicSim();
    void initialize(const char *model_name);
    void single_step();
    void render();
    void step_and_render();
    bool should_close();
    void set_actuator_torques(std::array<double, N_ACTUATORS> torques);
    double sim_time() const;
    std::array<double, N_ACTUATORS> actuator_positions() const;
    std::array<double, N_ACTUATORS> actuator_velocities() const;
    std::array<double, 3> base_position() const;
    std::array<double, 4> base_orientation() const;
    std::array<double, 3> base_velocity() const;
    std::array<double, 3> base_angular_velocity() const;

private:
    void end();
    GLFWwindow *window_;
    mjvOption opt_;  // visualization options
    mjrContext con_; // custom GPU context

    const bool fixed_base_;
    const float timestep_;
    const int kOrientationVars = 4;
    const int kPositionVars = 3;

    std::array<double, N_ACTUATORS> actuator_torques_;
};

#endif