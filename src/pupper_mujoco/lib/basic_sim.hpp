#ifndef MUJOCO_BASIC_SIM
#define MUJOCO_BASIC_SIM

#include <array>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

const int kNumActuators = 12;

// Not a real class because mujoco is written in a c style
// that requires global objects because it only uses
// void* function callbacks
// Specific to PupperV3
class MujocoBasicSim
{
public:
    MujocoBasicSim(bool fixed_base);
    ~MujocoBasicSim();
    void initialize(const char *model_name);
    void single_step();
    void render();
    void step_and_render();
    bool should_close();
    void set_actuator_torques(std::array<float, kNumActuators> torques);
    std::array<float, kNumActuators> actuator_positions();
    std::array<float, kNumActuators> actuator_velocities();
    std::array<float, 3> base_position();
    std::array<float, 4> base_orientation();
    std::array<float, 3> base_velocity();
    std::array<float, 3> base_angular_velocity();

private:
    void end();
    GLFWwindow *window_;
    mjvOption opt_;  // visualization options
    mjrContext con_; // custom GPU context

    const bool fixed_base_;
    const int orientation_vars = 4;
    const int position_vars = 3;
    std::array<float, kNumActuators> actuator_torques_;
};

#endif