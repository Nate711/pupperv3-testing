#ifndef MUJOCO_CORE
#define MUJOCO_CORE

#include <vector>
#include <array>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
// #include "simulator_interface.hpp"
#include "mujoco_base.hpp"
#include <mutex>

// Not a real class because mujoco is written in a c style
// that requires global objects because it only uses
// void* function callbacks
// Specific to PupperV3
class MujocoCore : public MujocoBase
{

public:
    MujocoCore(const char *model_file,
               bool floating_base,
               float timestep);
    ~MujocoCore();
    void single_step();
    void render();
    bool should_close();
    void set_actuator_torques(std::vector<double> torques);
    double sim_time();
    int n_actuators();
    mjtNum *qvel();
    mjtNum *qpos();
    mjtNum *ctrl();

private:
    void initialize(const char *model_name);

    std::mutex protect_model_data_;

    static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow *window, int button, int act, int mods);
    static void mouse_move(GLFWwindow *window, double xpos, double ypos);
    static void scroll(GLFWwindow *window, double xoffset, double yoffset);

    static GLFWwindow *window_; // GLFW window

    static mjModel *model; // MuJoCo model
    static mjData *data;   // MuJoCo data

    // mouse interaction
    static bool button_left;
    static bool button_middle;
    static bool button_right;
    static double lastx;
    static double lasty;

    static mjvScene scn;    // abstract scene
    static mjvCamera cam;   // abstract camera
    static mjvOption opt_;  // visualization options
    static mjrContext con_; // custom GPU context
};

#endif