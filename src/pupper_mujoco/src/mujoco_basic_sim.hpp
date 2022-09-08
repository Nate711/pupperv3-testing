#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

mjvScene scn;   // abstract scene
mjvCamera cam;  // abstract camera
mjvOption opt;  // visualization options
mjrContext con; // custom GPU context
GLFWwindow *window;

// Not a real class because mujoco is written in a c style
// that requires global objects because it only uses
// void* function callbacks
class MujocoBasicSim
{
public:
    MujocoBasicSim();
    void initialize(int argc, const char **argv);
    void step();
    void end();
    bool should_close();

private:
};