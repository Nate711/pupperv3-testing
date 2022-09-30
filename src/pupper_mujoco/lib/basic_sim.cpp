#include "basic_sim.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>
#include <iomanip>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "constants.hpp"

// #define BASIC_SIM_DEBUG

mjModel *model = NULL; // MuJoCo model
mjData *data = NULL;   // MuJoCo data

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

mjvScene scn;  // abstract scene
mjvCamera cam; // abstract camera
// mjvOption opt;  // visualization options
// mjrContext con; // custom GPU context

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(model, data);
        mj_forward(model, data);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
    {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
    {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (button_left)
    {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void BasicSim::end()
{
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con_);

    // free MuJoCo model and data
    mj_deleteData(data);
    mj_deleteModel(model);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

void BasicSim::initialize(const char *model_file)
{
    // load and compile model
    char error[1000] = "Could not load binary model";
    if (std::strlen(model_file) > 4 && !std::strcmp(model_file + std::strlen(model_file) - 4, ".mjb"))
    {
        model = mj_loadModel(model_file, 0);
    }
    else
    {
        model = mj_loadXML(model_file, 0, error, 1000);
    }
    if (!model)
    {
        mju_error_s("Load model error: %s", error);
    }

    // make data
    data = mj_makeData(model);

    // set dt
    model->opt.timestep = timestep_;

    // init GLFW
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con_);

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con_, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window_, keyboard);
    glfwSetCursorPosCallback(window_, mouse_move);
    glfwSetMouseButtonCallback(window_, mouse_button);
    glfwSetScrollCallback(window_, scroll);
}

void BasicSim::single_step()
{
    // mj_step(model, data); better when using actuation model
    mj_step1(model, data);
    for (int i = 0; i < N_ACTUATORS; i++)
    {
        data->ctrl[i] = actuator_torques_.at(i); // apply latest control values
    }
    mj_step2(model, data);
#ifdef BASIC_SIM_DEBUG
    std::cout << "nq: " << model->nq << " nv: " << model->nv << " nu: " << model->nu << std::endl;
    std::cout << std::setprecision(4);
    std::cout << "Sim time: " << data->time << std::endl;
#endif
}

void BasicSim::render()
{
#ifdef BASIC_SIM_DEBUG
    std::cout << "Rendering" << std::endl;
#endif
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model, data, &opt_, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con_);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void BasicSim::step_and_render()
{
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = data->time;
    while (data->time - simstart < 1.0 / 60.0)
    {
        single_step();
    }
    render();
}

bool BasicSim::should_close()
{
    return glfwWindowShouldClose(window_);
}

/*
 * TODO: figure out why negative required
 */
void BasicSim::set_actuator_torques(std::array<double, N_ACTUATORS> torques)
{
    for (size_t i = 0; i < N_ACTUATORS; i++)
    {
        actuator_torques_.at(i) = torques.at(i);
    }
}

std::array<double, N_ACTUATORS> BasicSim::actuator_positions() const
{
    std::array<double, N_ACTUATORS> positions;
    int start_idx = fixed_base_ ? 0 : kOrientationVars + kPositionVars;
    for (int i = 0; i < N_ACTUATORS; i++)
    {
        positions.at(i) = data->qpos[i + start_idx];
    }
    return positions;
}
std::array<double, N_ACTUATORS> BasicSim::actuator_velocities() const
{
    std::array<double, N_ACTUATORS> velocities;
    int start_idx = fixed_base_ ? 0 : 6;
    for (int i = 0; i < N_ACTUATORS; i++)
    {
        velocities.at(i) = data->qvel[i + start_idx];
    }
    return velocities;
}
std::array<double, 4> BasicSim::base_orientation() const
{
    if (fixed_base_)
    {
        return {0.0, 0.0, 0.0, 0.0};
    }
    else
    {
        return {data->qpos[0], data->qpos[1], data->qpos[2], data->qpos[3]};
    }
}
std::array<double, 3> BasicSim::base_position() const
{
    if (fixed_base_)
    {
        return {0.0, 0.0, 0.0};
    }
    else
    {
        return {data->qpos[4], data->qpos[5], data->qpos[6]};
    }
}
std::array<double, 3> BasicSim::base_angular_velocity() const
{
    if (fixed_base_)
    {
        return {0.0, 0.0, 0.0};
    }
    else
    {
        return {data->qvel[0], data->qvel[1], data->qvel[2]};
    }
}
std::array<double, 3> BasicSim::base_velocity() const
{
    if (fixed_base_)
    {
        return {0.0, 0.0, 0.0};
    }
    else
    {
        return {data->qvel[3], data->qvel[4], data->qvel[5]};
    }
}

double BasicSim::sim_time() const
{
    return data->time;
}

BasicSim::BasicSim(bool fixed_base,
                   float timestep) : fixed_base_(fixed_base),
                                     timestep_(timestep)
{
    actuator_torques_.fill(0.0);
}

BasicSim::~BasicSim()
{
#ifdef BASIC_SIM_DEBUG
    std::cout << "ENDING" << std::endl;
#endif
    end();
}