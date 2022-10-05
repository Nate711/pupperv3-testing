#include "mujoco_core.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <cassert>
#include <mutex>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

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

MujocoCore::MujocoCore(const char *model_file,
                       bool floating_base,
                       float timestep) : floating_base_(floating_base),
                                         timestep_(timestep)
{
    initialize(model_file);

    positions_ = std::vector<double>(n_actuators_, 0.0);
    velocities_ = std::vector<double>(n_actuators_, 0.0);
}

MujocoCore::~MujocoCore()
{
#ifdef BASIC_SIM_DEBUG
    std::cout << "ENDING" << std::endl;
#endif
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

void MujocoCore::initialize(const char *model_file)
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

    // record parsed number of actuators and states
    n_actuators_ = model->nu;
    n_coordinates = model->nq;

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

void MujocoCore::single_step()
{
    mj_step(model, data); // better when using actuation model
    // mj_step1(model, data);
    // need ctrl here?
    // mj_step2(model, data);
#ifdef BASIC_SIM_DEBUG
    std::cout << "nq: " << model->nq << " nv: " << model->nv << " nu: " << model->nu << std::endl;
    std::cout << std::setprecision(4);
    std::cout << "Sim time: " << data->time << std::endl;
#endif
}

void MujocoCore::render()
{
#ifdef BASIC_SIM_DEBUG
    std::cout << "Rendering" << std::endl;
#endif
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // Protect model and data
    {
        // std::unique_lock<std::mutex> lock(protect_model_data_);
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents(); // part of thread protected code

        // update scene and render
        mjv_updateScene(model, data, &opt_, NULL, &cam, mjCAT_ALL, &scn); // needs to be protected? yes bc model and data useage
    }

    mjr_render(viewport, &scn, &con_); // part of render func, not thread protect

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_); // part of render func, part of render func, not thread protect
}

bool MujocoCore::should_close()
{
    return glfwWindowShouldClose(window_);
}

void MujocoCore::set_actuator_torques(std::vector<double> torques)
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    assert(torques.size() == n_actuators_);
    for (int i = 0; i < n_actuators_; i++)
    {
        data->ctrl[i] = torques.at(i);
    }
}

std::vector<double> MujocoCore::actuator_positions()
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    int start_idx = floating_base_ ? kOrientationVars + kPositionVars : 0;
    for (int i = 0; i < n_actuators_; i++)
    {
        positions_.at(i) = data->qpos[i + start_idx];
    }
    return positions_;
}
std::vector<double> MujocoCore::actuator_velocities()
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    int start_idx = floating_base_ ? kBaseVelocityVars : 0;
    for (int i = 0; i < n_actuators_; i++)
    {
        velocities_.at(i) = data->qvel[i + start_idx];
    }
    return velocities_;
}
std::array<double, 4> MujocoCore::base_orientation()
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    if (floating_base_)
    {
        return {data->qpos[0], data->qpos[1], data->qpos[2], data->qpos[3]};
    }
    else
    {
        return {0.0, 0.0, 0.0, 0.0};
    }
}
std::array<double, 3> MujocoCore::base_position()
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    if (floating_base_)
    {
        return {data->qpos[4], data->qpos[5], data->qpos[6]};
    }
    else
    {
        return {0.0, 0.0, 0.0};
    }
}
std::array<double, 3> MujocoCore::base_angular_velocity()
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    if (floating_base_)
    {
        return {data->qvel[0], data->qvel[1], data->qvel[2]};
    }
    else
    {
        return {0.0, 0.0, 0.0};
    }
}
std::array<double, 3> MujocoCore::base_velocity()
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    if (floating_base_)
    {
        return {data->qvel[3], data->qvel[4], data->qvel[5]};
    }
    else
    {
        return {0.0, 0.0, 0.0};
    }
}

double MujocoCore::sim_time()
{
    // std::unique_lock<std::mutex> lock(protect_model_data_);
    return data->time;
}

int MujocoCore::n_actuators()
{
    return n_actuators_;
}