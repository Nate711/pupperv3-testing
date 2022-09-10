#include "mujoco_basic_sim.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
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
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void MujocoBasicSim::end()
{
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

void MujocoBasicSim::initialize(int argc, const char **argv)
{
    // check command-line arguments
    if (argc != 2)
    {
        std::printf(" USAGE:  basic modelfile\n");
        return;
    }

    // load and compile model
    char error[1000] = "Could not load binary model";
    if (std::strlen(argv[1]) > 4 && !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb"))
    {
        m = mj_loadModel(argv[1], 0);
    }
    else
    {
        m = mj_loadXML(argv[1], 0, error, 1000);
    }
    if (!m)
    {
        mju_error_s("Load model error: %s", error);
    }

    // make data
    d = mj_makeData(m);

    // set dt
    // m->opt.timestep = 0.0001;

    // init GLFW
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
}

void MujocoBasicSim::step()
{
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0)
    {
        // mj_step(m, d);
        mj_step1(m, d);
        d->ctrl[0] = std::sin(d->time);
        mj_step2(m, d);
        std::cout << d->time << std::endl;
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

bool MujocoBasicSim::should_close()
{
    return glfwWindowShouldClose(window);
}

MujocoBasicSim::MujocoBasicSim()
{
}

MujocoBasicSim::~MujocoBasicSim() {
    std::cout << "ENDING" << std::endl;
    end();
}

// main function
int main(int argc, const char **argv)
{
    MujocoBasicSim basic_sim;
    basic_sim.initialize(argc, argv);
    // run main loop, target real-time simulation and 60 fps rendering
    while (!basic_sim.should_close())
    {
        basic_sim.step();
    }
    return 1;
}