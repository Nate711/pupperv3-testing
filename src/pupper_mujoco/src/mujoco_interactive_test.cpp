#include "mujoco_core_interactive.hpp"

#include <rclcpp/rclcpp.hpp>
#include "mujoco_interactive_node.hpp"
#include "actuator_model.hpp"
#include <memory>

int main(int argc, char *argv[])
{
    /*
     * spin is a single-threaded executor
     * one callback for publishing at 500hz?
     * one callback for subscribing to control messages
     */
    rclcpp::init(argc, argv);

    for (int i = 0; i < argc; i++)
    {
        std::cout << argv[i] << " ";
    }

    // const char *model_xml = "/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_fixed_base.xml";
    // bool floating_base = false;

    // const char *model_xml = "/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_floating_base.xml";
    // bool floating_base = true;

    // float timestep = 0.004;
    // float publish_rate = 100.0;
    // float sim_step_rate = 250.0; // 1000 seems to make legs move after a delay

    // Construct actuator models
    ActuatorParams params(
        /*kp=*/2.0,
        /*kd=*/0.5,
        /*bus_voltage =*/30.0,
        /*kt=*/0.1,
        /*phase_resistance=*/0.7,
        /*saturation_torque=*/4.5,
        /*software_torque_limit =*/3.0);
    PIDActuatorModel actuator_model(params);
    std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models(12, std::make_shared<PIDActuatorModel>(params));

    std::vector<std::string> joint_names;
    const std::string joint_names_[12] = {
        "leg_front_r_1",
        "leg_front_r_2",
        "leg_front_r_3",
        "leg_front_l_1",
        "leg_front_l_2",
        "leg_front_l_3",
        "leg_back_r_1",
        "leg_back_r_2",
        "leg_back_r_3",
        "leg_back_l_1",
        "leg_back_l_2",
        "leg_back_l_3",
    };
    for (int i = 0; i < 12; i++)
    {
        joint_names.push_back(joint_names_[i]);
    }

    std::shared_ptr<MujocoInteractiveNode> node_ptr;
    node_ptr = std::make_shared<MujocoInteractiveNode>(joint_names,
                                                       actuator_models);
    node_ptr->start_simulation();

    // Start spinning node callbacks
    std::thread spin_thread([node_ptr]()
                            { rclcpp::spin(node_ptr); });

    // Begin GUI
    node_ptr->gui_loop();

    rclcpp::shutdown();
    return 0;
}

// // // run event loop
// int main(int argc, const char **argv)
// {
//     // initialize everything
//     init();

//     // request loadmodel if file given (otherwise drag-and-drop)
//     if (argc > 1)
//     {
//         mju::strcpy_arr(filename, argv[1]);
//         settings.loadrequest = 2;
//     }

//     // start simulation thread
//     std::thread simthread(simulate);

//     // event loop
//     while (!glfwWindowShouldClose(window) && !settings.exitrequest)
//     {
//         // start exclusive access (block simulation thread)
//         mtx.lock();

//         // load model (not on first pass, to show "loading" label)
//         if (settings.loadrequest == 1)
//         {
//             loadmodel();
//         }
//         else if (settings.loadrequest > 1)
//         {
//             settings.loadrequest = 1;
//         }

//         // handle events (calls all callbacks)
//         glfwPollEvents();

//         // prepare to render
//         prepare();

//         // end exclusive access (allow simulation thread to run)
//         mtx.unlock();

//         // render while simulation is running
//         render(window);
//     }

//     // stop simulation thread
//     settings.exitrequest = 1;
//     simthread.join();

//     // delete everything we allocated
//     uiClearCallback(window);
//     free(ctrlnoise);
//     mj_deleteData(d);
//     mj_deleteModel(m);
//     mjv_freeScene(&scn);
//     mjr_freeContext(&con);

//     // terminate GLFW (crashes with Linux NVidia drivers)
// #if defined(__APPLE__) || defined(_WIN32)
//     glfwTerminate();
// #endif

//     return 0;
// }
