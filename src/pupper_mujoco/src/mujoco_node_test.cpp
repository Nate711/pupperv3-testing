#include <rclcpp/rclcpp.hpp>
#include "mujoco_node.hpp"
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
    ActuatorModel actuator_model(params);
    std::vector<std::shared_ptr<ActuatorModelInterface>> actuator_models(12, std::make_shared<ActuatorModel>(params));

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

    rclcpp::spin(std::make_shared<MujocoNode>(
        joint_names,
        actuator_models));

    // Option 5: joint commands not read fast enough?
    // MujocoNode node(model_xml,
    //                 floating_base,
    //                 timestep,
    //                 joint_names,
    //                 actuator_model,
    //                 publish_rate);
    // node.step_and_render_loop_spinsome();
    rclcpp::shutdown();
    return 0;
}
