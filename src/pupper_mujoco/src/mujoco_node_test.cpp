#include <rclcpp/rclcpp.hpp>
#include "mujoco_node.hpp"
#include "actuator_model.hpp"

int main(int argc, char *argv[])
{
    /*
     * spin is a single-threaded executor
     * one callback for publishing at 500hz?
     * one callback for subscribing to control messages
     */
    rclcpp::init(argc, argv);

    const char *model_xml = "/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_fixed_base.xml";
    bool floating_base = false;
    float timestep = 0.001;
    float publish_rate = 500.0;
    ActuatorParams params(
        /*kp=*/2.0,
        /*kd=*/0.5,
        /*bus_voltage =*/30.0,
        /*kt=*/0.1,
        /*phase_resistance=*/0.7,
        /*saturation_torque=*/4.5,
        /*software_torque_limit =*/3.0);
    ActuatorModel actuator_model(params);
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

    rclcpp::spin(std::make_shared<MujocoNode>(model_xml,
                                              floating_base,
                                              timestep,
                                              joint_names,
                                              actuator_model,
                                              publish_rate));
    rclcpp::shutdown();
    return 0;
}
