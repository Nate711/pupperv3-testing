#include <rclcpp/rclcpp.hpp>
#include "basic_sim_node.hpp"

int main(int argc, char *argv[])
{
    /*
     * spin is a single-threaded executor
     * one callback for publishing at 500hz?
     * one callback for subscribing to control messages
     */
    rclcpp::init(argc, argv);
    bool fixed_base = false;
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

    rclcpp::spin(std::make_shared<BasicSimNode>(/*fixed_base=*/fixed_base,
                                                /*timestep=*/timestep,
                                                /*actuator_params=*/params,
                                                /*publish_rate=*/publish_rate));
    rclcpp::shutdown();
    return 0;
}
