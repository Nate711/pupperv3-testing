#ifndef ROBOT_EMULATOR
#define ROBOT_EMULATOR

#include <vector>
#include <array>

#include "basic_sim.hpp"
#include "constants.hpp"

class RobotEmulator
{
private:
    BasicSim basic_sim_;
    std::vector<ActuatorModel> actuator_models_;

    // maybe just make a struct to mimic the interface?
    std::array<double, N_ACTUATORS> kps_;
    std::array<double, N_ACTUATORS> kds_;
    std::array<double, N_ACTUATORS> position_targets_;
    std::array<double, N_ACTUATORS> velocity_targets_;
    std::array<double, N_ACTUATORS> feedforward_torques_;

    void single_step();
    void render();

public:
    RobotEmulator(bool fixed_base, float timestep, ActuatorParams actuator_params);
    void initialize(const char *model_name);
    void step_and_render();
    bool should_close();
    double sim_time() const;

    std::array<double, N_ACTUATORS> actuator_positions() const;
    std::array<double, N_ACTUATORS> actuator_velocities() const;
    std::array<double, 3> base_position() const;
    std::array<double, 4> base_orientation() const;
    std::array<double, 3> base_velocity() const;
    std::array<double, 3> base_angular_velocity() const;

    void command_actuator_torques(std::array<double, N_ACTUATORS> kps,
                                  std::array<double, N_ACTUATORS> kds,
                                  std::array<double, N_ACTUATORS> position_targets,
                                  std::array<double, N_ACTUATORS> velocity_targets,
                                  std::array<double, N_ACTUATORS> feedforward_torques);
};

#endif // ROBOT_EMULATOR