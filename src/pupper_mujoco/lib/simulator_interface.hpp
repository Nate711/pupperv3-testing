#ifndef H_SIMULATOR_INTERFACE
#define H_SIMULATOR_INTERFACE

#include <vector>
#include <array>

class SimulatorInterface
{
public:
    virtual void single_step() = 0;
    virtual void render() = 0;
    virtual bool should_close() = 0;
    virtual void set_actuator_torques(std::vector<double> torques) = 0;
    virtual double sim_time() = 0;
    virtual int n_actuators() = 0;
    virtual std::vector<double> actuator_positions() = 0;
    virtual std::vector<double> actuator_velocities() = 0;
    virtual std::vector<double> actuator_efforts() = 0;
    virtual std::array<double, 3> base_position() = 0;
    virtual std::array<double, 4> base_orientation() = 0;
    virtual std::array<double, 3> base_velocity() = 0;
    virtual std::array<double, 3> base_angular_velocity() = 0;
};

#endif