#ifndef H_MUJOCO_BASE
#define H_MUJOCO_BASE

#include <vector>
#include <array>
#include "simulator_interface.hpp"
#include <iostream>
#include <mujoco/mujoco.h>

class MujocoBase : public SimulatorInterface
{
protected:
    const bool floating_base_;
    const float timestep_;
    int n_actuators_;
    int n_coordinates_;
    int n_coordinate_dts_;

    const int kOrientationVars = 4;
    const int kPositionVars = 3;
    const int kBaseVelocityVars = 6;

public:
    virtual void single_step() = 0;
    virtual void render() = 0;
    virtual bool should_close() = 0;
    virtual void set_actuator_torques(std::vector<double> torques) = 0;
    virtual double sim_time() = 0;
    virtual int n_actuators() = 0;

    virtual mjtNum *qpos() = 0;
    virtual mjtNum *qvel() = 0;

    MujocoBase(bool floating_base, float timestep) : floating_base_(floating_base), timestep_(timestep)
    {
    }`

    std::vector<double> actuator_positions()
    {
        // std::unique_lock<std::mutex> lock(protect_model_data_);
        int start_idx = floating_base_ ? kOrientationVars + kPositionVars : 0;
        return std::vector<double>(qpos() + start_idx, qpos() + start_idx + n_actuators_);
    }
    std::vector<double> actuator_velocities()
    {
        // std::unique_lock<std::mutex> lock(protect_model_data_);
        int start_idx = floating_base_ ? kBaseVelocityVars : 0;
        return std::vector<double>(qvel() + start_idx, qvel() + start_idx + n_actuators_);
    }
    std::array<double, 4> base_orientation()
    {
        // std::unique_lock<std::mutex> lock(protect_model_data_);
        if (floating_base_)
        {
            return {qpos()[3], qpos()[4], qpos()[5], qpos()[6]};
        }
        else
        {
            return {1.0, 0.0, 0.0, 0.0};
        }
    }
    std::array<double, 3> base_position()
    {
        // std::unique_lock<std::mutex> lock(protect_model_data_);
        if (floating_base_)
        {
            return {qpos()[0], qpos()[1], qpos()[2]};
        }
        else
        {
            return {0.0, 0.0, 0.0};
        }
    }
    std::array<double, 3> base_angular_velocity()
    {
        // std::unique_lock<std::mutex> lock(protect_model_data_);
        if (floating_base_)
        {
            return {qvel()[3], qvel()[4], qvel()[5]};
        }
        else
        {
            return {0.0, 0.0, 0.0};
        }
    }
    std::array<double, 3> base_velocity()
    {
        // std::unique_lock<std::mutex> lock(protect_model_data_);
        if (floating_base_)
        {
            return {qvel()[0], qvel()[1], qvel()[2]};
        }
        else
        {
            return {0.0, 0.0, 0.0};
        }
    }
};

#endif