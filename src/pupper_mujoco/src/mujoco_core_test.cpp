#include "mujoco_core.hpp"

#include <cstdio>

// main function
int main(int argc, const char **argv)
{
    MujocoCore core("/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_fixed_base.xml", false, 0.002);

    // run main loop, target real-time simulation and 60 fps rendering
    while (!core.should_close())
    {
        auto simstart = core.sim_time();
        while (core.sim_time() - simstart < 1.0 / 60.0)
        {
            core.single_step();
        }
        core.render();
    }
    return 1;
}