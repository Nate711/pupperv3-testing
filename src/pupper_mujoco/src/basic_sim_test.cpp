#include "basic_sim.hpp"

#include <cstdio>

// main function
int main(int argc, const char **argv)
{
    BasicSim basic_sim(true);
    // check command-line arguments
    // if (argc != 2)
    // {
    //     std::printf(" USAGE:  basic modelfile\n");
    //     return 0;
    // }
    
    // 70 sim seconds elapse in 60 real seconds
    basic_sim.initialize("/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3.xml");
    // run main loop, target real-time simulation and 60 fps rendering
    while (!basic_sim.should_close())
    {
        basic_sim.step_and_render();
    }
    return 1;
}