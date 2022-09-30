# Prereqs
Install mujoco v2.2.2

Change hard-coded paths in 
``src/pupper_mujoco/lib/basic_sim_node.cpp``. I would make these relative if I knew how...

# Simulation
```
colcon build
source install/local_setup.bash
ros2 run pupper_mujoco basic_sim_node_test
```

In another terminal
```
source install/local_setup.bash
ros2 run mock_controller mock_pub
```

# Write custom controller
Write a node that publishes
``JointCommand`` messages. Which for reference looks like
```
std_msgs/Header header

string[12] name
float64[12] kp
float64[12] kd
float64[12] position_target
float64[12] velocity_target
float64[12] feedforward_torque
```
Note that the array sizes are hard-coded to 12, and they become ``std:array<double, 12>``. Should probably make variable-length.

# TODO
Refactor everything to adopt any robot xml. Which means number of actuators can be variable. Allow plugging in a custom actuator model.