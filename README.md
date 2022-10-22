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

string[] name
float64[] kp
float64[] kd
float64[] position_target
float64[] velocity_target
float64[] feedforward_torque
```


# TODO
* Refactor everything to adopt any robot xml. Which means number of actuators can be variable. Allow plugging in a custom actuator model.
* 