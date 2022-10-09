from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pupper_mujoco",
            executable="mujoco_node_test",
            name="mujoco_node_test",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "publish_rate": 250.0,
                "sim_step_rate": 250.0,
                "sim_render_rate": 30.0,
                "model_xml":
                # "/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_fixed_base.xml",
                "/home/nathan/pupperv3-testing/src/pupper_mujoco/src/urdf/pupper_v3_floating_base.xml",
                # "floating_base": False,
                "floating_base":True,
                "timestep": 0.004
            }])
    ])