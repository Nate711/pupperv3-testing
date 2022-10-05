import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pupper_interfaces.msg import JointCommand

import numpy as np

import time


class MockController(Node):

    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointCommand,
                                                '/joint_commands', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointCommand()
        msg.kp = tuple(np.ones(12) * 5.0)
        msg.kd = tuple(np.ones(12) * 0.5)

        phase = time.time() * 2 * 3.14
        value = np.sin(phase) * 0.5
        msg.position_target = tuple(np.ones(12) * value)
        msg.velocity_target = tuple(np.zeros(12))
        msg.feedforward_torque = tuple(np.zeros(12))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.kp)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MockController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()