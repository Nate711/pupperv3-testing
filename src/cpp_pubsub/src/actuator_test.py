import can
import time
import math
import struct

import rclpy
from rclpy.node import Node


from sensor_msgs.msg import JointState

RATE = 100

CHANNEL = "can0"
LOOP_RATE = 100
ENCODER_CPR = 36000
GEAR_RATIO = 10
CONVERSION_FACTOR = 2 * math.pi / (ENCODER_CPR * GEAR_RATIO)

commands = {"get_status": 0x9C, "get_angle":0x92, "cmd_vel": 0xA2, "cmd_pos": 0xA3, "startup_0": 0x76, "startup_1": 0x88, "startup_2": 0x77}

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 1 / RATE  # seconds

        self.initial_pos = [0] * 3

        for id in [1, 2, 3]:
            self.start_motor(id)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # pos1 = self.send(1, "get_angle")
        # pos2 = self.send(2, "get_angle")
        # pos3 = self.send(3, "get_angle")
        # print(pos1, '\t',
        # pos2, '\t',
        # pos3)

        msg = JointState()
        msg.name = [
            "leg_front_r_1",
            "leg_front_r_2",
            "leg_front_r_3",
            "leg_back_r_1",
            "leg_back_r_2",
            "leg_back_r_3",
            "leg_front_l_1",
            "leg_front_l_2",
            "leg_front_l_3",
            "leg_back_l_1",
            "leg_back_l_2",
            "leg_back_l_3",
            ]
        
        msg.position = [
            self.send(0, 1, "get_angle"),
            self.send(0, 2, "get_angle"),
            self.send(0, 3, "get_angle"),
            self.send(1, 1, "get_angle"),
            self.send(1, 2, "get_angle"),
            self.send(1, 3, "get_angle"),
        ] + 6 * [0.0]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def send(self, channel, id, command, value=None):
        with can.Bus(interface="socketcan", channel="can"+str(channel), bitrate=1000000) as bus:
            data = [commands[command], 0, 0, 0, 0, 0, 0, 0]
            if command in {"cmd_vel", "cmd_pos"}:
                data[4] = value & 0xFF
                data[5] = (value >> 8) & 0xFF
                data[6] = (value >> 16) & 0xFF
                data[7] = (value >> 24) & 0xFF
            msg = can.Message(arbitration_id = 0x140 + id, data = data, is_extended_id=False)
            try:
                bus.send(msg)
                if command in {"get_status", "cmd_vel", "cmd_pos"}:
                    message = bus.recv()
                    pos = message.data[6] + (message.data[7] << 8)
                    # print(pos)
                    return pos
                if command in {"get_angle"}:
                    message = bus.recv()
                    pos = struct.unpack_from('i', message.data, offset=1)[0]
                    pos = pos * CONVERSION_FACTOR
                    return pos
            except can.CanError:
                print("not sent")

    def start_motor(self, id):
        for i in (0,1):
            self.send(i, id, "startup_0")
            self.send(i, id, "startup_1")
            self.send(i, id, "startup_2")
        # self.initial_pos[id-1] = self.send(id, "get_status")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()