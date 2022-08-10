import can
import time
import math
import struct

import rclpy
from rclpy.node import Node


from sensor_msgs.msg import JointState

RATE = 100

LOOP_RATE = 100
ENCODER_CPR = 36000
GEAR_RATIO = 10
CONVERSION_FACTOR = 2 * math.pi / (ENCODER_CPR * GEAR_RATIO)

COMMANDS = {
    "get_status": 0x9C,
    "get_angle": 0x92,
    "cmd_vel": 0xA2,
    "cmd_pos": 0xA3,
    "startup_0": 0x76,
    "startup_1": 0x88,
    "startup_2": 0x77,
}

CAN0 = "can0"
CAN1 = "can1"


class MotorPublisher(Node):
    def __init__(self, channels, channel_ids):
        super().__init__("motor_publisher")
        self.motor_communicator = MotorCommunicator(channels, channel_ids)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
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
            self.motor_communicator.get_multi_angle("can0", 1),
            self.motor_communicator.get_multi_angle("can0", 2),
            self.motor_communicator.get_multi_angle("can0", 3),
            self.motor_communicator.get_multi_angle("can1", 1),
            self.motor_communicator.get_multi_angle("can1", 2),
            self.motor_communicator.get_multi_angle("can1", 3),
        ] + 6 * [0.0]
        print(msg.position)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


class MotorCommunicator:
    def __init__(self, channel_ids, bitrate=1000000):
        self.bitrate = bitrate
        self.channel_ids = channel_ids
        self.initialized = False

    def canbus(self, channel):
        return can.Bus(interface="socketcan", channel=channel, bitrate=self.bitrate)

    def intialize_motors(self):
        for (channel, id) in self.channel_ids:
            self.initialize_motor(channel, id)
        self.initialized = True

    def initialize_motor(self, channel, motor_id):
        with self.canbus(channel) as bus:
            self.send(bus, motor_id, [COMMANDS["startup_0"], 0, 0, 0, 0, 0, 0, 0])
            self.send(bus, motor_id, [COMMANDS["startup_1"], 0, 0, 0, 0, 0, 0, 0])
            self.send(bus, motor_id, [COMMANDS["startup_2"], 0, 0, 0, 0, 0, 0, 0])

    def check_initialization_(self, channel, motor_id):
        if not self.initialized or (channel, motor_id) not in self.channel_ids:
            raise Exception("Motor not initialized")

    def can_id(self, motor_id):
        return 0x140 + motor_id

    def send(self, bus, motor_id, payload):
        can_id = self.can_id(motor_id)
        msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
        try:
            bus.send(msg)
        except can.CanError as e:
            print("Send error: ", e)

    def get_multi_angle(self, channel, motor_id):
        self.check_initialization_(channel, motor_id)
        with self.canbus(channel) as bus:
            try:
                self.send(
                    bus, motor_id, payload=[COMMANDS["get_angle"], 0, 0, 0, 0, 0, 0, 0]
                )
                print("sent get angle")

                msg_rx = bus.recv()
                pos = struct.unpack_from("i", msg_rx.data, offset=1)[0]
                pos = pos * CONVERSION_FACTOR
                return pos
            except can.CanError as e:
                print("get_multi_angle error: ", e)


def main(args=None):
    start = time.time()
    motor_communicator = MotorCommunicator(channel_ids=[(CAN0, 1)])
    print(f"made comm: {time.time() - start:0.5f}s")
    motor_communicator.intialize_motors()
    print(f"initialized motors {time.time() - start:0.5f}s")

    while True:
        read_start = time.time()
        multi_pos = motor_communicator.get_multi_angle(channel=CAN0, motor_id=1)
        print(f"read took {time.time() - read_start:0.5f}s")
        print(f"multi pos {multi_pos:0.3f}rad")
        time.sleep(0.01)


if __name__ == "__main__":
    main()
