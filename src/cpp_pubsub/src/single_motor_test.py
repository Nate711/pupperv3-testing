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

COMMANDS = {"get_status": 0x9C, 
            "get_angle":0x92,
            "cmd_vel": 0xA2, 
            "cmd_pos": 0xA3, 
            "startup_0": 0x76, 
            "startup_1": 0x88, 
            "startup_2": 0x77}

class MotorPublisher(Node):
    def __init__(self, channels, channel_ids):
            super().__init__('motor_publisher')
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
            self.start_motor(channel, id)
        self.initialized = True
    
    def check_initialization_(self, channel, motor_id):
        if not self.initialized or (channel, motor_id) not in self.channel_ids:
            raise Exception("Motor not initialized")
    
    def start_motor(self, channel, id):
        self.send(channel, id, "startup_0")
        self.send(channel, id, "startup_1")
        self.send(channel, id, "startup_2")

    def can_id(self, motor_id):
        return 0x140 + motor_id

    def send(self, channel, motor_id, payload):
        with self.canbus(channel) as bus:
            can_id = self.can_id(motor_id)
            msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
            try:
                bus.send(msg)
            except can.CanError as e:
                print("Send error: ", e)
    
    def start_motor(self, channel, motor_id):
        self.send(channel, motor_id, [COMMANDS["startup_0"], 0, 0, 0, 0, 0, 0, 0])
        self.send(channel, motor_id, [COMMANDS["startup_1"], 0, 0, 0, 0, 0, 0, 0])
        self.send(channel, motor_id, [COMMANDS["startup_2"], 0, 0, 0, 0, 0, 0, 0])

    def get_multi_angle(self, channel, motor_id):
        self.check_initialization_(channel, motor_id)
        self.send(channel, motor_id, [COMMANDS["get_angle"], 0, 0, 0, 0, 0, 0, 0])
        with self.canbus(channel) as bus:
            try:
                msg = bus.recv()
                pos = struct.unpack_from('i', msg.data, offset=1)[0]
                pos = pos * CONVERSION_FACTOR
                return pos
            except can.CanError as e:
                print("get_multi_angle error: ", e)

def main(args=None):
    motor_communicator = MotorCommunicator(channel_ids=[("can0", 1)])
    motor_communicator.intialize_motors()
    motor_communicator.get_multi_angle(channel="can0", motor_id=1)


if __name__ == '__main__':
    main()