import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float32, Int8
from geometry_msgs.msg import Quaternion, Twist
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply
from math import pi
from numpy import float64

from mavros_msgs.msg import GimbalDeviceAttitudeStatus


class APUSensory(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.heading_rate = None
        self.attitude_msg = None

        self.attitude_publisher_ = self.create_subscription(
            GimbalDeviceAttitudeStatus, '/autopilot/attitude', self.attitude_subscribe_callback, 10
        )
        self.orientation_publisher_ = self.create_publisher(GnssInsOrientationStamped, '/autoware_orientation', 10)
        self.heading_rate_publisher_ = self.create_publisher(String, '/heading_rate', 10)

        self.heading_rate_timer = self.create_timer(1/50, self.heading_rate_callback)
        self.orientation_timer = self.create_timer(1/50, self.autoware_orientation_callback)

    def attitude_subscribe_callback(self, msg: GimbalDeviceAttitudeStatus):
        self.attitude_msg = msg
        self.heading_rate = msg.q.z

    def heading_rate_callback(self):
        if self.heading_rate is None:
            return
        msg = String()
        msg.data = str(self.heading_rate)
        self.heading_rate_publisher_.publish(msg)
        self.heading_rate = None

    def autoware_orientation_callback(self):
        if self.attitude_msg is None:
            return

        _msg = GnssInsOrientationStamped()
        _msg.header.stamp = self.get_clock().now().to_msg()
        _msg.header.frame_id = 'base_link'

        _msg.orientation.orientation = self.attitude_msg.q
        _msg.orientation.rmse_rotation_x = 0.05
        _msg.orientation.rmse_rotation_y = 0.05
        _msg.orientation.rmse_rotation_z = 0.05

        self.orientation_publisher_.publish(_msg)
        self.attitude_msg = None


def main():
    rclpy.init()
    apu = APUSensory("Autopilot_Sensory")
    rclpy.spin(apu)
    apu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()