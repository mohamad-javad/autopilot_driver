import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String, Float32, Int8
from geometry_msgs.msg import Quaternion, Twist

from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GPSRAW, GPSINPUT, VfrHud
from mavros_msgs.srv import CommandLong
from pymavlink import mavutil as mavu
from pymavlink.dialects.v20.smrc import MAVLink_autoware_mpu_message as MPU_Message
from pymavlink.dialects.v20.smrc import *

# from autopilot_driver.msg import Mpu
# from autopilot_driver.srv import SendGPSMsg, SendMPUMsg


class Autopilot(Node):
    def __init__(self):
        super().__init__('Autopilot_Base')
        self.imu_publisher_ = self.create_publisher(Imu, '/autopilot/imu', 10)
        self.attitude_publisher_ = self.create_publisher(
            GimbalDeviceAttitudeStatus, '/autopilot/attitude', 10
        )
        self.vfrHud_publisher_ = self.create_publisher(VfrHud, '/autopilot/vfr_hud', 10)
        # self.gps_service_ = self.create_service(
        #     SendGPSMsg, '/autopilot/gps_srv', self.gps_msg_responder, qos_profile=2)
        # self.mpu_service_ = self.create_service(
        #     SendMPUMsg, '/autopilot/mpu_srv', self.mpu_msg_responder, qos_profile=2)

        self.initialize_connection()
        self.reciever_timer = self.create_timer(0.01, self.reciever_callback)
        self.att_msg = None

    def initialize_connection(self):
        while True:
            try:
                self.serial_connection = mavu.mavlink_connection('/dev/ttyACM0', baud=115200)
                self.serial_connection.wait_heartbeat()
                break
            except Exception as e:
                self._logger.error('Error in connection: {}'.format(e))
                time.sleep(1)
        self._logger.info('Connection established.')

    # def gps_msg_responder(self, req: SendGPSMsg.Request, res: SendGPSMsg.Response):
    #     pass
    #
    # def mpu_msg_responder(self, req: SendMPUMsg.Request, res: SendMPUMsg.Response):
    #     pass

    def reciever_callback(self):
        msg = self.serial_connection.recv_msg()
        if msg is not None and msg.get_type() != 'BAD_DATA':
            if msg.get_type() == 'VFR_HUD':
                self.vfr_hud_callback(msg)
            elif msg.get_type() == 'ATTITUDE':
                self.attitude_callback(msg)
                self.att_msg = msg
            elif msg.get_type() == 'RAW_IMU' and self.att_msg is not None:
                self.imu_callback(self.att_msg, msg)
            # else:
                # self._logger.info('Message type: {}'.format(msg.get_type()))

    def vfr_hud_callback(self, msg):
        _msg = VfrHud()
        _msg.header.stamp = self.get_clock().now().to_msg()
        _msg.airspeed = msg.airspeed
        _msg.groundspeed = msg.groundspeed
        _msg.heading = msg.heading
        _msg.throttle = float(msg.throttle)
        _msg.climb = msg.climb

        self.vfrHud_publisher_.publish(_msg)

    def attitude_callback(self, msg):
        _msg = GimbalDeviceAttitudeStatus()
        _msg.header.stamp = self.get_clock().now().to_msg()
        _msg.q.x = msg.roll
        _msg.q.y = msg.pitch
        _msg.q.z = msg.yaw
        _msg.q.w = 0.0
        _msg.angular_velocity_x = msg.rollspeed
        _msg.angular_velocity_y = msg.pitchspeed
        _msg.angular_velocity_z = msg.yawspeed

        self.attitude_publisher_.publish(_msg)

    def imu_callback(self, att_msg, imu_msg):
        _msg = Imu()
        _msg.header.stamp = self.get_clock().now().to_msg()
        _msg.orientation.x = att_msg.roll
        _msg.orientation.y = att_msg.pitch
        _msg.orientation.z = att_msg.yaw
        _msg.orientation.w = 0.0
        _msg.angular_velocity.x = float(imu_msg.xgyro)
        _msg.angular_velocity.y = float(imu_msg.ygyro)
        _msg.angular_velocity.z = float(imu_msg.zgyro)
        _msg.linear_acceleration.x = float(imu_msg.xacc)
        _msg.linear_acceleration.y = float(imu_msg.yacc)
        _msg.linear_acceleration.z = float(imu_msg.zacc)

        self.imu_publisher_.publish(_msg)


def main():
    rclpy.init()
    autopilot = Autopilot()
    rclpy.spin(autopilot)
    autopilot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
