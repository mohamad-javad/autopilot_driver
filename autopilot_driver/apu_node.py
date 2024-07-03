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

from autopilot_msgs.msg import Mpu
from autopilot_msgs.srv import SendGPS, SendMPUMsg


class Autopilot(Node):
    def __init__(self):
        super().__init__('Autopilot_Base')
        self.imu_publisher_ = self.create_publisher(Imu, '/autopilot/imu', 10)
        self.attitude_publisher_ = self.create_publisher(
            GimbalDeviceAttitudeStatus, '/autopilot/attitude', 10
        )
        self.vfrHud_publisher_ = self.create_publisher(VfrHud, '/autopilot/vfr_hud', 10)
        self.gps_service_ = self.create_service(
            SendGPS, '/autopilot/gps_srv', self.gps_msg_responder)
        self.mpu_service_ = self.create_service(
            SendMPUMsg, '/autopilot/mpu_srv', self.mpu_msg_responder)

        self.initialize_connection()
        self.reciever_timer = self.create_timer(0.01, self.reciever_callback)
        self.att_msg = None
        self.gps_msg: GPSINPUT = None
        self.mpu_msg: Mpu = None
        self.is_gps_msg = False
        self.is_mpu_msg = False

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
            if self.is_gps_msg:
                self.serial_connection.mav.send(self.gps_msg)
                self.is_gps_msg = False
            if self.is_mpu_msg:
                self.serial_connection.mav.send(self.mpu_msg)
                self.is_mpu_msg = False

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
        _msg.angular_velocity.x = float(att_msg.rollspeed)
        _msg.angular_velocity.y = float(att_msg.pitchspeed)
        _msg.angular_velocity.z = float(att_msg.yawspeed)
        _msg.linear_acceleration.x = float(imu_msg.xacc)
        _msg.linear_acceleration.y = float(imu_msg.yacc)
        _msg.linear_acceleration.z = float(imu_msg.zacc)

        self.imu_publisher_.publish(_msg)


    def gps_msg_responder(self, req: SendGPS.Request, res: SendGPS.Response):
        try:
            g_msg = req.gps_msg
            self.gps_msg = mavu.mavlink.MAVLink_gps_input_message(
                time_usec=int(g_msg.time_usec),
                gps_id=int(g_msg.gps_id),
                ignore_flags=int(g_msg.ignore_flags),
                time_week_ms=int(g_msg.time_week_ms),
                time_week=int(g_msg.time_week),
                fix_type=int(g_msg.fix_type),
                lat=int(g_msg.lat),
                lon=int(g_msg.lon),
                alt=int(g_msg.alt),
                hdop=int(g_msg.hdop),
                vdop=int(g_msg.vdop),
                vn=int(g_msg.vn),
                ve=int(g_msg.ve),
                vd=int(g_msg.vd),
                speed_accuracy=int(g_msg.speed_accuracy),
                horiz_accuracy=int(g_msg.horiz_accuracy),
                vert_accuracy=int(g_msg.vert_accuracy),
                satellites_visible=int(g_msg.satellites_visible)
            )

        except Exception as e:
            self._logger.error('Error in creating GPS message: {}'.format(e))
            res.success = False
            return res

        self.is_gps_msg = True
        res.success = True
        return res

    def mpu_msg_responder(self, req: SendMPUMsg.Request, res: SendMPUMsg.Response):
        try:
            m_msg = req.mpu_msg
            self.mpu_msg = MPU_Message(
                preamble=m_msg.preamble,
                mode=m_msg.mode,
                sta_ref=m_msg.sta_ref,
                str_ref=m_msg.str_ref,
                yaw_ref=m_msg.yaw_ref,
                yaw_rate_ref=m_msg.yaw_rate_ref,
                swa=m_msg.swa,
                swar=m_msg.swar,
                reserved0=m_msg.reserved0,
                reserved1=m_msg.reserved1,
                reserved2=m_msg.reserved2,
                reserved3=m_msg.reserved3,
                speed_ref=m_msg.speed_ref,
                acc_ref=m_msg.acc_ref,
                jerk_ref=m_msg.jerk_ref,
                gpa_ref=m_msg.gpa_ref,
                ebrake_ref=m_msg.ebrake_ref,
                hbrake_ref=m_msg.hbrake_ref,
                reserved4=m_msg.reserved4,
                reserved5=m_msg.reserved5,
                reserved6=m_msg.reserved6,
                gear=m_msg.gear,
                mode_fb=m_msg.mode_fb,
                switch_fb=m_msg.switch_fb,
                steer_fb=m_msg.steer_fb,
                acc_fb=m_msg.acc_fb,
                ebrake_fb=m_msg.ebrake_fb,
                hbrake_fb=m_msg.hbrake_fb
            )
        except Exception as e:
            self._logger.error('Error in creating MPU message: {}'.format(e))
            res.success = False
            return res

        self.is_mpu_msg = True
        res.success = True
        return res


def main():
    rclpy.init()
    autopilot = Autopilot()
    rclpy.spin(autopilot)
    autopilot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
