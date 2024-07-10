import time
import os

import serial.serialutil
import binascii

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String, Float32, Int8
from geometry_msgs.msg import Quaternion, Twist

from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GPSRAW, GPSINPUT, VfrHud
from mavros_msgs.srv import CommandLong
from pymavlink import mavutil as mavu, mavparm
from pymavlink.dialects.v20.ardupilotmega import MAVLink_ipc_data_message as MPU_Message
from autopilot_msgs.msg import Mpu
from autopilot_msgs.srv import SendGPS, SendMPUMsg, SetMode


class Autopilot(Node):
    def __init__(self):
        super().__init__('Autopilot_Base')
        self.mode_name = "GUIDED"
        self.mode = 15
        self.is_rebooted = False
        self.attitude_msg = None
        self.last_sent = 0
        self.imu_msg = None
        self.vfrHdu_msg = None
        self.imu_publisher_ = self.create_publisher(Imu, '/autopilot/imu', 10)
        self.attitude_publisher_ = self.create_publisher(
            GimbalDeviceAttitudeStatus, '/autopilot/attitude', 10
        )
        self.vfrHud_publisher_ = self.create_publisher(VfrHud, '/autopilot/vfr_hud', 10)
        self.gps_service_ = self.create_service(
            SendGPS, '/autopilot/gps_srv', self.gps_msg_responder)
        self.mpu_service_ = self.create_service(
            SendMPUMsg, '/autopilot/mpu_srv', self.mpu_msg_responder)
        self.mode_service_ = self.create_service(
            SetMode, '/autopilot/set_mode', self.set_mode_responder)

        # set rates
        self.mav_parm = mavparm.MAVParmDict()

        self.initialize_connection()
        # create timers
        self.receiver_timer = self.create_timer(0.001, self.receiver_callback)
        self.vfrHud_pub_timer = self.create_timer(1/50, self.vfrHud_pub_callback)
        self.imu_pub_timer = self.create_timer(1/50, self.imu_pub_callback)
        self.att_pub_timer = self.create_timer(1/50, self.att_pub_callback)
        self.att_msg = None
        self.gps_msg: GPSINPUT = None
        self.mpu_msg: Mpu = None
        self.is_gps_msg = False
        self.is_mpu_msg = False

    def initialize_connection(self):
        while True:
            try:
                serial_path = '/dev/ttyACM'
                for i in range(10):
                    if os.path.exists(serial_path + str(i)):
                        serial_path += str(i)
                        break
                self._logger.info(f"Connecting to Autopilot in port {serial_path}")
                self.serial_connection = mavu.mavlink_connection(serial_path, baud=921600)
                self.serial_connection.wait_heartbeat()
                self.serial_connection.arducopter_disarm()
                break
            except Exception as e:
                self._logger.error('Error in connection: {}'.format(e))
                time.sleep(1)
        try:
            self._logger.info("Setting Attitude Frequency to 50")
            assert self.mav_parm.mavset(
               self.serial_connection, "sr0_extra1", 50, retries=5
            ), "Cant Set Attitude Frequency"
            self._logger.info("==> Done!")
        except Exception as e:
            self._logger.error('Error in setting frequency: {}'.format(e))
        try:
            self._logger.info("Setting Raw_IMU Frequency to 50")
            assert self.mav_parm.mavset(
                   self.serial_connection, "sr0_raw_sens", 50, retries=5
            ), "Cant Set Attitude Frequency"
            self._logger.info("==> Done!")
        except Exception as e:
            self._logger.error('Error in setting frequency: {}'.format(e))
            # try:
            #     self._logger.info("Setting VFR_HUD Frequency to 50")
            #     att_freq = False
            #     while not att_freq:
            #         att_freq = self.mav_parm.mavset(self.serial_connection, "sr0_", 50)
            # except Exception as e:
            #     self._logger.error('Error in setting frequency: {}'.format(e))
        self.reboot_and_arm()

        self._logger.info(f'Connection is established to port: <{serial_path}>')

    def reboot_and_arm(self):
        try:
            assert not self.is_rebooted, "Autopilot Initialization is Done!"
            self._logger.info("Rebooting the Autopilot. Please wait ...!")
            time.sleep(3)
            self.serial_connection.reboot_autopilot()
            time.sleep(3)
            self._logger.info("Reconnecting to the Autopilot")
            self.is_rebooted = True
            self.initialize_connection()
        except Exception as e:
            msg = str(e)
            try:
                self._logger.info("Disarming the Autopilot. Please wait ...!")
                time.sleep(2)
                self.serial_connection.arducopter_disarm()
                time.sleep(3)
                self._logger.info(f"Set the Autopilot Mode to {self.mode_name}")
                self.serial_connection.set_mode(self.mode)
                self._logger.info("Arming the Autopilot")
                self.serial_connection.arducopter_arm()
            except Exception as e:
                self._logger.error('Error in setting mode and arming the APU: {}'.format(e))
                self._logger.info("Reconnecting to Autopilot")
                self.initialize_connection()
            self._logger.info(msg)

    def receiver_callback(self):
        try:
            msg = self.serial_connection.recv_msg()
            if msg is not None and msg.get_type() != 'BAD_DATA':
                if msg.get_type() == 'VFR_HUD':
                    self.vfr_hud_callback(msg)
                elif msg.get_type() == 'ATTITUDE':
                    self.attitude_callback(msg)
                    self.att_msg = msg
                elif msg.get_type() == 'RAW_IMU':
                    self.imu_callback(msg)
        except serial.serialutil.SerialException as e:
            self._logger.error("Error in serial connection: ".format(e))
            self.initialize_connection()

        except Exception as e:
            self._logger.error("Error in reading message: ".format(e))

        try:
            if self.is_gps_msg:
                self.serial_connection.mav.send(self.gps_msg)
                self.is_gps_msg = False
            if self.mpu_msg is not None and time.time() - self.last_sent > 0.01:
                self.serial_connection.write(self.mpu_msg)
        except serial.serialutil.SerialException as e:
            self._logger.error(f"Error in sending message: {e}")
        except Exception as e:
            self._logger.error("Error in sending message: ".format(e))
            self.initialize_connection()

    def vfr_hud_callback(self, msg):
        _msg = VfrHud()
        _msg.header.stamp = self.get_clock().now().to_msg()
        _msg.airspeed = msg.airspeed
        _msg.groundspeed = msg.groundspeed
        _msg.heading = msg.heading
        _msg.throttle = float(msg.throttle)
        _msg.climb = msg.climb

        self.vfrHdu_msg = _msg

    def vfrHud_pub_callback(self):
        if not self.vfrHdu_msg is None:
            self.vfrHud_publisher_.publish(self.vfrHdu_msg)
            self.vfrHdu_msg = None

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
        self.attitude_msg = _msg

    def att_pub_callback(self):
        if self.attitude_msg is not None:
            self.attitude_publisher_.publish(self.attitude_msg)
            self.attitude_msg = None

    def imu_callback(self, imu_msg):
        _msg = Imu()
        _msg.header.stamp = self.get_clock().now().to_msg()

        _msg.orientation.w = 0.0

        _msg.linear_acceleration.x = float(imu_msg.xacc)
        _msg.linear_acceleration.y = float(imu_msg.yacc)
        _msg.linear_acceleration.z = float(imu_msg.zacc)

        self.imu_msg = _msg

    def imu_pub_callback(self):
        if self.imu_msg is not None and self.att_msg is not None:
            self.imu_msg.orientation.x = self.att_msg.roll
            self.imu_msg.orientation.y = self.att_msg.pitch
            self.imu_msg.orientation.z = self.att_msg.yaw
            self.imu_msg.angular_velocity.x = float(self.att_msg.rollspeed)
            self.imu_msg.angular_velocity.y = float(self.att_msg.pitchspeed)
            self.imu_msg.angular_velocity.z = float(self.att_msg.yawspeed)
            self.imu_publisher_.publish(self.imu_msg)
            self.imu_msg = None
        return

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
            mpu_msg = MPU_Message(
                preamble=m_msg.preamble,
                mode=m_msg.mode,
                refrences=[m_msg.sta_ref, m_msg.str_ref, m_msg.yaw_ref, m_msg.yaw_rate_ref, m_msg.swa,
                            m_msg.swar, m_msg.reserved0, m_msg.reserved1, m_msg.reserved2, m_msg.reserved3,
                            m_msg.speed_ref, m_msg.acc_ref, m_msg.jerk_ref, m_msg.gpa_ref, m_msg.ebrake_ref,
                            m_msg.hbrake_ref, m_msg.reserved4,
                            m_msg.reserved5, m_msg.reserved6, m_msg.gear],
                oauMode=m_msg.mode_fb,
                oauSwitches=m_msg.switch_fb,
                oauSteering=m_msg.steer_fb,
                oauAcc=m_msg.acc_fb,
                oauEBrake=m_msg.ebrake_fb,
                oauHBrake=m_msg.hbrake_fb
            )

            self.mpu_msg = mpu_msg.pack(self.serial_connection.mav)
            self._logger.info(f"Sending: {binascii.hexlify(self.mpu_msg)}")
        except Exception as e:
            self._logger.error('Error in creating MPU message: {}'.format(e))
            res.success = False
            return res

        self.is_mpu_msg = True
        res.success = True
        return res

    def set_mode_responder(self, msg:SetMode.Request, res: SetMode.Response):
        mode: str= msg.mode.upper()
        modes = {"GUIDED": 15, "AUTO": 10, "MANUAL": 0, "RTL": 11, "STEERING": 3, "HOLD": 4}
        self.mode = modes.get(mode) or 15
        if mode in modes.keys():
            self._logger.warn(f"Setting mode to <{mode}>")
            self.mode_name = mode
            res.success = True
            self.reboot_and_arm()

        else:
            self._logger.error("No Change on mode")
            res.success= False

        return res


def main():
    rclpy.init()
    autopilot = Autopilot()
    rclpy.spin(autopilot)
    autopilot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
