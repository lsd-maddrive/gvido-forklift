#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import can
import time
import os

'''
sudo ip link set can0 up type can bitrate 500000
'''
class VescCanRpmNode(Node):
    def __init__(self):
        super().__init__('vesc_can_rpm_node')

        # Parameters
        self.declare_parameter('max_mech_rpm', 150)     # максимальная линейная скорость
        self.declare_parameter('max_turn_rpm', 120)      # максимальная скорость поворота (НОВЫЙ параметр)
        self.declare_parameter('pole_pairs', 15)
        self.declare_parameter('vesc_id_1', 0x76)
        self.declare_parameter('vesc_id_2', 0x73)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('reconnect_period', 2.0)

        self.max_mech_rpm   = self.get_parameter('max_mech_rpm').value
        self.max_turn_rpm   = self.get_parameter('max_turn_rpm').value
        self.pole_pairs     = self.get_parameter('pole_pairs').value
        self.vesc_id_1      = self.get_parameter('vesc_id_1').value
        self.vesc_id_2      = self.get_parameter('vesc_id_2').value
        self.can_interface  = self.get_parameter('can_interface').value
        self.reconnect_period = self.get_parameter('reconnect_period').value

        # CAN command ID
        self.cmd_set_rpm = 3
        self.can_id_1 = (self.cmd_set_rpm << 8) | self.vesc_id_1
        self.can_id_2 = (self.cmd_set_rpm << 8) | self.vesc_id_2

        # Internal state
        self.bus = None
        self.can_ok_printed = False
        self.can_error_printed = False

        # RPM for each motor
        self.left_erpm = 0
        self.right_erpm = 0

        # Subscribe to joystick
        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        # Timers
        self.create_timer(0.02, self.send_can_loop)
        self.create_timer(self.reconnect_period, self.check_can_connection)

        self.get_logger().info("\033[32mVESC CAN RPM node started.\033[0m")


    # ================================================================
    #                    CAN CONNECTOR
    # ================================================================
    def check_can_connection(self):
        if self.bus is not None:
            return

        try:
            self.bus = can.Bus(interface='socketcan', channel=self.can_interface)
            if not self.can_ok_printed:
                self.get_logger().info(
                    f"\033[32mCAN connected to {self.can_interface}\033[0m")
                self.can_ok_printed = True
                self.can_error_printed = False
        except Exception as e:
            if not self.can_error_printed:
                self.get_logger().error(
                    f"\033[31mCAN ERROR: cannot connect to {self.can_interface} ({e})\033[0m")
                self.can_error_printed = True
                self.can_ok_printed = False
            self.bus = None


    # ================================================================
    #                    JOYSTICK INPUT
    # ================================================================
    def joy_callback(self, msg: Joy):

        # -----------------------------
        # ЛИНЕЙНАЯ СКОРОСТЬ (RT/LT)
        # -----------------------------
        try:
            lt_raw = msg.axes[5]
            rt_raw = msg.axes[4]
        except IndexError:
            return

        rt = (1.0 - rt_raw) / 2.0     # 0..1
        lt = (1.0 - lt_raw) / 2.0     # 0..1

        linear_rpm = rt * self.max_mech_rpm - lt * self.max_mech_rpm

        try:
            turn_axis = msg.axes[2]    
        except IndexError:
            turn_axis = 0.0

        angular_rpm = turn_axis * self.max_turn_rpm

        # -----------------------------
        # Итоговые RPM для дифференциального привода
        # -----------------------------
        left_mech_rpm  = linear_rpm - angular_rpm
        right_mech_rpm = linear_rpm + angular_rpm

        self.left_erpm  = int(left_mech_rpm  * self.pole_pairs)
        self.right_erpm = int(right_mech_rpm * self.pole_pairs)


    # ================================================================
    #                    CAN TX LOOP
    # ================================================================
    def send_can_loop(self):
        if self.bus is None:
            return

        data_left  = self.left_erpm.to_bytes(4, 'big', signed=True)
        data_right = self.right_erpm.to_bytes(4, 'big', signed=True)

        msg_left = can.Message(
            arbitration_id=self.can_id_1,
            data=data_left,
            is_extended_id=True
        )

        msg_right = can.Message(
            arbitration_id=self.can_id_2,
            data=data_right,
            is_extended_id=True
        )

        try:
            self.bus.send(msg_left)
            self.bus.send(msg_right)

        except Exception as e:
            self.get_logger().error(
                f"\033[31mCAN ERROR: Failed to send frame ({e})\033[0m")
            self.bus = None


def main(args=None):
    rclpy.init(args=args)
    node = VescCanRpmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
