#!/usr/bin/python3
"""
This node subscribes to a set of sources for the inverse kinematics and
publishes a twist type message with the desired robot speed vector
[x_dot, y_dot, theta_dot]

This node exists so that there is a consistent type of input to the inverse
kinematics node, without the user explicitly specifying it.

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 10-11-22

"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped


class KinematicsInputHandler(Node):
    def __init__(self):
        super().__init__('kinematics_input_handler')

        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_yaw = 0.0

        self.joystick_speed_multiplier = 0.09  # to limit max input speed

        self.sub_joystick = self.create_subscription(
            Joy, 'joy', self.joystickCallback, 10)
        self.sub_keyboard = self.create_subscription(
            Twist, 'cmd_vel_keyboard', self.keyboardCallback, 10)

        self.sub_other = self.create_subscription(
            TwistStamped,'move_cmd_vel', self.otherInputCallback, 10)

        self.body_vel_pub_ = self.create_publisher(
            TwistStamped, '/robot_body_vel', 10)

        self.send_cmd_period = .01  # seconds
        self.send_body_vel_timer = self.create_timer(
            self.send_cmd_period, self.bodyVelBroadcast)

    def keyboardCallback(self, msg):
        """
        Callback function for the keyboard topic. Parses a keyboard message to
        body velocities in x, y, and yaw
        @param: self
        @param: msg - Twist message format
        """
        self.cmd_vel_x = msg.linear.x
        self.cmd_vel_y = msg.linear.y
        self.cmd_vel_yaw = msg.angular.z


    def joystickCallback(self, msg):
        """
        Callback function for the joystick topic. Parses a joystick message to
        body velocities in x, y, and yaw
        @param: self
        @param: msg - Joy message format
        """
        self.cmd_vel_x = msg.axes[1] * self.joystick_speed_multiplier
        self.cmd_vel_y = msg.axes[0] * self.joystick_speed_multiplier
        self.cmd_vel_yaw = msg.axes[3] * self.joystick_speed_multiplier
        self.turbo_multiplier = (msg.buttons[5] * .01)

    def otherInputCallback(self, msg):
        """
        Callback function for a topic that is published by the action server
        that interfaces the BT (or other sources that publish a twist msg to
        this topic)
        @param: self
        @param: msg - TwistStamped message format
        """
        twist_msg = msg.twist
        self.keyboardCallback(twist_msg)

    def resetCMD(self):
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_yaw = 0.0

    def bodyVelBroadcast(self):
        # publish velocity setpoint vector to inverse kinematics node
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = self.cmd_vel_x
        twist_msg.twist.linear.y = self.cmd_vel_y
        twist_msg.twist.angular.z = self.cmd_vel_yaw

        self.body_vel_pub_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    kinematics_input_handler = KinematicsInputHandler()
    rclpy.spin(kinematics_input_handler)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kinematics_input_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
