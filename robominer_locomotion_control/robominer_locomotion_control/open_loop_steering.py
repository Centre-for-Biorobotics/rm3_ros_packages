#!/usr/bin/python3
"""
Takes robot body velocities as input from joystick.
Calculates inverse kinematics to determine screw velocities.
Publishes screw velocities

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 29-08-2020

"""

import rclpy
from rclpy.parameter import Parameter

from rclpy.node import Node

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist, TwistStamped
from robominer_msgs.msg import MotorModuleCommand
from std_msgs.msg import Float64, Float64MultiArray

import numpy as np
from math import tan, pi

motors = np.array([
        'front_right',
        'rear_right',
        'rear_left',
        'front_left'])

motors_dict = {
    "front_right": "0",
    "rear_right": "1",
    "rear_left": "2",
    "front_left": "3"
}

# rows: motor modules {fl, fr, rl, rr}
# cols: strafing direction {F, B, L, R}

class OpenLoopSteering(Node):
    def __init__(self):
        super().__init__('open_loop_steering')
        self.lx = 0.15                          # m longitudinal distance
        self.ly = 0.3                           # m lateral distance
        self.screw_helix_angle = pi/6.0         # pi/6 for fl and rr screws, -pi/6 for fr and rl
        self.screw_radius = 0.078 	            # m
        self.radpersec_to_rpm = 60.0 / (2*pi)
        self.rpm_to_radpersec = 2*pi/60.0
        self.kinematics_timer_period = 0.1      # seconds
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_yaw = 0.0
        self.speed_multiplier = 0.0             # speed multiplier that is applied to the body velocity input
        self.turbo_multiplier = 0.0

        self.declare_parameter('on_robot')
        self.on_robot = self.get_parameter('on_robot').value

        if not self.on_robot:
            self.declare_parameter('which_sim')
            self.which_sim = self.get_parameter('which_sim').get_parameter_value().string_value
            self.get_logger().info(f'which simulator: {self.which_sim}')

        self.platform_kinematics = np.array([
            [-1/tan(self.screw_helix_angle), -1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [-1/tan(self.screw_helix_angle),  1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [1/tan(self.screw_helix_angle),   1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [1/tan(self.screw_helix_angle),  -1, -(self.lx + self.ly / tan(self.screw_helix_angle))]])

        if self.on_robot or self.which_sim=='gazebo':
            self.speed_multiplier = .09
            if self.on_robot:
                self.get_logger().info(f'on robot')
            else:
                self.get_logger().info(f'on gazebo, speed multiplier: {self.speed_multiplier}')

            # self.sub_joystick = self.create_subscription(Joy, 'joy', self.joystickCallback, 10)
            self.sub_keyboard = self.create_subscription(TwistStamped, 'move_cmd_vel', self.actionServerCallback, 10)
            self.publisher_motor0_commands = self.create_publisher(MotorModuleCommand, '/motor0/motor_rpm_setpoint', 10)
            self.publisher_motor1_commands = self.create_publisher(MotorModuleCommand, '/motor1/motor_rpm_setpoint', 10)
            self.publisher_motor2_commands = self.create_publisher(MotorModuleCommand, '/motor2/motor_rpm_setpoint', 10)
            self.publisher_motor3_commands = self.create_publisher(MotorModuleCommand, '/motor3/motor_rpm_setpoint', 10)
            if not self.on_robot:
                # Rotation of screws on Gazebo
                self.publisher_screw_rotation = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        else:
            self.get_logger().info(f'on vortex (probably)')
            self.speed_multiplier = 0.2
            # self.sub_keyboard = self.create_subscription(Twist, 'cmd_vel', self.keyboardCallback, 10)
            self.sub_keyboard = self.create_subscription(TwistStamped, 'move_cmd_vel', self.actionServerCallback, 10)
            self.publisher_motor0_commands = self.create_publisher(Float64, '/motor0/motor_rpm_setpoint', 10)
            self.publisher_motor1_commands = self.create_publisher(Float64, '/motor1/motor_rpm_setpoint', 10)
            self.publisher_motor2_commands = self.create_publisher(Float64, '/motor2/motor_rpm_setpoint', 10)
            self.publisher_motor3_commands = self.create_publisher(Float64, '/motor3/motor_rpm_setpoint', 10)

        self.kinematics_timer = self.create_timer(self.kinematics_timer_period, self.inverseKinematics)

    def actionServerCallback(self, msg):
        """
        Callback function for a topic that is published by the action server that interfaces the BT
        @param: self
        @param: msg - TwistStamped message format
        """
        twist_msg = msg.twist
        self.keyboardCallback(twist_msg)

    def keyboardCallback(self, msg):
        """
        Callback function for the keyboard topic. Parses a keyboard message to body velocities in x, y, and yaw
        @param: self
        @param: msg - Twist message format
        """
        self.cmd_vel_x = msg.linear.x
        self.cmd_vel_y = msg.linear.y
        self.cmd_vel_yaw = msg.angular.z


    def joystickCallback(self, msg):
        """
        Callback function for the joystick topic. Parses a joystick message to body velocities in x, y, and yaw
        @param: self
        @param: msg - Joy message format
        """
        self.cmd_vel_x = msg.axes[1]
        self.cmd_vel_y = msg.axes[0]
        self.cmd_vel_yaw = msg.axes[2]
        self.turbo_multiplier = (msg.buttons[5] * .01)

    def inverseKinematics(self):
        """
        Solves the inverse kinematic problem to calculate actuator speeds from body velocity.
        Calls to publish the individual motor speeds.
        @param: self
        """
        speed_multiplier = 0
        speed_multiplier += self.speed_multiplier + self.turbo_multiplier

        self.robot_twist = np.array([self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw]) * speed_multiplier

        self.screw_speeds = (1.0/self.screw_radius) * np.dot(self.platform_kinematics, self.robot_twist) * self.radpersec_to_rpm

        self.speedsBroadcast()

    def speedsBroadcast(self):
        '''
        Publishes the results of inverse kinematics to 4 topics, one RPM setpoint for each screw.
        @param: self
        '''
        if self.on_robot or self.which_sim=='gazebo':
            self.motor_cmd = [MotorModuleCommand() for i in range(4)]
            for m in range(4):
                self.motor_cmd[m].header.stamp = self.get_clock().now().to_msg()
                self.motor_cmd[m].header.frame_id = motors[m]
                self.motor_cmd[m].motor_id = motors_dict[motors[m]]
                self.motor_cmd[m].motor_rpm_goal = int(self.screw_speeds[m])
            if not self.on_robot:
                screw_velocities = Float64MultiArray()
                screw_velocities.data.append(self.screw_speeds[0] * self.rpm_to_radpersec)
                screw_velocities.data.append(-self.screw_speeds[1] * self.rpm_to_radpersec)
                screw_velocities.data.append(self.screw_speeds[2] * self.rpm_to_radpersec)
                screw_velocities.data.append(-self.screw_speeds[3] * self.rpm_to_radpersec)
                self.publisher_screw_rotation.publish(screw_velocities)
        else:
            self.motor_cmd = [Float64() for i in range(4)]
            for m in range(4):
                self.motor_cmd[m].data = self.screw_speeds[m]

        self.publisher_motor0_commands.publish(self.motor_cmd[0])
        self.publisher_motor1_commands.publish(self.motor_cmd[1])
        self.publisher_motor2_commands.publish(self.motor_cmd[2])
        self.publisher_motor3_commands.publish(self.motor_cmd[3])


def main(args=None):
    rclpy.init(args=args)

    open_loop_steering = OpenLoopSteering()

    rclpy.spin(open_loop_steering)

    open_loop_steering.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
