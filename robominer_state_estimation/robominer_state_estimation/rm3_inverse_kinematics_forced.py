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
from std_msgs.msg import Float64
from robominer_state_estimation.rm3_dynamics import RobotDynamics

import yaml
import os
from ament_index_python.packages import get_package_share_directory

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

class RM3InverseKinematicsForced(Node):
    def __init__(self):
        super().__init__('rm3_inverse_kinematics')
        self.lx = 0.15                          # m longitudinal distance
        self.ly = 0.3                           # m lateral distance
        self.screw_helix_angle = pi/6.0         # pi/6 for fl and rr screws, -pi/6 for fr and rl
        self.screw_radius = 0.078 	            # m
        self.radpersec_to_rpm = 60.0 / (2*pi)
        self.rpm_to_radpersec = (2*pi) / 60.0
        self.kinematics_timer_period = 0.1      # seconds
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_yaw = 0.0
        self.speed_multiplier = 0.0             # speed multiplier that is applied to the body velocity input
        self.turbo_multiplier = 0.0

        self.sub_body_vel = self.create_subscription(
            TwistStamped, '/robot_body_vel', self.inputCallback, 10
        )


        self.platform_kinematics = np.array([
            [-1/tan(self.screw_helix_angle), -1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [-1/tan(self.screw_helix_angle),  1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [1/tan(self.screw_helix_angle),   1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [1/tan(self.screw_helix_angle),  -1, -(self.lx + self.ly / tan(self.screw_helix_angle))]])

        self.speed_multiplier = 1.0

        self.publisher_motor0_commands = self.create_publisher(MotorModuleCommand, '/motor0/motor_rpm_setpoint', 10)
        self.publisher_motor1_commands = self.create_publisher(MotorModuleCommand, '/motor1/motor_rpm_setpoint', 10)
        self.publisher_motor2_commands = self.create_publisher(MotorModuleCommand, '/motor2/motor_rpm_setpoint', 10)
        self.publisher_motor3_commands = self.create_publisher(MotorModuleCommand, '/motor3/motor_rpm_setpoint', 10)


        parameters_from_yaml = os.path.join(
                get_package_share_directory('robominer_state_estimation'),
                'config',
                'state_estimation_parameters.yaml'
                )

        # Load parameters from YAML file
        # ---------------------------------------------------------
        with open(parameters_from_yaml, 'r') as file:
            state_estimation_parameters = yaml.load(file, Loader=yaml.FullLoader)
        self.robotDynamics = RobotDynamics(state_estimation_parameters)
        self.kinematics_timer = self.create_timer(self.kinematics_timer_period, self.inverseKinematics)

    def inputCallback(self, msg):
        """
        Callback function for the keyboard topic. Parses a keyboard message to body velocities in x, y, and yaw
        @param: self
        @param: msg - Twist message format
        """
        self.cmd_vel_x = msg.twist.linear.x
        self.cmd_vel_y = msg.twist.linear.y
        self.cmd_vel_yaw = msg.twist.angular.z

    def inverseKinematics(self):
        """
        Solves the inverse kinematic problem to calculate actuator speeds from body velocity.
        Calls to publish the individual motor speeds.
        @param: self
        """

        self.robot_twist = np.array([self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw])
        self.screw_speeds = (1.0/self.screw_radius) * np.dot(self.platform_kinematics, self.robot_twist) * self.radpersec_to_rpm

        RPM1 = 5
        RPM2 = 10
        RPM3 = 15

        single_dof_velocity = np.max(np.abs(self.robot_twist))

        for m in range(4):
            if single_dof_velocity <= 0.0:
                self.screw_speeds[m] = 0
            elif single_dof_velocity < 1.0:
                if self.screw_speeds[m] > 0.0:
                    self.screw_speeds[m] = RPM1
                else:
                    self.screw_speeds[m] = -RPM1
            elif single_dof_velocity < 2.0:
                if self.screw_speeds[m] > 0.0:
                    self.screw_speeds[m] = RPM2
                else:
                    self.screw_speeds[m] = -RPM2
            else:
                if self.screw_speeds[m] > 0.0:
                    self.screw_speeds[m] = RPM3
                else:
                    self.screw_speeds[m] = -RPM3

        self.speedsBroadcast()

    def speedsBroadcast(self):
        '''
        Publishes the results of inverse kinematics to 4 topics, one RPM setpoint for each screw.
        @param: self
        '''
        self.motor_cmd = [MotorModuleCommand() for i in range(4)]
        for m in range(4):
            self.motor_cmd[m].header.stamp = self.get_clock().now().to_msg()
            self.motor_cmd[m].header.frame_id = motors[m]
            self.motor_cmd[m].motor_id = motors_dict[motors[m]]
            self.motor_cmd[m].motor_rpm_goal = int(self.screw_speeds[m])

        self.publisher_motor0_commands.publish(self.motor_cmd[0])
        self.publisher_motor1_commands.publish(self.motor_cmd[1])
        self.publisher_motor2_commands.publish(self.motor_cmd[2])
        self.publisher_motor3_commands.publish(self.motor_cmd[3])


def main(args=None):
    rclpy.init(args=args)

    rm3_inverse_kinematics = RM3InverseKinematicsForced()

    rclpy.spin(rm3_inverse_kinematics)

    rm3_inverse_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
