#!/usr/bin/python3
"""
Takes robot body velocities as input from joystick.
Calculates inverse dynamics (tau = sigma * B * omega) to determine screw velocities.
Publishes screw velocities

@author: Walid Remmas, Roza Gkliva
@contact: walid.remmas@taltech.ee
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

class RM3InverseDynamics(Node):
    def __init__(self):
        super().__init__('rm3_inverse_kinematics')
        self.radpersec_to_rpm = 60.0 / (2*pi)
        self.kinematics_timer_period = 0.1      # seconds
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_yaw = 0.0

        self.sub_body_vel = self.create_subscription(
            TwistStamped, '/robot_body_vel', self.inputCallback, 10
        )

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
        self.dynamics_timer = self.create_timer(self.kinematics_timer_period, self.inverseDynamics)

    def inputCallback(self, msg):
        """
        Callback function for the keyboard topic. Parses a keyboard message to body velocities in x, y, and yaw
        @param: self
        @param: msg - Twist message format
        """
        self.cmd_vel_x = msg.twist.linear.x
        self.cmd_vel_y = msg.twist.linear.y
        self.cmd_vel_yaw = msg.twist.angular.z

    def inverseDynamics(self):
        """
        Solves the inverse dynamics problem to calculate actuator speeds from body velocity.
        Calls to publish the individual motor speeds.
        @param: self
        """

        self.robot_twist = np.array([self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw])

        # Based on inverse dynamics
        allocation_matrix = np.dot(self.robotDynamics.sigma, self.robotDynamics.B)
        allocation_matrix_inv = np.linalg.pinv(allocation_matrix)
        robot_tau_6D0F = [self.robot_twist[0], self.robot_twist[1], 0.0, 0.0, 0.0, self.robot_twist[2]]

        self.screw_speeds = np.dot(allocation_matrix_inv, robot_tau_6D0F) * self.radpersec_to_rpm

        self.speedsBroadcast()

    def speedsBroadcast(self):
        '''
        Publishes the results of inverse kinematics to 4 topics, one RPM setpoint for each screw.
        @param: self
        '''
        # if self.on_robot or self.which_sim=='gazebo':
        self.motor_cmd = [MotorModuleCommand() for i in range(4)]
        for m in range(4):
            self.motor_cmd[m].header.stamp = self.get_clock().now().to_msg()
            self.motor_cmd[m].header.frame_id = motors[m]
            self.motor_cmd[m].motor_id = motors_dict[motors[m]]
            self.motor_cmd[m].motor_rpm_goal = int(self.screw_speeds[m])
        # else:
        #     self.motor_cmd = [Float64() for i in range(4)]
        #     for m in range(4):
        #         self.motor_cmd[m].data = self.screw_speeds[m]

        self.publisher_motor0_commands.publish(self.motor_cmd[0])
        self.publisher_motor1_commands.publish(self.motor_cmd[1])
        self.publisher_motor2_commands.publish(self.motor_cmd[2])
        self.publisher_motor3_commands.publish(self.motor_cmd[3])


def main(args=None):
    rclpy.init(args=args)

    rm3_inverse_dynamics = RM3InverseDynamics()

    rclpy.spin(rm3_inverse_dynamics)

    rm3_inverse_dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
