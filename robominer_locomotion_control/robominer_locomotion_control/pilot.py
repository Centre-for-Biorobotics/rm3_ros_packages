#!/usr/bin/python3
"""
Pilot node:
Node that handles control and regulation of body-velocities for trajectory tracking.

@author: Walid Remmas
@contact: walid.remmas@taltech.ee
@date: 28-11-22

"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

from robominer_msgs.msg import TrajectoryPoint
import tf_transformations


import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

class Pilot(Node):
    def __init__(self, config_params):
        super().__init__('Pilot')

        self.pilot_parameters = config_params

        self.dt = config_params["Pilot"]["dt"]
        self.J = np.zeros((6,6))
        self.J_inv = np.zeros((6,6))

        self.pos = np.zeros(6)
        self.vel = np.zeros(6)

        self.pos_d = np.zeros(6)
        self.vel_d = np.zeros(6)

        # Controller parameters
        # -----------------------------------------------------------------
        Kp = config_params["Control"]["PID"]["Kp"]
        Kd = config_params["Control"]["PID"]["Kd"]
        Ki = config_params["Control"]["PID"]["Ki"]

        windup = config_params["Control"]["PID"]["windup"]

        self.Kp = np.diag([Kp[0], Kp[1], 0.0, 0.0, 0.0, Kp[2]])
        self.Kd = np.diag([Kd[0], Kd[1], 0.0, 0.0, 0.0, Kd[2]])
        self.Ki = np.diag([Ki[0], Ki[1], 0.0, 0.0, 0.0, Ki[2]])
        self.windup = np.array([windup[0], windup[1], 0.0, 0.0, 0.0, windup[2]])
        self.saturation = np.array(config_params["Control"]["PID"]["saturation"])
        # -----------------------------------------------------------------

        self.int_position_error = np.zeros(6)

        self.sub_reference = self.create_subscription(
            TrajectoryPoint,'/reference_trajectory', self.onTrajectory, 10)

        reference_frame = config_params["Pilot"]["reference_frame"]
        self.sub_odom = self.create_subscription(
            Odometry, reference_frame, self.onOdom, 10)

        self.cmd_pub_ = self.create_publisher(
            TwistStamped, '/move_cmd_vel', 10)

        self.pilot_publish_period = self.dt # seconds
        self.pilot_timer = self.create_timer(self.pilot_publish_period, self.pilot_stepper)

    def onTrajectory(self, msg):
        # update desired trajectory
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.pos_d = [msg.pose.position.x, msg.pose.position.y, yaw]
        self.vel_d = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]


    def onOdom(self, msg):
        # Update odom position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        self.vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]


    def control(self, p, v, pI, vI):
        # Control function (for PID)
        e1 = np.dot(self.J_inv, (pI -p))
        e2 = np.dot(self.J_inv, vI) - v

        while e1[5] > np.pi:
            e1[5] -= 2.0 * np.pi
        while e1[5] < -np.pi:
            e1[5] += 2.0 * np.pi

        self.int_position_error += e1

        for i in range(0,6):
            if self.int_position_error[i] < -self.windup[i]:
                self.int_position_error[i] = -self.windup[i]
            elif self.int_position_error[i] > self.windup[i]:
                self.int_position_error[i] = self.windup[i]

        PID = np.dot(self.Kp , e1) + np.dot(self.Kd, e2) + np.dot(self.Ki, self.int_position_error)
        return PID


    def pilot_stepper(self):
        """
        """
        p = np.array([self.pos[0], self.pos[1], 0.0, 0.0, 0.0, self.pos[2]])
        pI = np.array([self.pos_d[0], self.pos_d[1], 0.0, 0.0, 0.0, self.pos_d[2]])

        v = np.array([self.vel[0], self.vel[1], 0.0, 0.0, 0.0, self.vel[2]])
        vI = np.array([self.vel_d[0], self.vel_d[1], 0.0, 0.0, 0.0, self.vel_d[2]])

        self.updateJacobian(0.0, 0.0, p[5])
        control_output = self.control(p, v, pI, vI)

        # saturation of control output
        body_vel = np.array([control_output[0], control_output[1], control_output[5]])
        for i in range(3):
            if body_vel[i] < -self.saturation[i]:
                body_vel[i] = -self.saturation[i]
            elif body_vel[i] > self.saturation[i]:
                body_vel[i] = self.saturation[i]

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = body_vel[0]
        twist_msg.twist.linear.y = body_vel[1]
        twist_msg.twist.angular.z = body_vel[2]

        self.cmd_pub_.publish(twist_msg)

    def updateJacobian(self, phi, theta, psi):
        """
        Function to update the Jacobian matrix.
        """
        xrot = np.array([[1.0, 0.0, 0.0],
                         [0.0, np.cos(phi), -np.sin(phi)],
                         [0.0, np.sin(phi), np.cos(phi)]])

        yrot = np.array([[np.cos(theta), 0.0, np.sin(theta)],
                         [0.0, 1.0, 0.0],
                         [-np.sin(theta), 0.0, np.cos(theta)]])

        zrot = np.array([[np.cos(psi), -np.sin(psi), 0.0],
                         [np.sin(psi), np.cos(psi), 0.0],
                         [0.0, 0.0, 1.0]])

        ROT = np.dot(np.dot(zrot, yrot), xrot)
        T = np.array([
                [1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                [0.0, np.cos(phi), -np.sin(phi)],
                [0.0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]])

        self.J[0:3, 0:3] = ROT
        self.J[3:6, 3:6] = T

        T_inv = np.linalg.inv(T)
        ROT_inv = np.linalg.inv(ROT)

        self.J_inv[0:3, 0:3] = ROT_inv
        self.J_inv[3:6, 3:6] = T_inv


def main(args=None):
    parameters_from_yaml = os.path.join(
            get_package_share_directory('robominer_locomotion_control'),
            'config',
            'pilot_parameters.yaml'
            )

    # Load parameters from YAML file
    with open(parameters_from_yaml, 'r') as file:
        pilot_parameters = yaml.load(file, Loader=yaml.FullLoader)

    rclpy.init(args=args)
    pilot = Pilot(pilot_parameters)
    rclpy.spin(pilot)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pilot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
