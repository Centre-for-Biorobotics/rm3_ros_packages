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

from robominer_locomotion_control.controllers import PIDController, SMController

class Pilot(Node):
    def __init__(self, config_params):
        super().__init__('Pilot')
        self.pilot_parameters = config_params
        self.dt = config_params["Pilot"]["dt"]
        type = config_params["Pilot"]["controller_type"]


        self.pos = np.zeros(6)
        self.vel = np.zeros(6)

        self.pos_d = np.zeros(6)
        self.vel_d = np.zeros(6)
        self.acc_d = np.zeros(6)

        if type == "PID":
            self.controller = PIDController(config_params)
        elif type == "SMC":
            self.controller = SMController(config_params)

        self.sub_reference = self.create_subscription(
            TrajectoryPoint,'/reference_trajectory', self.onTrajectory, 10)

        reference_frame = config_params["Pilot"]["reference_frame"]
        self.sub_odom = self.create_subscription(
            Odometry, reference_frame, self.onOdom, 10)

        self.cmd_pub_ = self.create_publisher(
            TwistStamped, '/move_cmd_vel', 10)

        self.pilot_publish_period = self.dt # seconds
        self.pilot_timer = self.create_timer(self.pilot_publish_period, self.pilot_stepper)

    def destroy_node(self):
        self.get_logger().info('Sending zero commands before Pilot node destruction')

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.cmd_pub_.publish(twist_msg)

        self.get_logger().info('--------------------')
        self.get_logger().info('Pilot Node destroyed')
        self.get_logger().info('--------------------')
        super().destroy_node()

    def onTrajectory(self, msg):
        # update desired trajectory
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.pos_d = [msg.pose.position.x, msg.pose.position.y, yaw]
        self.vel_d = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
        self.acc_d = [msg.acceleration.linear.x, msg.acceleration.linear.y, msg.acceleration.angular.z]


    def onOdom(self, msg):
        # Update odom position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        self.vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]


    def pilot_stepper(self):
        """
        """
        p = np.array([self.pos[0], self.pos[1], 0.0, 0.0, 0.0, self.pos[2]])
        pI = np.array([self.pos_d[0], self.pos_d[1], 0.0, 0.0, 0.0, self.pos_d[2]])

        v = np.array([self.vel[0], self.vel[1], 0.0, 0.0, 0.0, self.vel[2]])
        vI = np.array([self.vel_d[0], self.vel_d[1], 0.0, 0.0, 0.0, self.vel_d[2]])
        aI = np.array([self.acc_d[0], self.acc_d[1], 0.0, 0.0, 0.0, self.acc_d[2]])

        control_output = self.controller.control(p, v, pI, vI, aI)

        body_vel = np.array([control_output[0], control_output[1], control_output[5]])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = body_vel[0]
        twist_msg.twist.linear.y = body_vel[1]
        twist_msg.twist.angular.z = body_vel[2]

        self.cmd_pub_.publish(twist_msg)


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
