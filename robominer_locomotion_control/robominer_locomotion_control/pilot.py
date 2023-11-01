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

from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from robominer_msgs.msg import TrajectoryPoint
import transforms3d


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
        controller_type = config_params["Pilot"]["controller_type"]

        self.declare_parameter('in_simulation', False)
        self.in_simulation = self.get_parameter('in_simulation').value

        self.pos = np.zeros(6)

        self.vel = np.zeros(6)

        self.pos_d = np.zeros(6)
        self.vel_d = np.zeros(6)
        self.acc_d = np.zeros(6)

        self.p = np.zeros(6)
        self.p_prev = np.zeros(6)
        self.v = np.zeros(6)

        if controller_type == "PID":
            self.controller = PIDController(config_params)
        elif controller_type == "SMC":
            self.controller = SMController(config_params)

        self.sub_reference = self.create_subscription(
            TrajectoryPoint,'/reference_trajectory', self.onTrajectory, 10)

        self.reference_frame = config_params["Pilot"]["reference_frame"]

        if self.in_simulation:
            imu_topic =  "bno080_imu/data"
        else:
            imu_topic = config_params["Pilot"]["imu_topic"]

        self.cmd_pub_ = self.create_publisher(
            TwistStamped, '/move_cmd_vel', 10)

        self.robot_odom_pub = self.create_publisher(
            TrajectoryPoint, '/robot_odom', 10)

        self.pilot_publish_period = self.dt # seconds
        self.pilot_timer = self.create_timer(self.pilot_publish_period, self.pilot_stepper)

        self.sub_odom = self.create_subscription(
            Odometry, self.reference_frame, self.onOdom, 10)

        if not self.in_simulation:
            self.J = np.zeros((6,6)) # Jacobian matrix of the robot
            self.J_inv = np.zeros((6,6)) # Inverse of Jacobian matrix of the robot
            self.pos_prev = np.zeros(6)
            self.sub_imu = self.create_subscription(Imu, imu_topic, self.onImu, 10)
            self.imu_offset = 0
            self.fixed_offset = False

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
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw) = transforms3d.euler.quat2euler(orientation_list)
        self.pos_d = [msg.pose.position.x, msg.pose.position.y, yaw]
        self.vel_d = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
        self.acc_d = [msg.acceleration.linear.x, msg.acceleration.linear.y, msg.acceleration.angular.z]

    def onImu(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw) = transforms3d.euler.quat2euler(orientation_list)

        if not self.fixed_offset:
            self.imu_offset = yaw
            self.fixed_offset = True

        self.pos[5] = yaw - self.imu_offset
        # update angular_velocity
        self.vel[3] = 0.0
        self.vel[4] = 0.0
        self.vel[5] = msg.angular_velocity.z

    def onOdom(self, msg):
        # Update odom position
        if self.in_simulation:
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
            (roll, pitch, yaw) = transforms3d.euler.quat2euler(orientation_list)
            self.pos[5] = yaw
            self.vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, 0.0, 0.0, 0.0, msg.twist.twist.angular.z]

        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y


    def pilot_stepper(self):
        """
        """
        self.p = np.array([self.pos[0], self.pos[1], 0.0, 0.0, 0.0, self.pos[5]])
        pI = np.array([self.pos_d[0], self.pos_d[1], 0.0, 0.0, 0.0, self.pos_d[2]])

        if not self.in_simulation:
            self.updateJacobian(0.0, 0.0, self.pos[5])
            self.vel[:2] = (self.p[:2] - self.p_prev[:2]) / self.dt
            self.v = np.dot(self.J_inv, np.array([self.vel[0], self.vel[1], 0.0, 0.0, 0.0, self.vel[5]]))

        else:
            self.v = np.array([self.vel[0], self.vel[1], 0.0, 0.0, 0.0, self.vel[5]])

        vI = np.array([self.vel_d[0], self.vel_d[1], 0.0, 0.0, 0.0, self.vel_d[2]])
        aI = np.array([self.acc_d[0], self.acc_d[1], 0.0, 0.0, 0.0, self.acc_d[2]])

        control_output = self.controller.control(self.p, self.v, pI, vI, aI)

        body_vel = np.array([control_output[0], control_output[1], control_output[5]])

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = body_vel[0]
        twist_msg.twist.linear.y = body_vel[1]
        twist_msg.twist.angular.z = body_vel[2]

        self.cmd_pub_.publish(twist_msg)
        self.publishRobotOdom()

        self.p_prev = self.p

    def publishRobotOdom(self):
        odom_msg = TrajectoryPoint()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "base_link"
        odom_msg.pose.position.x = self.p[0]
        odom_msg.pose.position.y = self.p[1]
        odom_msg.pose.position.z = 0.0

        q = transforms3d.euler.euler2quat(0.0, 0.0, self.p[5])
        odom_msg.pose.orientation.w = q[0]
        odom_msg.pose.orientation.x = q[1]
        odom_msg.pose.orientation.y = q[2]
        odom_msg.pose.orientation.z = q[3]
        

        odom_msg.twist.linear.x = self.v[0]
        odom_msg.twist.linear.y = self.v[1]
        odom_msg.twist.linear.z = 0.0
        odom_msg.twist.angular.x = 0.0
        odom_msg.twist.angular.y = 0.0
        odom_msg.twist.angular.z = self.v[5]

        self.robot_odom_pub.publish(odom_msg)

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
