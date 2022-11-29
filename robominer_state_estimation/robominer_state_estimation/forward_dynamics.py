#!/usr/bin/python3
"""
Subscribes to robot screw velocities.
Estimates robot's pose and velocity based on a given dynamical model.
Publishes robot's estimated odometry and estimated wrenches.

@author: Walid Remmas
@contact: walid.remmas@taltech.ee
@date: 12-10-2022
"""

import rclpy
from rclpy.parameter import Parameter
import tf_transformations

from rclpy.node import Node

from geometry_msgs.msg import Twist, WrenchStamped, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from robominer_msgs.msg import MotorModuleCommand

import yaml
import os
from ament_index_python.packages import get_package_share_directory


import numpy as np

class DynamicsRM3(Node):
    def __init__(self):
        super().__init__('dynamic_state_estimation')

        parameters_from_yaml = os.path.join(
                get_package_share_directory('robominer_state_estimation'),
                'config',
                'state_estimation_parameters.yaml'
                )

        # Load parameters from YAML file
        with open(parameters_from_yaml, 'r') as file:
            state_estimation_parameters = yaml.load(file, Loader=yaml.FullLoader)

        self.useImu = state_estimation_parameters['robot']['enableIMU']
        self.imu_orientation = Quaternion()
        if self.useImu:
            self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
            # self.create_subscription(Imu, '/bno080_imu', self.imu_callback, 10)

        # Dynamic model variables
        # ------------------------------------------------
        self.dt = state_estimation_parameters['dynamics']['dt'] # Dynamic model integration period.
        self.rpm_to_radpersec = (2*np.pi)/60.0

        self.eta = np.zeros(6) # variable to store the pose of the robot in the world frame
        self.eta_dot = np.zeros(6) # variable to store the velocity of the robot in the world frame

        self.nu = np.zeros(6) # variable to store the velocity of the robot in the robot frame
        self.nu_dot = np.zeros(6) # variable to store the acceleration of the robot in the robot frame

        self.tau = np.zeros(6) # variable to store the wrenches of the robot
        self.J = np.zeros((6,6)) # Jacobian matrix of the robot
        self.J_inv = np.zeros((6,6)) # Inverse of Jacobian matrix of the robot
        self.drag = np.zeros((6,6)) # Friction matrix
        self.M = np.zeros((6,6)) # Inertial matrix
        # ------------------------------------------------

        # RM3 Dynamic parameters :
        # ---------------------------------------------------------
        self.lx = state_estimation_parameters['dynamics']['dimensions']['lx']  # m longitudinal distance
        self.ly = state_estimation_parameters['dynamics']['dimensions']['ly']  # m lateral distance
        self.alpha = np.deg2rad(state_estimation_parameters['dynamics']['alpha'])
        sigma_long = state_estimation_parameters['dynamics']['sigma_long'] # longitudinal screw coeff: Computed using linear regression based on experimental data
        sigma_lat = state_estimation_parameters['dynamics']['sigma_lat'] # lateral screw coeff: Computed using linear regression based on experimental data
        self.sigma = np.diag([sigma_long, sigma_lat, 0.0 ,0.0 ,0.0, sigma_lat]) # linear transform of screw_velocity to force parameter
        self.drag = np.diag(state_estimation_parameters['dynamics']['drag'])

        # The inertia of the robot is approximated using a 3D box model.
        robot_mass = state_estimation_parameters['dynamics']['mass'] #Kg
        robot_height = state_estimation_parameters['dynamics']['dimensions']['z'] #m
        robot_width = state_estimation_parameters['dynamics']['dimensions']['y'] #m
        robot_lenght = state_estimation_parameters['dynamics']['dimensions']['x'] #m
        Ix = (robot_mass / 12.0) * (robot_height**2 + robot_width**2)
        Iy = (robot_mass / 12.0) * (robot_height**2 + robot_lenght**2)
        Iz = (robot_mass / 12.0) * (robot_lenght**2 + robot_width**2)

        # Inertial matrix
        self.M = np.diag([robot_mass, robot_mass, robot_mass, Ix, Iy, Iz])
        self.M_inv = np.linalg.inv(self.M) # inverse of inertial matrix computed only once.

        # Force Allocation Model for RM3
        self.screw_velocities = np.zeros(4) #RM3 has 4 screws.
        self.B = np.array([[-1, -1, 1, 1], \
                           [-1, 1, 1, -1], \
                           [0, 0, 0, 0], \
                           [0, 0, 0, 0], \
                           [0, 0, 0, 0], \
                           [x * -self.lx * np.sin(self.alpha) for x in [1, 1, 1, 1] ]])
        # ---------------------------------------------------------

        self.create_subscription(MotorModuleCommand, '/motor0/motor_rpm_setpoint', self.front_right, 10)
        self.create_subscription(MotorModuleCommand, '/motor1/motor_rpm_setpoint', self.rear_right, 10)
        self.create_subscription(MotorModuleCommand, '/motor2/motor_rpm_setpoint', self.rear_left, 10)
        self.create_subscription(MotorModuleCommand, '/motor3/motor_rpm_setpoint', self.front_left, 10)

        self.estimated_odom_pub = self.create_publisher(Odometry, '/estimated_odom', 10)
        self.estimated_wrench_pub = self.create_publisher(WrenchStamped, '/estimated_wrench', 10)

        self.dynamics_estimation_timer = self.create_timer(self.dt, self.computeDynamics)


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

    def computeDynamics(self):
        """
        Update the dynamics model of the robot.
        @param: self
        """
        # Update jacobian matrix based on estimated roll, pitch and yaw
        self.updateJacobian(self.eta[3], self.eta[4], self.eta[5])
        # ---------------------------------------
        # Update jacobian matrix based on measured roll, pitch and yaw
        ## TODO: updateJacobian(self.eta[3], self.eta[6], self.eta[5])
        # ---------------------------------------
        self.tau = np.dot(self.sigma, np.dot(self.B, self.rpm_to_radpersec * self.screw_velocities.transpose() ))
        self.nu_dot = np.dot(self.M_inv, (self.tau - np.dot(self.drag, self.nu)))

        self.nu += self.dt * self.nu_dot
        self.eta_dot = np.dot(self.J, self.nu)
        self.eta += self.dt * self.eta_dot

        if self.useImu:
            imu_angles = tf_transformations.euler_from_quaternion([self.imu_orientation.x, self.imu_orientation.y,
                                            self.imu_orientation.z, self.imu_orientation.w])
            self.eta[3:6] = imu_angles

        self.publishDynamicResults()


    def publishDynamicResults(self):
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'dynamics'
        odom_msg.pose.pose.position.x = self.eta[0]
        odom_msg.pose.pose.position.y = self.eta[1]
        odom_msg.pose.pose.position.z = self.eta[2]
        if self.useImu:
            odom_msg.pose.pose.orientation = self.imu_orientation
        else:
            q = tf_transformations.quaternion_from_euler(self.eta[3], self.eta[4], self.eta[5])
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = self.nu[0]
        odom_msg.twist.twist.linear.y = self.nu[1]
        odom_msg.twist.twist.linear.z = self.nu[2]
        odom_msg.twist.twist.angular.x = self.nu[3]
        odom_msg.twist.twist.angular.y = self.nu[4]
        odom_msg.twist.twist.angular.z = self.nu[5]
        self.estimated_odom_pub.publish(odom_msg)

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = 'base_link'
        wrench_msg.wrench.force.x = self.tau[0]
        wrench_msg.wrench.force.y = self.tau[1]
        wrench_msg.wrench.force.z = self.tau[2]
        wrench_msg.wrench.torque.x = self.tau[3]
        wrench_msg.wrench.torque.y = self.tau[4]
        wrench_msg.wrench.torque.z = self.tau[5]
        self.estimated_wrench_pub.publish(wrench_msg)

    def front_right(self, msg):
        self.screw_velocities[0] = msg.motor_rpm_goal
        # self.get_logger().info(f'front_right: {self.fr_vel}')

    def rear_right(self, msg):
        self.screw_velocities[1] = msg.motor_rpm_goal
        # self.get_logger().info(f'rear_right: {self.rr_vel}')

    def rear_left(self, msg):
        self.screw_velocities[2] = msg.motor_rpm_goal
        # self.get_logger().info(f'rear_left: {self.rl_vel}')

    def front_left(self, msg):
        self.screw_velocities[3] = msg.motor_rpm_goal

    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation

def main(args=None):
    rclpy.init(args=args)

    forward_dynamics = DynamicsRM3()

    rclpy.spin(forward_dynamics)

    forward_dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
