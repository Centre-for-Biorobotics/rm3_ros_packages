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
import transforms3d

from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from robominer_msgs.msg import MotorModuleCommand, MotorModuleFeedback
from std_msgs.msg import Float64, Float64MultiArray


import yaml
import os
from ament_index_python.packages import get_package_share_directory
from robominer_state_estimation.rm3_dynamics import RobotDynamics

import numpy as np

class DynamicsRM3(Node):
    def __init__(self):
        super().__init__('dynamic_state_estimation')

        self.declare_parameter('in_simulation', False )
        self.in_simulation = self.get_parameter('in_simulation').value
        self.rpm_to_radpersec = (2*np.pi)/60.0

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
        # ---------------------------------------------------------
        self.screw_velocities = np.zeros(4) #RM3 has 4 screws.

        if self.robotDynamics.useImu:
            self.create_subscription(Imu, self.robotDynamics.imuTopic, self.imu_callback, 10)
            self.imu_orientation = Quaternion() # to store IMU data

        self.create_subscription(MotorModuleFeedback, '/front_right/motor_module', self.front_right, 10)
        self.create_subscription(MotorModuleFeedback, '/rear_right/motor_module', self.rear_right, 10)
        self.create_subscription(MotorModuleFeedback, '/rear_left/motor_module', self.rear_left, 10)
        self.create_subscription(MotorModuleFeedback, '/front_left/motor_module', self.front_left, 10)

        self.estimated_odom_pub = self.create_publisher(Odometry, '/estimated_odom_FDynamics', 10)
        self.estimated_wrench_pub = self.create_publisher(WrenchStamped, '/estimated_wrench_FDynamics', 10)

        if self.in_simulation:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.cmd_vel_stamped_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
            self.publisher_screw_rotation = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        self.dynamics_estimation_timer = self.create_timer(self.robotDynamics.dt, self.updateDynamics)


    def updateDynamics(self):
        """
        Update the dynamics model of the robot.
        @param: self
        """
        # Decision to Use either estimated orientation or measured orientation
        # to update the Jacobian
        # ----------------------------------------
        if self.robotDynamics.useImu:
            orientation = transforms3d.euler.quat2euler([self.imu_orientation.w, self.imu_orientation.x, self.imu_orientation.y,
                                            self.imu_orientation.z])
            # self.get_logger().info(f'YAW from IMU: {orientation[2]}')
        else:
            orientation = self.robotDynamics.eta[3:6]
        # ----------------------------------------

        # Update dynamics:
        self.robotDynamics.computeDynamics(self.screw_velocities, orientation)

        # Publish dynamic odometry and estimated forces:
        self.publishDynamicResults()


    def publishDynamicResults(self):
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link_est_ukf'
        odom_msg.pose.pose.position.x = self.robotDynamics.eta[0]
        odom_msg.pose.pose.position.y = self.robotDynamics.eta[1]
        odom_msg.pose.pose.position.z = self.robotDynamics.eta[2]
        if self.robotDynamics.useImu:
            odom_msg.pose.pose.orientation = self.imu_orientation
        else:
            q = transforms3d.euler.euler2quat(self.robotDynamics.eta[3], self.robotDynamics.eta[4], self.robotDynamics.eta[5])
            odom_msg.pose.pose.orientation.w = q[0]
            odom_msg.pose.pose.orientation.x = q[1]
            odom_msg.pose.pose.orientation.y = q[2]
            odom_msg.pose.pose.orientation.z = q[3]
            
        odom_msg.twist.twist.linear.x = self.robotDynamics.nu[0]
        odom_msg.twist.twist.linear.y = self.robotDynamics.nu[1]
        odom_msg.twist.twist.linear.z = self.robotDynamics.nu[2]
        odom_msg.twist.twist.angular.x = self.robotDynamics.nu[3]
        odom_msg.twist.twist.angular.y = self.robotDynamics.nu[4]
        odom_msg.twist.twist.angular.z = self.robotDynamics.nu[5]
        self.estimated_odom_pub.publish(odom_msg)

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = 'base_link_est_ukf'
        wrench_msg.wrench.force.x = self.robotDynamics.tau[0]
        wrench_msg.wrench.force.y = self.robotDynamics.tau[1]
        wrench_msg.wrench.force.z = self.robotDynamics.tau[2]
        wrench_msg.wrench.torque.x = self.robotDynamics.tau[3]
        wrench_msg.wrench.torque.y = self.robotDynamics.tau[4]
        wrench_msg.wrench.torque.z = self.robotDynamics.tau[5]
        self.estimated_wrench_pub.publish(wrench_msg)

        if self.in_simulation:
            body_vel = Twist()                  # for object_controller
            body_vel_stamped = TwistStamped()   # for kalman filter
            body_vel_stamped.header.stamp = self.get_clock().now().to_msg()

            body_vel.linear.x = self.robotDynamics.nu[0]
            body_vel_stamped.twist.linear.x = self.robotDynamics.nu[0]

            body_vel.linear.y = self.robotDynamics.nu[1]
            body_vel_stamped.twist.linear.y = self.robotDynamics.nu[1]
            body_vel.angular.z = self.robotDynamics.nu[5]
            body_vel_stamped.twist.angular.z = self.robotDynamics.nu[5]

            self.cmd_vel_pub.publish(body_vel)
            self.cmd_vel_stamped_pub.publish(body_vel_stamped)
            self.visualizeScrewsInGazebo()

    def visualizeScrewsInGazebo(self):
        screw_velocities = Float64MultiArray()
        # A division by a factor was necessary to convert rad/s to whatever is used in velocity controller in gazebo.
        velCorrection = 3766.86341 # this depends on the step size of the solver.
        screw_velocities.data.append(-int(self.screw_velocities[0]) * self.rpm_to_radpersec / velCorrection)
        screw_velocities.data.append(int(self.screw_velocities[1]) * self.rpm_to_radpersec / velCorrection)
        screw_velocities.data.append(-int(self.screw_velocities[2]) * self.rpm_to_radpersec / velCorrection)
        screw_velocities.data.append(int(self.screw_velocities[3]) * self.rpm_to_radpersec / velCorrection)
        self.publisher_screw_rotation.publish(screw_velocities)

    def front_right(self, msg):
        self.screw_velocities[0] = msg.motor_rpm
        # self.get_logger().info(f'front_right: {self.fr_vel}')

    def rear_right(self, msg):
        self.screw_velocities[1] = msg.motor_rpm
        # self.get_logger().info(f'rear_right: {self.rr_vel}')

    def rear_left(self, msg):
        self.screw_velocities[2] = msg.motor_rpm
        # self.get_logger().info(f'rear_left: {self.rl_vel}')

    def front_left(self, msg):
        self.screw_velocities[3] = msg.motor_rpm

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