#!/usr/bin/python3
"""
Takes motor module velocities as input from robot software.
Calculates robot forward kinematics to determine body velocity vector.
Publishes cmd_vel.

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 15-12-2021
"""

import rclpy

from rclpy.node import Node
from robominer_msgs.msg import MotorModuleCommand, MotorModuleFeedback
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster


import numpy as np
from math import pi, tan


class RM3ForwardKinematics(Node):
    """Docstring

    more docstring
    """

    def __init__(self):
        super().__init__('rm3_forward_kinematics')

        self.screw_radius = 0.078  # m
        self.screw_helix_angle = pi/6.0 # pi/6 for fl and rr screws, -pi/6 for fr and rl
        self.lx = 0.15
        self.ly = 0.3

        self.screw_speeds = [0.0, 0.0, 0.0, 0.0]
        self.fr_vel = 0.0
        self.rr_vel = 0.0
        self.rl_vel = 0.0
        self.fl_vel = 0.0

        self.cmd_screw_speeds = [0.0, 0.0, 0.0, 0.0]
        self.cmd_fr_vel = 0.0
        self.cmd_rr_vel = 0.0
        self.cmd_rl_vel = 0.0
        self.cmd_fl_vel = 0.0

        self.rpm_to_radpersec = (2*pi)/60.0

        self.fwd_kinematics = 1.0/4.0 * np.array([
            [-tan(self.screw_helix_angle), -tan(self.screw_helix_angle), tan(self.screw_helix_angle), tan(self.screw_helix_angle)],
            [-1.0,  1.0, 1.0,-1.0],
            [-1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly), -1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly), -1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly), -1/(self.lx + 1/tan(self.screw_helix_angle) * self.ly)]
        ])

        self.fb_vel_stamped_pub = self.create_publisher(TwistStamped, '/twist_FKinematics_stamped', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_stamped_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)

        self.publisher_screw_rotation = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        self.kinematics_timer_period = 0.1  # seconds
        self.kinematics_timer = self.create_timer(self.kinematics_timer_period, self.motor_to_body_vel)

        self.br = TransformBroadcaster(self)

        self.create_subscription(MotorModuleFeedback, '/front_right/motor_module', self.front_right, 10)
        self.create_subscription(MotorModuleFeedback, '/rear_right/motor_module', self.rear_right, 10)
        self.create_subscription(MotorModuleFeedback, '/rear_left/motor_module', self.rear_left, 10)
        self.create_subscription(MotorModuleFeedback, '/front_left/motor_module', self.front_left, 10)

        self.create_subscription(MotorModuleCommand, '/motor0/motor_rpm_setpoint', self.motor0, 10)
        self.create_subscription(MotorModuleCommand, '/motor1/motor_rpm_setpoint', self.motor1, 10)
        self.create_subscription(MotorModuleCommand, '/motor2/motor_rpm_setpoint', self.motor2, 10)
        self.create_subscription(MotorModuleCommand, '/motor3/motor_rpm_setpoint', self.motor3, 10)

        self.create_subscription(Odometry, '/odom/unfiltered', self.OdomCallback, 10)

    def front_right(self, msg):
        self.fr_vel = msg.motor_rpm
        # self.get_logger().info(f'front_right: {self.fr_vel}')

    def rear_right(self, msg):
        self.rr_vel = msg.motor_rpm
        # self.get_logger().info(f'rear_right: {self.rr_vel}')

    def rear_left(self, msg):
        self.rl_vel = msg.motor_rpm
        # self.get_logger().info(f'rear_left: {self.rl_vel}')

    def front_left(self, msg):
        self.fl_vel = msg.motor_rpm
        # self.get_logger().info(f'front_left: {self.fl_vel}')

    def motor0(self, msg):
        self.cmd_fr_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'front_right: {self.fr_vel}')

    def motor1(self, msg):
        self.cmd_rr_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'rear_right: {self.rr_vel}')

    def motor2(self, msg):
        self.cmd_rl_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'rear_left: {self.rl_vel}')

    def motor3(self, msg):
        self.cmd_fl_vel = msg.motor_rpm_goal
        # self.get_logger().info(f'front_left: {self.fl_vel}')

    def visualizeScrewsInGazebo(self):
        screw_velocities = Float64MultiArray()
        # A division by a factor was necessary to convert rad/s to whatever is used in velocity controller in gazebo.
        velCorrection = 3766.86341 # this depends on the step size of the solver.
        screw_velocities.data.append(-int(self.fr_vel) * self.rpm_to_radpersec / velCorrection)
        screw_velocities.data.append(int(self.rr_vel) * self.rpm_to_radpersec / velCorrection)
        screw_velocities.data.append(-int(self.rl_vel) * self.rpm_to_radpersec / velCorrection)
        screw_velocities.data.append(int(self.fl_vel) * self.rpm_to_radpersec / velCorrection)
        self.publisher_screw_rotation.publish(screw_velocities)

    def motor_to_body_vel(self):
        # Feedback body twist
        body_vel_stamped = TwistStamped()   # for kalman filter
        self.screw_speeds = np.array([self.fr_vel, self.rr_vel, self.rl_vel, self.fl_vel]) * self.rpm_to_radpersec
        self.robot_twist = self.screw_radius * np.dot(self.fwd_kinematics, self.screw_speeds)
        body_vel_stamped.header.stamp = self.get_clock().now().to_msg()
        body_vel_stamped.twist.linear.x = self.robot_twist[0]
        body_vel_stamped.twist.linear.y = self.robot_twist[1]
        body_vel_stamped.twist.angular.z = self.robot_twist[2]
        self.fb_vel_stamped_pub.publish(body_vel_stamped)
        # Command body twist
        cmd_body_vel = Twist()                  # for object_controller
        cmd_body_vel_stamped = TwistStamped()   # for kalman filter
        self.cmd_screw_speeds = np.array([self.cmd_fr_vel, self.cmd_rr_vel, self.cmd_rl_vel, self.cmd_fl_vel]) * self.rpm_to_radpersec
        self.cmd_robot_twist = self.screw_radius * np.dot(self.fwd_kinematics, self.cmd_screw_speeds)
        cmd_body_vel_stamped.header.stamp = self.get_clock().now().to_msg()
        cmd_body_vel.linear.x = self.cmd_robot_twist[0]
        cmd_body_vel_stamped.twist.linear.x = self.cmd_robot_twist[0]
        cmd_body_vel.linear.y = self.cmd_robot_twist[1]
        cmd_body_vel_stamped.twist.linear.y = self.cmd_robot_twist[1]
        cmd_body_vel.angular.z = self.cmd_robot_twist[2]
        cmd_body_vel_stamped.twist.angular.z = self.cmd_robot_twist[2]
        self.cmd_vel_pub.publish(cmd_body_vel)
        self.cmd_vel_stamped_pub.publish(cmd_body_vel_stamped)

        self.visualizeScrewsInGazebo()

    def OdomCallback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # base_link to world using ground truth odometry
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'gt_initial_pose'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z

        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w

        # Send the transformation
        self.br.sendTransform(t)




def main(args=None):
    rclpy.init(args=args)
    rm3_forward_kinematics = RM3ForwardKinematics()
    rclpy.spin(rm3_forward_kinematics)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rm3_forward_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
