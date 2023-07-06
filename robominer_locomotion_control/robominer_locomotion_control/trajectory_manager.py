#!/usr/bin/python3
"""
Trajectory generation and management:
This node gets predefined parameters from a config file and generates a desired trajectory

@author: Walid Remmas
@contact: walid.remmas@taltech.ee
@date: 10-11-22

"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from robominer_msgs.msg import TrajectoryPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf_transformations


import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

class TrajectoryManager(Node):
    def __init__(self, config_params):
        super().__init__('Trajectory_Manager')

        self.trajectory_parameters = config_params

        # Trajectory parameters
        self.dt = config_params["Trajectory"]["dt"]
        self.traj_type = config_params["Trajectory"]["type"]
        self.gamma = config_params["Trajectory"]["gamma"]

        if self.traj_type == "Elliptic":
            self.cosFreq = config_params["Elliptic"]["cosFreq"]
            self.sinFreq = config_params["Elliptic"]["sinFreq"]
            self.cosAmp = config_params["Elliptic"]["cosAmp"]
            self.sinAmp = config_params["Elliptic"]["sinAmp"]
            self.enable_LoS = config_params["Elliptic"]["line_of_sight"]

        if self.traj_type == "Lissajous":
            self.liss_a = config_params["Lissajous"]["a"]
            self.liss_b = config_params["Lissajous"]["b"]
            self.cosFreq = config_params["Lissajous"]["cosFreq"]
            self.sinFreq = config_params["Lissajous"]["sinFreq"]
            self.cosAmp = config_params["Lissajous"]["cosAmp"]
            self.sinAmp = config_params["Lissajous"]["sinAmp"]
            self.enable_LoS = config_params["Lissajous"]["line_of_sight"]

        if self.traj_type == "Waypoints":
            self.waypoint_time = config_params["Waypoints"]["waypoint_time"]
            self.waypoint_x = config_params["Waypoints"]["waypoint_x"]
            self.waypoint_y = config_params["Waypoints"]["waypoint_y"]
            self.waypoint_yaw = config_params["Waypoints"]["waypoint_yaw"]
            self.enable_LoS = config_params["Waypoints"]["line_of_sight"]
            self.enable_repeat = config_params["Waypoints"]["repeat"]
            self.waypoints_size = len(self.waypoint_x)
            self.cumulative_timer = self.waypoint_time[0]
            self.waypoint_number = 0

        # ODE storing variables:
        initial_pose = config_params["Trajectory"]["initial_pose"]
        self.xTraj = initial_pose[0]; self.xdTraj = 0.0; self.xddTraj = 0.0
        self.yTraj = initial_pose[1]; self.ydTraj = 0.0; self.yddTraj = 0.0
        self.yawTraj = initial_pose[5]; self.yaw_dTraj = 0.0; self.yaw_ddTraj = 0.0
        self.traj_pos = np.zeros(3)
        self.traj_vel = np.zeros(3)
        self.traj_acc = np.zeros(3)
        self.yaw_prev = 0.0

        self.traj_x_pos = initial_pose[0]
        self.traj_y_pos = initial_pose[1]
        self.traj_yaw_pos = initial_pose[5]
        self.traj_x_pos_next = 0
        self.traj_y_pos_next = 0

        # ROS2 related
        self.reference_trajectory_pub_ = self.create_publisher(
            TrajectoryPoint, '/reference_trajectory', 10)

        self.ref_yaw_pub_ = self.create_publisher(
            Float64, '/reference_yaw', 10)

        self.sub_pose_from_topic = self.create_subscription(
            PoseStamped, '/goal_pose', self.updatePose, 10)

        self.trajectory_publish_period = self.dt # seconds
        self.trajectory_time = 0.0
        self.trajectory_manager_timer = self.create_timer(
            self.trajectory_publish_period, self.trajectory_stepper)

    def trajectory_stepper(self):
        # Get desired trajectory pose and velocity based on traj type and ODE filters
        # ---------------------------------------------------------------------

        if self.traj_type == "Elliptic":
            self.traj_x_pos = self.cosAmp * (-np.cos(self.trajectory_time * self.cosFreq) + 1)
            self.traj_y_pos = self.sinAmp * np.sin(self.trajectory_time * self.sinFreq)

            if self.enable_LoS:
                self.traj_x_pos_next = self.cosAmp * (-np.cos((self.trajectory_time + 0.1) * self.cosFreq) + 1)
                self.traj_y_pos_next = self.sinAmp * np.sin((self.trajectory_time + 0.1) * self.sinFreq)
                x = self.traj_x_pos_next - self.traj_x_pos
                y = self.traj_y_pos_next - self.traj_y_pos
                self.traj_yaw_pos = np.arctan2(y, x)
            else:
                self.traj_yaw_pos = 0.0 # or something...
        # ---------------------------------------------------------------------

        # ---------------------------------------------------------------------
        elif self.traj_type == "Lissajous":
            self.traj_x_pos = self.cosAmp * (-np.cos(self.liss_a * self.trajectory_time * self.cosFreq) + 1)
            self.traj_y_pos = self.sinAmp * np.sin(self.liss_b * self.trajectory_time * self.sinFreq)

            if self.enable_LoS:
                self.traj_x_pos_next = self.cosAmp * (-np.cos(self.liss_a * (self.trajectory_time + 0.1) * self.cosFreq) + 1)
                self.traj_y_pos_next = self.sinAmp * np.sin(self.liss_b * (self.trajectory_time + 0.1) * self.sinFreq)
                x = self.traj_x_pos_next - self.traj_x_pos
                y = self.traj_y_pos_next - self.traj_y_pos
                self.traj_yaw_pos = np.arctan2(y, x)
            else:
                self.traj_yaw_pos = 0.0 # or something...
        # ---------------------------------------------------------------------

        # ---------------------------------------------------------------------
        elif self.traj_type == "Waypoints":
            self.traj_x_pos = self.waypoint_x[self.waypoint_number]
            self.traj_y_pos = self.waypoint_y[self.waypoint_number]
            self.traj_yaw_pos = np.deg2rad(self.waypoint_yaw[self.waypoint_number])

            if self.trajectory_time >= self.cumulative_timer:
                if self.waypoint_number < (self.waypoints_size-1):
                    self.cumulative_timer += self.waypoint_time[self.waypoint_number]
                    self.waypoint_number += 1
                elif self.enable_repeat:
                    self.cumulative_timer = self.waypoint_time[0]
                    self.trajectory_time = 0.0
                    self.waypoint_number = 0
        # ---------------------------------------------------------------------

        self.filterTrajectoryFromODE([self.traj_x_pos, self.traj_y_pos, self.traj_yaw_pos])
        self.publishTrajectory()

        self.trajectory_time += self.dt



    def filterTrajectoryFromODE(self, reference):
        """
        Update position, velocity, and acceleration trajectory using a 2nd order
        ODE
        @param: self
        @param: pI - (6x1) desired pose vector
        @result: updates the pos vel and acc trajcetories.
        """

        self.xTraj += self.dt * self.xdTraj
        self.xdTraj += self.dt * self.xddTraj
        self.xddTraj = (self.gamma[0]**2 * (reference[0] - self.xTraj) - 2.0 * self.gamma[0] * self.xdTraj)

        self.yTraj += self.dt * self.ydTraj
        self.ydTraj += self.dt * self.yddTraj
        self.yddTraj =  (self.gamma[1]**2 * (reference[1] - self.yTraj) - 2.0 * self.gamma[1] * self.ydTraj)

        self.yawTraj += self.dt * self.yaw_dTraj
        traj_yaw_pos = np.unwrap([self.yaw_prev, reference[2]])[1]
        self.yaw_dTraj += self.dt * self.yaw_ddTraj
        self.yaw_ddTraj = self.gamma[2]**2 * (reference[2] - self.yawTraj) - 2.0 * self.gamma[2] * self.yaw_dTraj

        self.yaw_prev = traj_yaw_pos

        self.traj_pos = np.array([self.xTraj, self.yTraj, self.yawTraj])
        self.traj_vel = np.array([self.xdTraj, self.ydTraj, self.yaw_dTraj])
        self.traj_acc = np.array([self.xddTraj, self.yddTraj, self.yaw_ddTraj])

    def publishTrajectory(self):
        """
        Helper function to publish the desired trajectory
        @param: self
        @result: publishes trajectory as robominer_msgs/TrajectoryPoint message.
        """
        traj_msg = TrajectoryPoint()
        ref_yaw_msg = Float64()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "base_link"
        traj_msg.pose.position.x = self.traj_pos[0]
        traj_msg.pose.position.y = self.traj_pos[1]
        traj_msg.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.traj_pos[2])
        traj_msg.pose.orientation.x = q[0]
        traj_msg.pose.orientation.y = q[1]
        traj_msg.pose.orientation.z = q[2]
        traj_msg.pose.orientation.w = q[3]

        traj_msg.twist.linear.x = self.traj_vel[0]
        traj_msg.twist.linear.y = self.traj_vel[1]
        traj_msg.twist.linear.z = 0.0
        traj_msg.twist.angular.x = 0.0
        traj_msg.twist.angular.y = 0.0
        traj_msg.twist.angular.z = self.traj_vel[2]

        traj_msg.acceleration.linear.x = self.traj_acc[0]
        traj_msg.acceleration.linear.y = self.traj_acc[1]
        traj_msg.acceleration.linear.z = 0.0
        traj_msg.acceleration.angular.x = 0.0
        traj_msg.acceleration.angular.y = 0.0
        traj_msg.acceleration.angular.z = self.traj_acc[2]

        self.reference_trajectory_pub_.publish(traj_msg)

        ref_yaw_msg.data = np.rad2deg(self.traj_pos[2])
        self.ref_yaw_pub_.publish(ref_yaw_msg)

    def updatePose(self, msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.traj_x_pos = msg.pose.position.x
        self.traj_y_pos = msg.pose.position.y
        self.traj_yaw_pos = yaw

def main(args=None):
    parameters_from_yaml = os.path.join(
            get_package_share_directory('robominer_locomotion_control'),
            'config',
            'trajectory_parameters.yaml'
            )

    # Load parameters from YAML file
    with open(parameters_from_yaml, 'r') as file:
        trajectory_parameters = yaml.load(file, Loader=yaml.FullLoader)

    rclpy.init(args=args)
    trajectory_manager = TrajectoryManager(trajectory_parameters)
    rclpy.spin(trajectory_manager)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
