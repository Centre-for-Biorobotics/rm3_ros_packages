#!/usr/bin/python3
"""
Subscribes to robot screw velocities.
Estimates robot's pose and velocity based on a given dynamical model.
Publishes robot's estimated odometry and estimated wrenches.
@author: Walid Remmas
@contact: walid.remmas@taltech.ee
@date: 12-10-2022
"""

import transforms3d

import numpy as np

class RobotDynamics():
    def __init__(self, state_estimation_parameters):
        self.useImu = state_estimation_parameters['robot']['enableIMU']
        self.imuTopic = state_estimation_parameters['robot']['imuTopic']
        # Dynamic model variables
        # ------------------------------------------------
        self.dt = state_estimation_parameters['dynamics']['dt'] # Dynamic model integration period.

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
        self.B = np.array([[-1, -1, 1, 1], \
                           [-1, 1, 1, -1], \
                           [0, 0, 0, 0], \
                           [0, 0, 0, 0], \
                           [0, 0, 0, 0], \
                           [x * -self.lx * np.sin(self.alpha) for x in [1, 1, 1, 1] ]])
        # ---------------------------------------------------------

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

    def computeDynamics(self, screw_velocities, orientation):
        """
        Update the dynamics model of the robot.
        @param: self
        """

        self.updateJacobian(self.eta[3], self.eta[4], self.eta[5])
        # self.get_logger().info(f'YAW model: {self.eta[5]}')
        # ---------------------------------------
        self.tau = np.dot(np.dot(self.sigma, self.B), screw_velocities.transpose())
        self.nu_dot = np.dot(self.M_inv, (self.tau - np.dot(self.drag, self.nu)))

        self.nu += self.dt * self.nu_dot
        self.eta_dot = np.dot(self.J, self.nu)
        self.eta += self.dt * self.eta_dot

        if self.useImu:
            self.eta[5] = orientation[2]
