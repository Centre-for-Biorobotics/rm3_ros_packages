#!/usr/bin/python3
"""
Controllers classes:
Definition of controllers that can be used for autonomous / semi-autonomous control.

@author: Walid Remmas
@contact: walid.remmas@taltech.ee
@date: 13-02-22

"""

import rclpy
from rclpy.node import Node

import tf_transformations

import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

from robominer_state_estimation.rm3_dynamics import RobotDynamics


class PIDController():
    def __init__(self, config_params):
        """
        """
        # Make this conditional based on model free or model-based PID
        # ---------------------------------------------------------
        # Get robot model for
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
        self.dt = config_params["Pilot"]["dt"]

        # Controller parameters
        # -----------------------------------------------------------------
        Kp = config_params["Control"]["PID"]["Kp"]
        Kd = config_params["Control"]["PID"]["Kd"]
        Ki = config_params["Control"]["PID"]["Ki"]
        saturation = config_params["Control"]["PID"]["saturation"]
        windup = config_params["Control"]["PID"]["windup"]

        self.Kp = np.diag([Kp[0], Kp[1], 0.0, 0.0, 0.0, Kp[2]])
        self.Kd = np.diag([Kd[0], Kd[1], 0.0, 0.0, 0.0, Kd[2]])
        self.Ki = np.diag([Ki[0], Ki[1], 0.0, 0.0, 0.0, Ki[2]])
        self.windup = np.array([windup[0], windup[1], 0.0, 0.0, 0.0, windup[2]])
        self.saturation = np.array([saturation[0], saturation[1], 0.0, 0.0, 0.0, saturation[2]])
        # -----------------------------------------------------------------
        self.int_position_error = np.zeros(6)

    def control(self, p, v, pI, vI, aI = np.zeros(6)):
        """
        """
        self.robotDynamics.updateJacobian(0.0, 0.0, p[5])

        # Control function (for PID)
        e1 = np.dot(self.robotDynamics.J_inv, (pI -p))
        e2 = np.dot(self.robotDynamics.J_inv, vI) - v

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

        # saturation of control output
        for i in range(6):
            if PID[i] < -self.saturation[i]:
                PID[i] = -self.saturation[i]
            elif PID[i] > self.saturation[i]:
                PID[i] = self.saturation[i]

        return PID


class SMController:
    def __init__(self, config_params):
        """
        """

        # Make this conditional based on model free or model-based PID
        # ---------------------------------------------------------
        # Get robot model for
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
        self.dt = config_params["Pilot"]["dt"]

        K = config_params["Control"]["SMC"]["K"]
        alpha = config_params["Control"]["SMC"]["alpha"]
        A = config_params["Control"]["SMC"]["A"]
        saturation = config_params["Control"]["SMC"]["saturation"]

        self.modelBased = config_params["Control"]["SMC"]["useModel"]

        self.K = np.array([K[0], K[1], 0.0, 0.0, 0.0, K[2]])
        self.alpha = np.array([alpha[0], alpha[1], 0.0, 0.0, 0.0, alpha[2]])
        self.A = np.array([A[0], A[1], 0.0, 0.0, 0.0, A[2]])
        self.saturation = np.array([saturation[0], saturation[1], 0.0, 0.0, 0.0, saturation[2]])

        self.J_prev = np.zeros((6,6))


    def control(self, p, v, pI, vI, aI = np.zeros(6)):
        """
        """
        self.robotDynamics.updateJacobian(0.0, 0.0, p[5])

        e1 = pI - p
        e2 = np.dot(self.robotDynamics.J_inv, vI) - v

        while e1[5] > np.pi:
            e1[5] -= 2.0 * np.pi
        while e1[5] < -np.pi:
            e1[5] += 2.0 * np.pi

        #First order SMC
        ss = e2 + self.alpha * e1

        u = np.zeros(6)
        u[0] = self.K[0] * np.tanh(self.A[0] * ss[0])
        u[1] = self.K[1] * np.tanh(self.A[1] * ss[1])
        u[2] = self.K[2] * np.tanh(self.A[2] * ss[2])
        u[3] = self.K[3] * np.tanh(self.A[3] * ss[3])
        u[4] = self.K[4] * np.tanh(self.A[4] * ss[4])
        u[5] = self.K[5] * np.tanh(self.A[5] * ss[5])

        if self.modelBased:
            dyn_ff = np.zeros(6)
            M = self.robotDynamics.M
            D = self.robotDynamics.drag
            dyn_comp = -np.dot(D, v)

            J_inv_dot = (self.robotDynamics.J_inv - self.J_prev) / self.dt

            u = np.dot(M, u + np.dot(J_inv_dot, vI) - 0*self.alpha * (vI - np.dot(self.robotDynamics.J,v) ) + \
                       np.dot(self.robotDynamics.J_inv, aI)) + dyn_comp

        for i in range(6):
            if u[i] >= self.saturation[i]:
                u[i] = self.saturation[i]
            if u[i] <= -self.saturation[i]:
                u[i] = -self.saturation[i]

        self.J_prev = self.robotDynamics.J_inv
        return u
