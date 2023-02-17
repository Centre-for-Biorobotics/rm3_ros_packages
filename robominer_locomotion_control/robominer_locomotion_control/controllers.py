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

class PIDController(Node):
    def __init__(self, config_params):
        super().__init__('Pilot')

        self.dt = config_params["Pilot"]["dt"]

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

    def control(self, p, v, pI, vI, J, J_inv):
        # Control function (for PID)
        e1 = np.dot(J_inv, (pI -p))
        e2 = np.dot(J_inv, vI) - v

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
