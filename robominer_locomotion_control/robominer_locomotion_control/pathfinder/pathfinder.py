"""
Pathfinder
Node that can do wall-following or A* pathfinding to a goal

@author: Tanel Kossas
@contact: tanel.kossas@gmail.com
"""

from __future__ import annotations
from typing import Callable
from tf_transformations import euler_from_quaternion

import math
import numpy as np
import os
import yaml

import rclpy
from rclpy.node import Node
from typing import List
from collections import deque

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, Point32, Pose, TwistStamped
from robominer_msgs.msg import WhiskerArray, Whisker, WhiskerPosInGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32, String

from dataclasses import dataclass
from simple_pid import PID

from .pathplanning.a_star import a_star
from .pathplanning.theta_star import theta_star

from .direction.direction import Direction
from .direction.direction_twist import DirectionTwist

from .graph.node_position import NodePosition
from .graph.graph import Graph

from .control.algorithm import PathfinderAlgorithm

from filterpy import kalman

from .util.whisker_helpers import (
    calc_whisker_pressure_avg,
    calc_whisker_pressure_max,
    calc_whiskers_inclination_euclid,
    whisker_pressure,
    create_bias_matrix,
    create_averaged_bias_matrix,
    create_adjusted_whisker_matrix,
    whiskers_add_noise,
    whiskers_create_simulated_bias_matrix,
    whiskers_apply_simulated_bias,
    WHISKER_ROW_AMOUNT,
    WHISKERS_PER_ROW_AMOUNT
)

@dataclass
class Factor:
    description: str
    weight: float
    movement: np.ndarray


NO_MOVEMENT = np.array([0, 0, 0], dtype=float)

# Debugging toggles
LOG_FACTORS = False
LOG_HIGHEST_P = False
USE_KALMAN_FILTER = False
LOG_SURROUNDINGS = True
LOG_LINE_OF_SIGHT = True # shows L for line of sight to objective in surroundings log

class RM3Pathfinder(Node):
    def __init__(self, pathfinder_config_params : dict, sim_params : dict):
        super().__init__('RM3Pathfinder')
        self.pathfinder_params = pathfinder_config_params  # parameters related to the navigation algorithm
        self.sim_params = sim_params  # parameters related to the robot

        self.direction_to_whisker_row_map = get_direction_to_whisker_row_dict(self.sim_params)

        self.active = False
        self.curr_node_position = None
        self.curr_pose = None

        self.whisker_matrix = None
        self.std_err44 = list()

        self.not_touched_wall = True
        self.not_touched_wall_times = 0
        self.not_touched_wall_threshold = 1

        self.simulated_bias_matrix = None
        self.final_bias_matrix = None
        self.bias_matrices = deque(maxlen=self.pathfinder_params["Whiskers"]["BiasNoiseSamples"])

        self.calibration_initiated = True
        self.directions_calibrated = set()

        self.calibration_p_max_deque = deque(maxlen=10)

        self.control_vel : float = 1.
        self.pub_vel = self.create_subscription(Float32, '/cmd_pathfinder_vel', self.on_velocity, 10)

        self.curr_algorithm : PathfinderAlgorithm = PathfinderAlgorithm.NONE
        self.pub_algo = self.create_subscription(String, '/cmd_pathfinder_algorithm', self.on_algorithm_select, 10)

        # TODO get rid of
        self.tracked_wall_direction = Direction.RIGHT

        self.destination : Point32 = None
        self.pub_dest = self.create_subscription(Point32, '/cmd_pathfinder_destination', self.on_destination, 10)

        self.direction_wall = 0
        self.direction_path = 0

        self.p_max_whisker = 0.
        self.dir_max_val = 0.

        self.whisker_amount = WHISKER_ROW_AMOUNT * WHISKERS_PER_ROW_AMOUNT

        if USE_KALMAN_FILTER:
            self.kalman_filter_whiskers_full = kalman.KalmanFilter(dim_x=self.whisker_amount * 3, dim_z=self.whisker_amount * 3)
            self.kalman_filter_whiskers_full.x = np.eye(self.whisker_amount * 3)  # current value
            self.kalman_filter_whiskers_full.F = np.eye(self.whisker_amount * 3)
            self.kalman_filter_whiskers_full.H = np.eye(self.whisker_amount * 3)
            self.kalman_filter_whiskers_full.R = 0 * np.eye(self.whisker_amount * 3)
            self.kalman_filter_whiskers_full.alpha = 1.0
        else:
            self.kalman_filter_whiskers : np.ndarray = np.zeros(self.whisker_amount * 3)  # Initialise
            self.kalman_filter_r = 4

        self.y_axis_movement = 0
        self.x_axis_movement = 0

        self.x_axis_turning = 0

        self.abs_angle = 0.0

        self.graph = Graph()
        self.path = []

        self.prev_movement_log = ""

        self.whisker_pressures_avg = {}
        self.whisker_pressures_max = {}

        self.direction_wall_pid = self.create_pid("DirectionWall")
        self.direction_path_pid = self.create_pid("DirectionPath")
        self.horizontal_pid = self.create_pid("Horizontal")
        self.x_axis_pid = self.create_pid("XAxis")
        self.x_axis_turning_pid = self.create_pid("XAxisTurning")

        ref_frame = self.pathfinder_params["Pathfinder"]["ReferenceFrame"]
        self.sub_odom = self.create_subscription(Odometry, ref_frame, self.on_odometry, 10)
        # for real-world use
        self.sub_odom = self.create_subscription(Pose, 'robot_pose_filtered', self.on_pose, 10)

        self.sub_whisker = self.create_subscription(WhiskerArray, '/whiskers', self.on_whisker, 10)
        # Duplicate for simulation
        self.sub_whisker_sim = self.create_subscription(WhiskerArray, '/WhiskerStates', self.on_whisker, 10)

        self.pub_whisker_pre_kalman = self.create_publisher(WhiskerArray, '/whiskers_unfiltered', 10)
        self.pub_whisker_kalman = self.create_publisher(WhiskerArray, '/whiskers_filtered', 10)

        self.get_logger().info('Initialized')

        self.robot_speed_pub = self.create_publisher(TwistStamped, '/move_cmd_vel', 10)

        if LOG_SURROUNDINGS:
            self.create_timer(5, self.log_surroundings)  # every 5 seconds

        # For monitoring only
        self.publisher_error_direction_wall = self.create_publisher(Float64, '/whiskerErrors/direction_wall', 10)
        self.publisher_output_direction_wall = self.create_publisher(Float64, '/whiskerErrors/direction_wall_out', 10)
        self.publisher_error_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis', 10)
        self.publisher_output_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis_out', 10)
        self.publisher_error_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis', 10)
        self.publisher_output_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis_out', 10)
        self.publisher_error_direction_path = self.create_publisher(Float64, '/whiskerErrors/direction_path', 10)
        self.publisher_output_direction_path = self.create_publisher(Float64, '/whiskerErrors/direction_path_out', 10)
        self.publisher_error_translated_dist = self.create_publisher(Float64, '/whiskerErrors/dist_path', 10)

        self.publisher_angle_between_pos = self.create_publisher(Float64, '/whiskerErrors/angle_between_pos', 10)
        self.publisher_abs_angle = self.create_publisher(Float64, '/whiskerErrors/abs_angle_robot', 10)

        self.pub_fac_zero = self.create_publisher(Float64, '/factor/zero', 10)
        self.pub_fac_forward = self.create_publisher(Float64, '/factor/forward', 10)
        self.pub_fac_wall_distance = self.create_publisher(Float64, '/factor/wall_distance', 10)
        self.pub_fac_direction_wall = self.create_publisher(Float64, '/factor/wall_direction', 10)
        self.pub_fac_direction_path = self.create_publisher(Float64, '/factor/direction_path', 10)
        self.pub_fac_forward_coll_handle = self.create_publisher(Float64, '/factor/forward_coll_handle', 10)
        self.pub_fac_hard_coll_handle = self.create_publisher(Float64, '/factor/hard_coll_handle', 10)
        self.pub_fac_calibration = self.create_publisher(Float64, '/factor/calibration', 10)
    
    def whisker_row(self, direction : Direction):
        return self.whisker_matrix[self.direction_to_whisker_row_map[direction]]
    
    def row(self, matrix, direction : Direction):
        return matrix[self.direction_to_whisker_row_map[direction]]
    
    def stop(self):
        self.publish_movement(NO_MOVEMENT)

    def create_pid(self, pid_name):
        pid_params = self.pathfinder_params["Control"]["PID"][pid_name]

        return PID(Kp=pid_params["Kp"],
                   Ki=pid_params["Ki"],
                   Kd=pid_params["Kd"],
                   setpoint=pid_params["Setpoint"],
                   output_limits=pid_params["Saturation"],
                   auto_mode=False
        )

    def on_odometry(self, msg: Odometry):
        self.on_pose(msg.pose)

    def on_pose(self, msg: Pose):
        self.curr_pose = msg

        self.abs_angle = self.calc_abs_angle()

        self.curr_node_position = self.graph.translate_position_to_graph_pos(self.curr_pose.position)

        if not self.graph.node_exists(self.curr_node_position):
            self.graph.add_node(self.curr_node_position, passable=True)

    def calc_abs_angle(self):
        """
        Orientation.z is approximately -1 to 1, likely ]-1;1].
        """
        orient = [self.curr_pose.orientation.x, self.curr_pose.orientation.y,
                  self.curr_pose.orientation.z, self.curr_pose.orientation.w]

        _, _, yaw = euler_from_quaternion(orient)
        
        return yaw * 180 / np.pi
    
    def on_algorithm_select(self, msg: String):
        try:
            self.curr_algorithm = PathfinderAlgorithm._value2member_map_[msg.data]
            if self.curr_algorithm == PathfinderAlgorithm.NONE:
                self.inactivate_movement()
            else:
                self.activate_movement()

            self.not_touched_wall = True
            self.not_touched_wall_times = 0

            if self.curr_algorithm == PathfinderAlgorithm.LEFT_WALL_FOLLOWER:
                self.tracked_wall_direction = Direction.LEFT
            elif self.curr_algorithm == PathfinderAlgorithm.RIGHT_WALL_FOLLOWER:
                self.tracked_wall_direction = Direction.RIGHT

            self.get_logger().info("Algorithm changed to: " + str(self.curr_algorithm))
        except ValueError:
            self.get_logger().error("Invalid algorithm passed with value: " + msg.data)

    def curr_pathfinding_func(self) -> Callable:
        if self.curr_algorithm == PathfinderAlgorithm.A_STAR:
            return a_star
        elif self.curr_algorithm == PathfinderAlgorithm.THETA_STAR:
            return theta_star
        
        return None

    def on_velocity(self, msg: Float32):
        self.control_vel = msg.data
        self.get_logger().info("Velocity changed to: " + str(self.control_vel))
        self.get_logger().info("Velocity thing passed as type: " + str(type(msg.data)))

    def on_destination(self, msg: Point32):
        # TODO implement float destinations
        self.destination = NodePosition(int(round(msg.x)), int(round(msg.y)))

        self.get_logger().info("Destination changed to: " + str(self.destination))

        # recalculate path
        algo_func = self.curr_pathfinding_func()
        if algo_func is None:
            return
        self.path = algo_func(self.graph, self.curr_node_position, self.destination)

    def apply_kalman_to_whiskers(self, whisker_matrix):
        """
        Applies Kalman in-place.
        """
        whisker_amount = len(whisker_matrix) * len(whisker_matrix[0])

        update_matrix = np.zeros((whisker_amount * 3))

        for i in range(len(whisker_matrix)):
            if whisker_matrix[i] is None:
                continue
            for j in range(len(whisker_matrix[i])):
                curr_whisker_num = i * WHISKERS_PER_ROW_AMOUNT + j
                update_matrix[curr_whisker_num] = whisker_matrix[i][j].x
                update_matrix[curr_whisker_num + whisker_amount] = whisker_matrix[i][j].y
                update_matrix[curr_whisker_num + whisker_amount * 2] = whisker_matrix[i][j].z

        if USE_KALMAN_FILTER:
            self.kalman_filter_whiskers_full.predict()
            self.kalman_filter_whiskers_full.update(update_matrix)
        else:
            self.kalman_filter_whiskers += (1 / self.kalman_filter_r) * (update_matrix - self.kalman_filter_whiskers)

        # Populate whisker matrix with the new kalman filter values
        for i in range(len(whisker_matrix)):
            if whisker_matrix[i] is None:
                continue
            if USE_KALMAN_FILTER:
                for j in range(len(whisker_matrix[i])):
                    curr_whisker_num = i * WHISKERS_PER_ROW_AMOUNT + j
                    whisker_matrix[i][j].x = self.kalman_filter_whiskers_full.x[curr_whisker_num][0]
                    whisker_matrix[i][j].y = self.kalman_filter_whiskers_full.x[curr_whisker_num + whisker_amount][0]
                    whisker_matrix[i][j].z = self.kalman_filter_whiskers_full.x[curr_whisker_num + whisker_amount * 2][0]
            else:
                for j in range(len(whisker_matrix[i])):
                    curr_whisker_num = i * WHISKERS_PER_ROW_AMOUNT + j
                    whisker_matrix[i][j].x = self.kalman_filter_whiskers[curr_whisker_num]
                    whisker_matrix[i][j].y = self.kalman_filter_whiskers[curr_whisker_num + whisker_amount]
                    whisker_matrix[i][j].z = self.kalman_filter_whiskers[curr_whisker_num + whisker_amount * 2]

        return whisker_matrix
    
    def publish_filtered_whiskers(self, matrix, publisher):
        arr = WhiskerArray()
        arr.whiskers = []

        for i in range(len(matrix)):
            if matrix[i] is None:
                continue
            for j in range(len(matrix[i])):
                w = Whisker()
                w.pos = WhiskerPosInGrid()
                w.pos.row_num = i
                w.pos.col_num = j
                w.x = matrix[i][j].x
                w.y = matrix[i][j].y
                w.z = matrix[i][j].z
                arr.whiskers.append(w)

        publisher.publish(arr)

    def on_whisker(self, msg: WhiskerArray):
        """
        Process whisker output for movement input.
        """
        self.whiskers = msg.whiskers
        self.whisker_matrix = self.preprocess_whiskers(msg.whiskers)

        if self.whisker_matrix is None:
            # No matrix is given at the start due to taking noise samples for calculating bias
            return

        self.assign_whisker_pressures_for_directions()

        self.mark_collision_points()

        self.calculate_errors()

        self.determine_and_publish_movement()

    def preprocess_whiskers(self, whiskers: WhiskerArray):
        if self.pathfinder_params["Whiskers"]["Simulation"]["AddNoise"] == "enabled":
            whiskers_add_noise(whiskers, -np.pi / 2, np.pi /2)

        if self.pathfinder_params["Whiskers"]["Simulation"]["AddSimulatedBias"] == "enabled":
            if self.simulated_bias_matrix is None:
                self.simulated_bias_matrix = whiskers_create_simulated_bias_matrix()
            whiskers_apply_simulated_bias(whiskers, self.simulated_bias_matrix)

        if self.pathfinder_params["Pathfinder"]["SkipBiasMatrixCalculation"] == "enabled":
            self.final_bias_matrix = np.zeros((WHISKER_ROW_AMOUNT, WHISKERS_PER_ROW_AMOUNT, 3), dtype=float)

        if self.final_bias_matrix is None:
            self.bias_matrices.append(create_bias_matrix(whiskers))
            if len(self.bias_matrices) >= self.pathfinder_params["Whiskers"]["BiasNoiseSamples"]:
                self.final_bias_matrix = create_averaged_bias_matrix(self.bias_matrices)
                self.bias_matrices.clear()
            return

        whisker_matrix = create_adjusted_whisker_matrix(whiskers, self.final_bias_matrix)

        for whisker_row in whisker_matrix:
            if whisker_row is None:
                continue
            for whisker in whisker_row:
                dist = whisker_pressure(whisker)

                if LOG_HIGHEST_P and dist > self.p_max_whisker:
                    self.p_max_whisker = dist
                    self.get_logger().info("highest pressure value (should be 1 for high values): {:.2f}".format(self.p_max_whisker))

        self.publish_filtered_whiskers(whisker_matrix, self.pub_whisker_pre_kalman)

        kalman_filtered_whiskers = self.apply_kalman_to_whiskers(whisker_matrix)

        self.publish_filtered_whiskers(kalman_filtered_whiskers, self.pub_whisker_kalman)

        return kalman_filtered_whiskers
    
    def assign_whisker_pressures_for_directions(self):
        whisker_pressures_avg_tmp = {}
        whisker_pressures_max_tmp = {}
        for direction in [Direction.LEFT, Direction.RIGHT, Direction.FORWARD, Direction.BACKWARD]:
            if self.whisker_row(direction) is None:
                continue

            whisker_pressures_avg_tmp[direction] = calc_whisker_pressure_avg(self.whisker_row(direction))
            whisker_pressures_max_tmp[direction] = calc_whisker_pressure_max(self.whisker_row(direction))

        self.whisker_pressures_avg = whisker_pressures_avg_tmp
        self.whisker_pressures_max = whisker_pressures_max_tmp

    def mark_collision_points(self):
        for direction in [Direction.LEFT, Direction.RIGHT, Direction.FORWARD, Direction.BACKWARD]:
            if abs(self.whisker_pressures_max[direction]) > 0.1:
                self.mark_graph_point_after_collision(direction)

                # self.get_logger().error("Detected max : " + str(whisker_pressures_max_tmp[direction]) + " " + str(direction))

    def calculate_errors(self):
        direction_wall = self.get_direction_error()
        direction_path = self.get_path_following_angle_error()
        y_axis_movement = self.calc_axis_error(self.publisher_error_y_axis, self.tracked_wall_direction, 0.7, 0.3)
        x_axis_movement = self.calc_axis_error(self.publisher_error_x_axis, Direction.FORWARD, 0.7, 0.3)

        self.assign_pid_values(direction_wall, direction_path, y_axis_movement, x_axis_movement)

    def assign_pid_values(self, direction_wall, direction_path, y_axis_movement, x_axis_movement):
        if (direction_wall > 0) == (self.direction_wall > 0):  # Avoid wind-up, set to zero when crossing 0
            self.direction_wall_pid.set_auto_mode(False)
            self.direction_wall_pid.set_auto_mode(True, self.direction_wall)
        pid_dir_wall = self.direction_wall_pid(direction_wall)
        self.direction_wall = pid_dir_wall if pid_dir_wall is not None else direction_wall

        pid_dir_path = self.direction_path_pid(direction_path)
        self.direction_path = pid_dir_path if pid_dir_path is not None else direction_path

        pid_y = self.horizontal_pid(y_axis_movement)
        self.y_axis_movement = pid_y if pid_y is not None else y_axis_movement

        pid_x = self.x_axis_pid(x_axis_movement)
        self.x_axis_movement = pid_x if pid_x is not None else x_axis_movement

        self.publisher_output_direction_wall.publish(Float64(data=float(self.direction_wall)))
        self.publisher_output_direction_path.publish(Float64(data=float(self.direction_path)))
        self.publisher_output_y_axis.publish(Float64(data=float(self.y_axis_movement)))
        self.publisher_output_x_axis.publish(Float64(data=float(self.x_axis_movement)))
        
    def mark_graph_point_after_collision(self, collision_direction: Direction):
        if self.curr_node_position is None:
            return

        collision_angle_modification = 0
        if collision_direction == Direction.RIGHT:
            collision_angle_modification = -90
        elif collision_direction == Direction.BACKWARD:
            collision_angle_modification = 180
        elif collision_direction == Direction.LEFT:
            collision_angle_modification = 90

        collision_angle = add_angles(self.abs_angle, collision_angle_modification)

        x, y = self.curr_node_position.x, self.curr_node_position.y

        marked_distance = 1

        if collision_angle < -157.5:  # Backward
            x -= marked_distance
        elif collision_angle < -112.5:  # Back Right
            x -= marked_distance
            y -= marked_distance
        elif collision_angle < -67.5:  # Right
            y -= marked_distance
        elif collision_angle < -22.5:  # Forward Right
            x += marked_distance
            y -= marked_distance
        elif collision_angle < 22.5:  # Forward
            x += marked_distance
        elif collision_angle < 67.5:  # Forward Left
            x += marked_distance
        elif collision_angle < 112.5:  # Left
            x += marked_distance
        elif collision_angle < 157.5:  # Backward Left
            y += marked_distance
        else:  # Backward
            x -= marked_distance

        collision_point = NodePosition(x, y)
        if self.graph.node_passable(collision_point):
            # New collision
            self.graph.mark_node_unpassable(collision_point)
            
            # Recalculate due to new position
            if self.destination and (self.path is None or self.path is not None and collision_point in self.path):
                self.path = self.curr_pathfinding_func()(self.graph, self.curr_node_position, self.destination)

    def get_direction_error(self):
        """
        Positive values mean the robot is turned towards the left.
        Return value is [-1; 1]
        """
        # 
        direction = 0
        if self.whisker_row(Direction.RIGHT) is not None:
            direction += calc_whiskers_inclination_euclid(self.whisker_row(Direction.RIGHT))

        if self.whisker_row(Direction.LEFT) is not None:
            direction -= calc_whiskers_inclination_euclid(self.whisker_row(Direction.LEFT))
        
        if self.whisker_row(Direction.FORWARD) is not None:
            perpendicular_direction = .3 * calc_whisker_pressure_avg(self.whisker_row(Direction.FORWARD)) + .7 * calc_whisker_pressure_max(self.whisker_row(Direction.FORWARD))
            if perpendicular_direction > .2:
                direction += -perpendicular_direction * 2 if self.tracked_wall_direction == Direction.LEFT else perpendicular_direction * 2
        
        #if self.whisker_row(Direction.BACKWARD) is not None:
        #    perpendicular_direction = .3 * calc_whisker_pressure_avg(self.whisker_row(Direction.BACKWARD)) + .7 * calc_whisker_pressure_max(self.whisker_row(Direction.BACKWARD))
        #    direction += perpendicular_direction if self.tracked_wall_direction == Direction.LEFT else -perpendicular_direction

        # Normalize to [-1; 1]
        direction /= 3

        if abs(direction) > self.dir_max_val:
            self.dir_max_val = abs(direction)
            # self.get_logger().info("Max direction: {:.2f}".format(self.dir_max_val))
            
        self.publisher_error_direction_wall.publish(Float64(data=float(direction)))

        return direction

    def calc_axis_error(self, input_publisher, addDirection, avgWeight, maxWeight):
        """
        As pressures are [0, 1], then avg and max whisker pressures are also [0, 1], so each axis total error is also [0, 1].
        """
        # TODO implement axis errors in a way that addDirection isn't needed (instead change PID targets)
        avg_component, max_component = 0., 0.
        if addDirection in self.whisker_pressures_avg and addDirection in self.whisker_pressures_max:
            avg_component += self.whisker_pressures_avg[addDirection]
            max_component += self.whisker_pressures_max[addDirection]
        
        if addDirection.opposite() in self.whisker_pressures_avg and addDirection.opposite() in self.whisker_pressures_max:
            avg_component -= self.whisker_pressures_avg[addDirection.opposite()]
            max_component -= self.whisker_pressures_max[addDirection.opposite()]

        total_movement = avgWeight * avg_component + maxWeight * max_component

        input_publisher.publish(Float64(data=float(total_movement)))

        return total_movement

    def activate_movement(self):
        self.get_logger().info("Movement activated")
        self.active = True
        self.direction_wall_pid.set_auto_mode(True, 0)
        self.direction_path_pid.set_auto_mode(True, 0)
        self.horizontal_pid.set_auto_mode(True, 0)
        self.x_axis_pid.set_auto_mode(True, 0)
        self.x_axis_turning_pid.set_auto_mode(True, 0)

        if self.curr_algorithm in [PathfinderAlgorithm.A_STAR, PathfinderAlgorithm.THETA_STAR] and self.destination is not None:
            self.path = self.curr_pathfinding_func()(self.graph, self.curr_node_position, self.destination)
        else:
            self.path = []

    def inactivate_movement(self):
        self.get_logger().info("Movement inactivated")
        self.active = False
        self.inactivate_pids()

        if self.curr_algorithm in [PathfinderAlgorithm.A_STAR, PathfinderAlgorithm.THETA_STAR]:
            self.path = []

    def inactivate_pids(self):
        self.direction_wall_pid.set_auto_mode(False, self.direction_wall)
        self.direction_path_pid.set_auto_mode(False, self.direction_path)
        self.horizontal_pid.set_auto_mode(False, self.y_axis_movement)
        self.x_axis_pid.set_auto_mode(False, self.x_axis_movement)
        self.x_axis_turning_pid.set_auto_mode(False, self.x_axis_movement)

    def publish_errors(self):
        self.publisher_output_x_axis.publish(Float64(data=float(self.x_axis_movement)))
        self.publisher_output_y_axis.publish(Float64(data=float(self.y_axis_movement)))
        self.publisher_output_direction_wall.publish(Float64(data=float(self.direction_wall)))

    def log_surroundings(self):
        self.get_logger().info(self.graph.get_surroundings(self.curr_node_position, 20, self.path, log_line_of_sight=LOG_LINE_OF_SIGHT))
        self.get_logger().info('Pos: ' + str(self.curr_node_position))
        self.get_logger().info('Path: ' + str([str(n) for n in self.path]))
    
    def determine_and_publish_movement(self) -> None:
        mov_twist = self.determine_movement() * self.control_vel
        self.publish_movement(mov_twist)

    def publish_movement(self, mov_lst):
        """
        lst has to be an iterable with three values [x, y, z]
        """
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"

        if self.pathfinder_params["Pathfinder"]["Simulation"] == "enabled":
            twist_msg.twist.linear.x = mov_lst[0] * 10
            twist_msg.twist.linear.y = mov_lst[1] * 10
            twist_msg.twist.angular.z = mov_lst[2] * 10
        else:
            twist_msg.twist.linear.x = mov_lst[0]
            twist_msg.twist.linear.y = mov_lst[1]
            twist_msg.twist.angular.z = mov_lst[2]
        self.robot_speed_pub.publish(twist_msg)

    def get_path_following_angle_error(self):
        if self.path is None or len(self.path) == 0:
            return 0

        translated_curr_pos = self.graph.translate_position_to_graph_pos_unrounded(self.curr_pose.position)

        dist = distance(translated_curr_pos.x, translated_curr_pos.y, self.path[0].x, self.path[0].y)
        
        if dist < 0.5:
            # self.get_logger().warn('Removed first!')
            self.path.pop(0)
            if len(self.path) == 0:
                return 0

        step_p = Point()
        step_p.x, step_p.y = float(self.path[0].x), float(self.path[0].y)

        self.publisher_abs_angle.publish(Float64(data=float(self.abs_angle)))

        translated_dist = distance(translated_curr_pos.x, translated_curr_pos.y, step_p.x, step_p.y)
        self.publisher_error_translated_dist.publish(Float64(data=float(translated_dist)))

        angle_between_pos = angle_between_positions(translated_curr_pos, step_p)
        self.publisher_angle_between_pos.publish(Float64(data=float(angle_between_pos)))

        angle_error = (angle_between_pos - self.abs_angle) * np.clip(translated_dist / 0.7, 0.05, 1)
        self.publisher_error_direction_path.publish(Float64(data=float(angle_error)))
        
        return angle_error

    def determine_movement(self) -> np.ndarray:
        """
        Output: [x, y, yaw]
        """
        if self.curr_algorithm == PathfinderAlgorithm.NONE:
            return NO_MOVEMENT
        elif self.curr_algorithm in [PathfinderAlgorithm.A_STAR, PathfinderAlgorithm.THETA_STAR]:
            return self.determine_movement_path_planning()
        elif self.curr_algorithm == PathfinderAlgorithm.LEFT_WALL_FOLLOWER:
            return self.determine_movement_wall_follower(Direction.LEFT)
        elif self.curr_algorithm == PathfinderAlgorithm.RIGHT_WALL_FOLLOWER:
            return self.determine_movement_wall_follower(Direction.RIGHT)
        else:
            self.get_logger().error("Invalid pathfinder algorithm: " + str(self.curr_algorithm))
            return NO_MOVEMENT

    def determine_movement_path_planning(self):
        if self.curr_node_position == self.destination or self.destination is None:
            if self.path:
                self.path = []
            return NO_MOVEMENT

        collision_prevention_system_part = self.hard_collision_reverse_system(hard_collision_avg_threshold=0.4, hard_collision_max_threshold=0.8, 
                                                                              tracking_direction=None)
        if collision_prevention_system_part is not None:
            self.log_movement('Hard collision avoidance')
            self.publish_factors(hard_coll=1)
            return collision_prevention_system_part * 3
        
        if self.calibration_initiated:
            self.log_movement('Calibration')
            calibration_movement = self.calibration_movement()
            if calibration_movement is not None:
                self.publish_factors(calibration=1)
                return calibration_movement
        
        self.log_movement('Weight factor-based movement')
        weight_factors = list()

        weight_factors.append(Factor(
            'Base normalisation factor',
            1,
            NO_MOVEMENT
        ))

        weight_factors.append(Factor(
            'Turn to keep level with path',
            abs(self.direction_path),
            DirectionTwist.TURN_RIGHT.twist if self.direction_path >= 0 else DirectionTwist.TURN_LEFT.twist
        ))

        MAX_FORWARD_MOVEMENT = 1.0
        forward_weight = MAX_FORWARD_MOVEMENT # * np.clip(1 - abs(self.direction_path) / 10, 0, 1)
        weight_factors.append(Factor(
            'Forward movement factor',
            forward_weight,
            Direction.FORWARD.move_twist()
        ))

        if LOG_FACTORS:
            self.get_logger().info(str(weight_factors))

        normalize_factor_weights(weight_factors)
        
        self.publish_factors(weight_factors)

        return self.calculate_movement_from_factors(weight_factors)

    def publish_factors(self, factors = [], hard_coll = 0., calibration = 0.):
        for factor in factors:
            if factor.description == 'Base normalisation factor':
                self.pub_fac_zero.publish(Float64(data=factor.weight))
            elif factor.description == 'Wall tracking PID (push L/R)':
                self.pub_fac_wall_distance.publish(Float64(data=factor.weight))
            elif factor.description == 'Forward movement factor':
                self.pub_fac_forward.publish(Float64(data=factor.weight))
            elif factor.description == 'Turn to keep level with the side walls':
                self.pub_fac_direction_wall.publish(Float64(data=factor.weight))
            elif factor.description == 'Handle forward pressure':
                self.pub_fac_forward_coll_handle.publish(Float64(data=factor.weight))
            elif factor.description == 'Turn to keep level with path':
                self.pub_fac_direction_path.publish(Float64(data=factor.weight))
                
        self.pub_fac_hard_coll_handle.publish(Float64(data=float(hard_coll)))
        self.pub_fac_calibration.publish(Float64(data=float(calibration)))

    def determine_movement_wall_follower(self, tracked_wall_direction : Direction):
        if tracked_wall_direction not in [Direction.LEFT, Direction.RIGHT]:
            self.get_logger().error("Tracked direction is " + str(tracked_wall_direction) + ", not left or right")
            return NO_MOVEMENT

        #if not self.active:
        #    return tracked_wall_direction.move_twist() if self.curr_pose is not None else NO_MOVEMENT

        collision_prevention_system_part = self.hard_collision_reverse_system(hard_collision_avg_threshold=0.4, hard_collision_max_threshold=0.8, 
                                                                              tracking_direction=tracked_wall_direction)
        if collision_prevention_system_part is not None:
            self.log_movement('Hard collision avoidance')
            self.publish_factors(hard_coll=1)
            return collision_prevention_system_part
        
        if self.calibration_initiated:
            self.log_movement('Calibration')
            calibration_movement = self.calibration_movement()
            if calibration_movement is not None:
                self.publish_factors(calibration=1)
                return calibration_movement
        
        self.log_movement('Weight factor-based movement')
        weight_factors = list()

        if self.whisker_pressures_avg[tracked_wall_direction] < 0.02:
            self.not_touched_wall_times += 1
            if self.not_touched_wall_times + 1 >= self.not_touched_wall_threshold:
                self.inactivate_pids()
                self.not_touched_wall = True
                return tracked_wall_direction.move_twist() * .05
        
        if self.not_touched_wall:
            self.activate_movement()  # reset PID controllers
            self.not_touched_wall = False
            self.not_touched_wall_times = 0

        # Factors are in the order of tuning
        weight_factors.append(Factor(
            'Base normalisation factor',
            1,
            NO_MOVEMENT
        ))

        #if self.whisker_pressures_max[Direction.FORWARD] <= 0.4 and self.whisker_pressures_max[Direction.BACKWARD] <= 0.4:
        # Don't try to get nearer to the wall if stuck between walls
        weight_factors.append(Factor(
            'Wall tracking PID (push L/R)',
            abs(self.y_axis_movement),
            tracked_wall_direction.move_twist() if self.y_axis_movement >= 0.0 else tracked_wall_direction.opposite().move_twist()
        ))
        
        # Reduce forward speed when far from wall
        # setpoint_y_axis_val = self.pathfinder_params["Control"]["PID"]["Horizontal"]["Kp"] * self.pathfinder_params["Control"]["PID"]["Horizontal"]["Setpoint"]
        #forward_movespeed_percentage_y = np.clip(1 - abs(self.y_axis_movement) / (1.25 * setpoint_y_axis_val), 0, 1)

        forward_movespeed_percentage_y = np.clip(1 - abs(self.y_axis_movement) / 0.5, 0, 1)
        forward_movespeed_percentage_dir = np.clip(1 - abs(self.direction_wall) / 0.8, 0, 1)

        MAX_FORWARD_MOVEMENT = 1.0
        forward_weight = MAX_FORWARD_MOVEMENT * forward_movespeed_percentage_y * forward_movespeed_percentage_dir
        weight_factors.append(Factor(
            'Forward movement factor',
            forward_weight,
            Direction.FORWARD.move_twist()
        ))
            
        weight_factors.append(Factor(
            'Turn to keep level with the side walls',
            abs(self.direction_wall),
            DirectionTwist.TURN_LEFT.twist if self.direction_wall >= 0 else DirectionTwist.TURN_RIGHT.twist
        ))

        # movement_left = tracked_wall_direction == Direction.LEFT  # positive
        
        #return [forward_weight * 50, self.y_axis_movement * 50 if movement_left else -self.y_axis_movement * 50, -self.direction]
        """
        weight_factors.append(Factor(
            'Handle forward pressure',
            5 if Direction.FORWARD in self.whisker_pressures_max and self.whisker_pressures_avg[Direction.FORWARD] > 0.2 else 0,
            [0, -0.4, -1] if tracked_wall_direction == Direction.LEFT else [0, 0.4, 1]
        ))
        """

        """
        factors.append(Factor(
            'Handle backward pressure',
            100 if Direction.BACKWARD in self.whisker_pressures_max and self.whisker_pressures_avg[Direction.BACKWARD] > 0.3 else 0,
            [0, 0.4, 1] if tracked_wall_direction == Direction.LEFT else [0, -0.4, -1]
        ))
        """

        """
            weight_factors.append(Factor(
                'Handle getting stuck front/back',
                abs(self.x_axis_movement),
                [0, 0, 1] if (self.x_axis_movement >= 0) == (tracked_wall_direction == Direction.LEFT) else [0, 0, -1]  # XAND
            ))
        """

        """
        factors.append(Factor(
            'Push PID (push F/B)',
            abs(self.x_axis_movement),
            Direction.FORWARD.move_twist() if self.horizontal_movement >= 0.0 else Direction.FORWARD.opposite().move_twist()
        ))
        
        factors.append(Factor(
            'Rotate towards tracked wall when pushing away from forward/backward wall',
            abs(self.x_axis_turning),
            tracked_wall_direction.turn_twist() if self.horizontal_movement >= 0.0 else tracked_wall_direction.opposite().turn_twist()
        ))
        """
        """
        factors.append(Factor(
            'Backward movement factor',
            1 if self.whisker_pressures_max[Direction.BACKWARD] < 0.3 and self.whisker_pressures_max[Direction.FORWARD] > 0.1 else 0,
            Direction.FORWARD.move_twist()
        ))
        """

        if LOG_FACTORS:
            self.get_logger().info(str(weight_factors))

        normalize_factor_weights(weight_factors)
        
        self.publish_factors(weight_factors)

        return self.calculate_movement_from_factors(weight_factors)
    
    def calculate_movement_from_factors(self, factors: List[Factor]):
        output = np.zeros(3)
        for factor in factors:
            output += factor.movement * factor.weight
        return output
        
    def calibration_movement_dir(self, direction):
        if direction in self.directions_calibrated:
            return

        self.calibration_p_max_deque.append(self.whisker_pressures_max[direction])
        
        if len(self.calibration_p_max_deque) < self.calibration_p_max_deque.maxlen \
            or np.array(self.calibration_p_max_deque).std() > 0.01:
            return direction.opposite().move_twist() * .03

        self.bias_matrices.append(create_bias_matrix(self.whiskers))
        if len(self.bias_matrices) == self.bias_matrices.maxlen:
            self.get_logger().info("Calibrated: " + str(direction))
            self.get_logger().info("Calibratable standard dev: " + str(np.array(self.calibration_p_max_deque).std()))
            
            avg_bias_matrix = create_averaged_bias_matrix(self.bias_matrices)
            row = self.direction_to_whisker_row_map[direction]
            self.final_bias_matrix[row] = avg_bias_matrix[row]
            
            self.directions_calibrated.add(direction)
            
            self.calibration_p_max_deque.clear()
            self.bias_matrices.clear()
            
            return None

        return NO_MOVEMENT
    
    def calibration_movement(self):
        for direction in [Direction.LEFT, Direction.RIGHT, Direction.FORWARD, Direction.BACKWARD]:
            mov_lst = self.calibration_movement_dir(direction)
            if mov_lst is not None:
                return mov_lst

        self.calibration_initiated = False
        return None

    def hard_collision_reverse_system(self, hard_collision_avg_threshold, hard_collision_max_threshold, tracking_direction: Direction):
        """
        Check if robot is near collision and return an appropriate twist
        """
        try:
            max_avg_pressure_direction = max(self.whisker_pressures_avg, key=self.whisker_pressures_avg.get)
            max_max_pressure_direction = max(self.whisker_pressures_max, key=self.whisker_pressures_max.get)

            if self.whisker_pressures_avg[max_avg_pressure_direction] > hard_collision_avg_threshold:
                moveDirection = max_avg_pressure_direction.opposite()
                # self.get_logger().info("avg: " + str(max(self.whisker_pressures_avg.values())))
            elif self.whisker_pressures_max[max_max_pressure_direction] > hard_collision_max_threshold:
                moveDirection = max_max_pressure_direction.opposite()
                # self.get_logger().info("max: " + str(max(self.whisker_pressures_max.values())))
            else:
                return None
            
            # Fix getting stuck front-back when wall-following
            if tracking_direction is not None and moveDirection in [Direction.FORWARD, Direction.BACKWARD] \
                and (self.whisker_pressures_max[Direction.FORWARD] >= (hard_collision_max_threshold - 0.2) or self.whisker_pressures_avg[Direction.FORWARD] >= (hard_collision_avg_threshold)  - 0.2) \
                and (self.whisker_pressures_max[Direction.BACKWARD] >= (hard_collision_max_threshold - 0.2) or self.whisker_pressures_avg[Direction.BACKWARD] >= (hard_collision_avg_threshold - 0.2)):
                mov_lst = tracking_direction.opposite().move_twist() * 0.05
                mov_lst[2] = -0.3 if tracking_direction == Direction.LEFT else 0.3
                return mov_lst

            return moveDirection.move_twist() * 0.05
        except ValueError:
            return NO_MOVEMENT

    def log_movement(self, new_movement) -> None:
        if new_movement != self.prev_movement_log:
            self.get_logger().info(new_movement)
            self.prev_movement_log = new_movement


def normalize_factor_weights(factors: List[Factor]) -> None:
    """
    Normalize factor weights to sum up to 1.
    """
    total_sum = sum(abs(factor.weight) for factor in factors)
    for factor in factors:
        factor.weight /= total_sum


def add_angles(a1, a2):
    angle = (a1 + a2) % 360
    if angle > 180:
        angle -=360
    return angle

def angle_between_positions(p1: Point, p2: Point):
    """Returns angle between two points in degrees"""
    return math.degrees(math.atan2(p2.y - p1.y, p2.x - p1.x))


def distance(x1: float, y1: float, x2: float, y2: float):
    return ((y2 - y1)**2 + (x2 - x1)**2)**0.5


def within_threshold(p1, p2, threshold):
    return distance(p1.x, p1.y, p2.x, p2.y) < threshold


def get_direction_to_whisker_row_dict(sim_params):
    whisker_rows = {  # default, if every row is present
        Direction.LEFT: 0,
        Direction.RIGHT: 1,
        Direction.FORWARD: 2,
        Direction.BACKWARD: 3
    }

    """
    if sim_params['sensors']['whiskers']['enable_bottom_whiskers'] != 'enable':
        for dir in whisker_rows:
            whisker_rows[dir] -= 6

    if sim_params['sensors']['whiskers']['enable_side_whiskers'] != 'enable':
        for dir in whisker_rows:
            whisker_rows[dir] -= 2
    """

    return whisker_rows


def main(args=None):
    pathf_param_from_yaml = os.path.join(
        get_package_share_directory('robominer_locomotion_control'),
        'config',
        'pathfinder_parameters.yaml'
        )
    
    with open(pathf_param_from_yaml, 'r') as pathfinder_file:
        pathfinder_parameters = yaml.load(pathfinder_file, Loader=yaml.FullLoader)

    sim_param_yaml = os.path.join(
        get_package_share_directory('rm3_gazebo'),
        'config',
        'simulation_parameters.yaml'
        )

    with open(sim_param_yaml, 'r') as sim_file:
        simulation_parameters = yaml.load(sim_file, Loader=yaml.FullLoader)
    
    rclpy.init(args=args)
    pathfinder = RM3Pathfinder(pathfinder_parameters, simulation_parameters)

    try:
        rclpy.spin(pathfinder)
    except KeyboardInterrupt:
        pathfinder.inactivate_movement()
        pathfinder.stop()
        pathfinder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
