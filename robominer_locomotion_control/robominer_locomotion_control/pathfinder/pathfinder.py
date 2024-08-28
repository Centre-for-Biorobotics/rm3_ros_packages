"""
Pathfinder
Node that can do wall-following or A* pathfinding to a goal

@author: Tanel Kossas
@contact: tanel.kossas@gmail.com
"""

from __future__ import annotations
from typing import Callable, List
import transforms3d
import math
import numpy as np
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from collections import deque

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, PoseStamped, TwistStamped
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

from .util.whisker_helpers import (
    calc_whisker_pressure_avg,
    calc_whisker_pressure_max,
    calc_whiskers_inclination_euclid,
    directional_whisker_weight,
    create_bias_matrix,
    create_averaged_bias_matrix,
    create_adjusted_whisker_matrix,
    whisker_pressure,
    whiskers_add_noise,
    whiskers_create_simulated_bias_matrix,
    whiskers_apply_simulated_bias,
    WHISKER_ROW_AMOUNT,
    WHISKERS_PER_ROW_AMOUNT,
    NOISE_STD_DEV
)

@dataclass
class Factor:
    description: str
    weight: float
    movement: np.ndarray

pathf_param_from_yaml = os.path.join(
    get_package_share_directory('robominer_locomotion_control'),
    'config',
    'pathfinder_parameters.yaml'
    )

with open(pathf_param_from_yaml, 'r') as pathfinder_file:
    SIMULATION = yaml.load(pathfinder_file, Loader=yaml.FullLoader)["Pathfinder"]["Simulation"] == "enabled"

NO_MOVEMENT = np.array([0, 0, 0], dtype=float)

# Movement
BASE_SPEED_X = 1 if not SIMULATION else 30
BASE_SPEED_Y = 1 if not SIMULATION else 10
BASE_SPEED_TURN = 1 if not SIMULATION else 10

MINIMUM_SLOW_MOVEMENT_VELOCITY = 0.025 if SIMULATION else 0.15

# Pressure thresholds
#    Soft collision can also be considered wall contact with a whisker
SOFT_COLLISION_AVG_P_THRESHOLD = 0.05
SOFT_COLLISION_MAX_P_THRESHOLD = 0.05

#    Thresholds at which to force pulling away from the wall
HARD_COLLISION_AVG_P_THRESHOLD = 0.4
HARD_COLLISION_MAX_P_THRESHOLD = 0.8

MIN_PRESSURE_FOR_COLLISION_MARK_ON_GRAPH = 0.1

# Navigation
ENABLE_WALL_FOLLOWING_IN_PATH_PLANNING = True

NAVIGATION_ALIGN_ANGLE_MAX_ERROR = 5

GRAPH_SQUARE_SIZE = 0.5  # compared to odometry information
GRAPH_ROBOT_SIZE = 2  # compared to graph square size, used to measure line-of-sight to node

NODE_REMOVAL_DISTANCE = 0.5
WALL_FOLLOW_NODE_REMOVAL_DISTANCE = 3
DESTINATION_REACHED_DISTANCE = 0.1

WALL_FOLLOW_CYCLES_AFTER_TURN = 10

# Error weights
DIR_ERROR_FRONT_AND_REAR_AVG_WEIGHT = 0.7
DIR_ERROR_FRONT_AND_REAR_MAX_WEIGHT = (1 - DIR_ERROR_FRONT_AND_REAR_AVG_WEIGHT)
DIR_ERROR_FRONT_AND_REAR_THRESHOLD = .2

Y_AXIS_ERROR_AVG_WEIGHT = 0.7
Y_AXIS_ERROR_MAX_WEIGHT = (1 - DIR_ERROR_FRONT_AND_REAR_AVG_WEIGHT)

# Calibration constants
CALIBRATION_P_MAX_STD_DEV = 0.015
CALIBRATION_P_MAX_STD_DEV_MEASUREMENT_CYCLE_LENGTH = 10

CALIBRATION_MOVEMENT_MULTIPLIER_BACKWARD_AND_FORWARD = .35 if not SIMULATION else .25
CALIBRATION_MOVEMENT_MULTIPLIER_LEFT_AND_RIGHT = .2 if not SIMULATION else .05

CALIBRATION_FINAL_MOVE_BACK_MAX_SPEED = .15 if not SIMULATION else .05

# Hard collision avoidance
HARD_COLLISION_ROTATION_VELOCITY = 0.3
HARD_COLLISION_STUCK_FIX_THRESHOLD_REDUCTION = 0.3

HARD_COLLISION_MOVEMENT_SPEED = 0.03 if SIMULATION else 0.2
HARD_COLLISION_TURN_SPEED = 0.2

RECALCULATE_PATH_WHEN_STUCK_THRESHOLD_SECONDS = 30

# Debugging toggles
LOG_FACTORS = False
LOG_EXECUTION_TIME = False
LOG_HIGHEST_P = False
LOG_SURROUNDINGS = True
LOG_LINE_OF_SIGHT = True # shows L for line of sight to objective in surroundings log

WHISKER_ROW_DICT = {  # default, if every row is present
    Direction.LEFT: [4, 0],
    Direction.RIGHT: [5, 2],
    Direction.FORWARD: [1, 3],
    Direction.BACKWARD: [7, 6]
}

WHISKER_ROW_DICT_SIM = {
    Direction.LEFT: [0, 4],
    Direction.RIGHT: [1, 5],
    Direction.FORWARD: [2, 6],
    Direction.BACKWARD: [3, 7]
}

CYCLES_UNTIL_RECALIBRATION = 1000

class RM3Pathfinder(Node):
    def __init__(self, pathfinder_config_params : dict, sim_params : dict):
        super().__init__('RM3Pathfinder')
        self.pathfinder_params = pathfinder_config_params  # parameters related to the navigation algorithm
        self.sim_params = sim_params  # parameters related to the robot

        self.is_simulation = SIMULATION  # self.pathfinder_params["Pathfinder"]["Simulation"] == "enabled"

        self.direction_to_whisker_row_map = WHISKER_ROW_DICT if not self.is_simulation else WHISKER_ROW_DICT_SIM
        for key, item in self.direction_to_whisker_row_map.items():
            self.get_logger().info("{}: {}".format(str(key), str(item)))

        self.curr_node_position = None
        self.curr_pose = None

        self.curr_node_position_start_time = None

        self.lined_up_with_wall = False
        self.lining_up_initialised = False
        self.wall_follow_cycles_done_after_turning = 0

        self.curr_max_contact_dir = None
        self.curr_max_contact_val = 0
        self.curr_max_contact_threshold = 0.1

        self.use_wall_following = False
        self.align_with_next_path_node = False

        self.prev_hard_collision = None

        self.whisker_matrix = None

        self.simulated_bias_matrix = None
        self.final_bias_matrix = None
        self.bias_matrices = deque(maxlen=self.pathfinder_params["Whiskers"]["BiasNoiseSamples"])

        self.curr_cycles_until_calibration = 0
        self.calibration_initiated = True
        self.calibration_direction_cancelled_due_to_collision = False
        self.directions_calibrated = set()

        self.calibration_p_max_deque = deque(maxlen=CALIBRATION_P_MAX_STD_DEV_MEASUREMENT_CYCLE_LENGTH)

        self.control_vel : float = 1.
        self.pub_vel = self.create_subscription(Float32, '/cmd_pathfinder_vel', self.on_velocity, 10)

        self.algorithm : PathfinderAlgorithm = PathfinderAlgorithm.NONE
        self.pub_algo = self.create_subscription(String, '/cmd_pathfinder_algorithm', self.on_algorithm_select, 10)

        self.tracked_wall_dir: Direction = None

        self.destination : Point = None
        self.pub_dest = self.create_subscription(Point, '/cmd_pathfinder_destination', self.on_destination, 10)

        self.error_dir_wall = 0.
        self.error_dir_path = 0.
        self.error_y = 0.

        self.goal_orientation = None
        self.error_goal_orientation = 0.
        
        self.error_dir_path_unclipped = 0.

        self.p_max_whisker = 0.

        self.whisker_amount = WHISKER_ROW_AMOUNT * WHISKERS_PER_ROW_AMOUNT

        self.filtered_whiskers : np.ndarray = np.zeros(self.whisker_amount * 3)  # Initialise
        self.filter_r = 3

        self.abs_angle = 0.0

        self.graph = Graph(GRAPH_SQUARE_SIZE, GRAPH_ROBOT_SIZE)
        self.path = []

        self.prev_movement_log = ""

        # average and maximum whisker pressures for a given direction
        self.dir_p_avg = {}
        self.dir_p_max = {}

        self.pid_dir_wall = self.create_pid("DirectionWall")
        self.pid_dir_path = self.create_pid("DirectionPath")
        self.pid_y = self.create_pid("Horizontal")

        ref_frame = self.pathfinder_params["Pathfinder"]["ReferenceFrame"]
        self.sub_odom = self.create_subscription(Odometry, ref_frame, self.on_odometry, 10)
        # for real-world use
        self.sub_odom = self.create_subscription(PoseStamped, 'robot_pose_filtered', self.on_pose_stamped, 10)

        self.sub_whisker = self.create_subscription(WhiskerArray, '/whiskers', self.on_whisker, 10)
        # Duplicate for simulation
        self.sub_whisker_sim = self.create_subscription(WhiskerArray, '/WhiskerStates', self.on_whisker, 10)

        self.pub_whisker_unfiltered = self.create_publisher(WhiskerArray, '/whiskers_unfiltered', 10)
        self.pub_whisker_filtered = self.create_publisher(WhiskerArray, '/whiskers_filtered', 10)

        self.get_logger().info('Initialized')

        self.robot_speed_pub = self.create_publisher(TwistStamped, '/move_cmd_vel', 10)

        if LOG_SURROUNDINGS:
            self.create_timer(5, self.log_surroundings)  # every 5 seconds

        # For monitoring only
        
        self.publisher_error_direction_wall = self.create_publisher(Float64, '/whiskerErrors/direction_wall', 10)
        self.publisher_output_direction_wall = self.create_publisher(Float64, '/whiskerErrors/direction_wall_out', 10)
        self.publisher_error_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis', 10)
        self.publisher_output_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis_out', 10)
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
        self.pub_fac_move_back_to_contact = self.create_publisher(Float64, '/factor/move_back_to_contact', 10)
    
    def rows(self, matrix, direction : Direction):
        rows = []
        for i in self.direction_to_whisker_row_map[direction]:
            rows.append(matrix[i])
        return rows
    
    def whisker_rows(self, direction : Direction):
        return self.rows(self.whisker_matrix, direction)
    
    def stop(self):
        self.publish_movement(NO_MOVEMENT)

    def create_pid(self, pid_name):
        pid_params = self.pathfinder_params["Control"]["PID"][pid_name]

        # Start in proportional-only mode
        return PID(Kp=pid_params["Kp"],
                   Ki=pid_params["Ki"],
                   Kd=pid_params["Kd"],
                   setpoint=pid_params["Setpoint"],
                   output_limits=pid_params["Saturation"],
                   auto_mode=True
        )

    def enable_pid_proportional_only(self, pid: PID, pid_name):
        if pid_name not in ["DirectionWall", "DirectionPath", "Horizontal"]:
            self.get_logger().error("No PID named " + str(pid_name))

        pid_params = self.pathfinder_params["Control"]["PID"][pid_name]

        p, _, _ = pid.components
        pid.set_auto_mode(False)
        pid.tunings = pid_params["Kp"], 0, 0
        pid.set_auto_mode(True, 0)

        if pid_name == "Horizontal":
            self.error_y = p
        elif pid_name == "DirectionWall":
            self.error_dir_wall = p
        elif pid_name == "DirectionPath":
            self.error_dir_path = p
        
    def enable_all_pid_components(self, pid: PID, pid_name):
        if pid_name not in ["DirectionWall", "DirectionPath", "Horizontal"]:
            self.get_logger().error("No PID named " + str(pid_name))

        pid_params = self.pathfinder_params["Control"]["PID"][pid_name]

        #pid.set_auto_mode(False)
        pid.tunings = pid_params["Kp"], pid_params["Ki"], pid_params["Kd"]
        pid.set_auto_mode(True)

    def reset_pid_i_component(self, pid: PID, pid_name):
        if pid_name not in ["DirectionWall", "DirectionPath", "Horizontal"]:
            self.get_logger().error("No PID named " + str(pid_name))

        p, i, d = pid.components
        pid.set_auto_mode(False)
        pid.set_auto_mode(True, p)

        if pid_name == "Horizontal":
            self.error_y = p
        elif pid_name == "DirectionWall":
            self.error_dir_wall = p
        elif pid_name == "DirectionPath":
            self.error_dir_path = p

    def on_odometry(self, msg: Odometry):
        self.curr_pose = msg.pose.pose
        self.perform_pose_based_calculations()

    def on_pose_stamped(self, msg: PoseStamped):
        self.curr_pose = msg.pose
        self.perform_pose_based_calculations()

    def perform_pose_based_calculations(self):
        self.abs_angle = self.calc_abs_angle_from_pose()

        new_pos = self.graph.translate_position_to_graph_pos(self.curr_pose.position)

        if self.algorithm in [PathfinderAlgorithm.A_STAR, PathfinderAlgorithm.THETA_STAR] and self.destination is not None and not self.calibration_initiated:
            if self.curr_node_position_start_time is None or self.curr_node_position != new_pos:
                self.curr_node_position_start_time = time.time()
            elif time.time() - self.curr_node_position_start_time > RECALCULATE_PATH_WHEN_STUCK_THRESHOLD_SECONDS:
                self.calculate_new_path()
                self.curr_node_position_start_time = 0

        if self.curr_node_position != new_pos:
            self.curr_node_position = new_pos

            if not self.graph.node_exists(self.curr_node_position):
                self.graph.add_node(self.curr_node_position, passable=True)
                if self.use_wall_following:
                    self.calculate_new_path()

    def calc_abs_angle_from_pose(self) -> float:
        """
        Returns value in range ]-180; 180]
        """
        orient = [self.curr_pose.orientation.w, self.curr_pose.orientation.x, self.curr_pose.orientation.y,
                  self.curr_pose.orientation.z]

        _, _, yaw = transforms3d.euler.quat2euler(orient)
        
        return yaw * 180 / np.pi
    
    def on_algorithm_select(self, msg: String):
        try:
            self.algorithm = PathfinderAlgorithm._value2member_map_[msg.data]
            if self.algorithm == PathfinderAlgorithm.NONE:
                self.inactivate_movement()
            else:
                self.activate_movement()

            self.use_wall_following = False

            if self.algorithm == PathfinderAlgorithm.LEFT_WALL_FOLLOWER:
                self.tracked_wall_dir = Direction.LEFT
            elif self.algorithm == PathfinderAlgorithm.RIGHT_WALL_FOLLOWER:
                self.tracked_wall_dir = Direction.RIGHT
            else:
                self.tracked_wall_dir = None

            self.get_logger().info("Algorithm changed to: " + str(self.algorithm))
        except ValueError:
            self.get_logger().error("Invalid algorithm passed with value: " + msg.data)

    def curr_pathfinding_func(self) -> Callable:
        if self.algorithm == PathfinderAlgorithm.A_STAR:
            return a_star
        elif self.algorithm == PathfinderAlgorithm.THETA_STAR:
            return theta_star
        
        return None

    def on_velocity(self, msg: Float32):
        self.control_vel = msg.data
        self.get_logger().info("Velocity changed to: " + str(self.control_vel))

    def on_destination(self, msg: Point):
        # TODO implement float destinations
        self.destination = NodePosition(int(round(msg.x)), int(round(msg.y)))

        self.get_logger().info("Destination changed to: " + str(self.destination))

        self.calculate_new_path()

    def apply_filtering_to_whiskers(self, whisker_matrix: List[List[Whisker]]):
        """
        Filters whiskers in-place
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

        self.filtered_whiskers += (1 / self.filter_r) * (update_matrix - self.filtered_whiskers)

        # Populate whisker matrix with the new filtered
        for i in range(len(whisker_matrix)):
            if whisker_matrix[i] is None:
                continue

            for j in range(len(whisker_matrix[i])):
                curr_whisker_num = i * WHISKERS_PER_ROW_AMOUNT + j
                whisker_matrix[i][j].x = self.filtered_whiskers[curr_whisker_num]
                whisker_matrix[i][j].y = self.filtered_whiskers[curr_whisker_num + whisker_amount]
                whisker_matrix[i][j].z = self.filtered_whiskers[curr_whisker_num + whisker_amount * 2]

        return whisker_matrix
    
    def publish_filtered_whiskers(self, matrix: List[List[Whisker]], publisher: Publisher):
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
        t = time.time()
        self.whiskers = msg.whiskers

        if self.curr_cycles_until_calibration == 0:
            self.calibration_initiated = True
        else:
            self.curr_cycles_until_calibration -= 1

        self.whisker_matrix = self.preprocess_whiskers(msg.whiskers)

        if self.whisker_matrix is None:
            # No matrix is given at the start due to taking noise samples for calculating bias
            return

        self.assign_whisker_pressures_for_directions()

        self.mark_collision_points()

        self.calculate_errors()

        self.determine_and_publish_movement()

        if LOG_EXECUTION_TIME:
            self.get_logger().info("Execution time: {:.4f}".format(time.time() - t))

    def preprocess_whiskers(self, whiskers: WhiskerArray):
        if self.is_simulation and self.pathfinder_params["Whiskers"]["Simulation"]["AddNoise"] == "enabled":
            whiskers_add_noise(whiskers, -np.pi / 2, np.pi /2)

        if self.is_simulation and self.pathfinder_params["Whiskers"]["Simulation"]["AddSimulatedBias"] == "enabled":
            if self.simulated_bias_matrix is None:
                self.simulated_bias_matrix = whiskers_create_simulated_bias_matrix()
            whiskers_apply_simulated_bias(whiskers, self.simulated_bias_matrix)

        if self.pathfinder_params["Pathfinder"]["SkipInitialBiasMatrixCalculation"] == "enabled":
            self.final_bias_matrix = np.zeros((WHISKER_ROW_AMOUNT, WHISKERS_PER_ROW_AMOUNT, 3), dtype=float)

        if self.final_bias_matrix is None:
            self.bias_matrices.append(create_bias_matrix(whiskers))
            if len(self.bias_matrices) >= self.pathfinder_params["Whiskers"]["BiasNoiseSamples"]:
                self.final_bias_matrix = create_averaged_bias_matrix(self.bias_matrices)
                self.bias_matrices.clear()
            return

        whisker_matrix = create_adjusted_whisker_matrix(whiskers, self.final_bias_matrix)

        # self.publish_filtered_whiskers(whisker_matrix, self.pub_whisker_unfiltered)

        filtered_whiskers = self.apply_filtering_to_whiskers(whisker_matrix)

        # self.publish_filtered_whiskers(filtered_whiskers, self.pub_whisker_filtered)

        if LOG_HIGHEST_P:
            self.log_highest_p(filtered_whiskers)

        return filtered_whiskers
    
    def log_highest_p(self, whisker_matrix: List[List[Whisker]]) -> None:
        curr_max_contact_row = None
        curr_max_contact_val = 0

        for row_ind in range(len(whisker_matrix)):
            whisker_row = whisker_matrix[row_ind]
            if whisker_row is None:
                continue
            for whisker in whisker_row:
                p = whisker_pressure(whisker)

                if p > curr_max_contact_val:
                    curr_max_contact_row = row_ind
                    curr_max_contact_val = p

                if LOG_HIGHEST_P and p > self.p_max_whisker:
                    self.p_max_whisker = p

        if curr_max_contact_val >= self.curr_max_contact_threshold:
            for key, num_lst in self.direction_to_whisker_row_map.items():
                if curr_max_contact_row in num_lst:
                    self.curr_max_contact_dir = key
                    self.curr_max_contact_val = curr_max_contact_val

    def assign_whisker_pressures_for_directions(self) -> None:
        whisker_pressures_avg_tmp = {}
        whisker_pressures_max_tmp = {}
        for direction in [Direction.LEFT, Direction.RIGHT, Direction.FORWARD, Direction.BACKWARD]:
            existing = [v for v in self.whisker_rows(direction) if v is not None]

            if len(existing) > 0:
                whisker_pressures_avg_tmp[direction] = 0
                whisker_pressures_max_tmp[direction] = 0

            existing: np.ndarray
            for lst in existing:
                whisker_pressures_avg_tmp[direction] += calc_whisker_pressure_avg(lst) / len(existing)

            # flatten list for getting a maximum
            whisker_pressures_max_tmp[direction] = calc_whisker_pressure_max([item for sublist in existing if sublist is not None for item in sublist])

        self.dir_p_avg = whisker_pressures_avg_tmp
        self.dir_p_max = whisker_pressures_max_tmp

    def mark_collision_points(self) -> None:
        # Mark all collision with index 7 as positive and towards left of the array.
        additional_angle_multiplier = {
            Direction.LEFT: -1,
            Direction.RIGHT: 1,
            Direction.FORWARD: 1,
            Direction.BACKWARD: -1
        }

        for direction in [Direction.LEFT, Direction.RIGHT, Direction.FORWARD, Direction.BACKWARD]:
            existing = [v for v in self.whisker_rows(direction) if v is not None]
            for lst in existing:
                for i in range(len(lst)):
                    lst[i]: float  # whisker
                    
                    if whisker_pressure(lst[i]) > MIN_PRESSURE_FOR_COLLISION_MARK_ON_GRAPH:
                        additional_angle = 0
                        if i == 7:
                            additional_angle = 40
                        elif i == 0:
                            additional_angle = -40
                        else:
                            dir_weight = directional_whisker_weight(i, WHISKERS_PER_ROW_AMOUNT)
                            dir_weight_minus_one_abs = dir_weight - 1 if dir_weight >= 0 else dir_weight + 1
                            additional_angle = dir_weight_minus_one_abs * 10

                        additional_angle *= additional_angle_multiplier[direction]

                        if not ENABLE_WALL_FOLLOWING_IN_PATH_PLANNING:
                            # The better angle does not seem to work well for regular wall-following
                            additional_angle = 0
                        
                        self.mark_graph_point_after_collision({direction}, add_angle=additional_angle)

    def calculate_errors(self) -> None:
        direction_wall = self.get_direction_error()
        direction_path = self.get_path_following_angle_error()
        self.error_dir_path_unclipped = direction_path
        y_axis_movement = self.calc_axis_error(self.publisher_error_y_axis, self.tracked_wall_dir, Y_AXIS_ERROR_AVG_WEIGHT, Y_AXIS_ERROR_MAX_WEIGHT)

        self.assign_pid_values(direction_wall, direction_path, y_axis_movement)

    def assign_pid_values(self, direction_wall, direction_path_error, y_axis_movement) -> None:
        if (direction_wall > 0) == (self.error_dir_wall > 0):  # Avoid wind-up, set to zero when crossing 0
            p, i, d = self.pid_dir_wall.components
            if i > 2:
                self.reset_pid_i_component(self.pid_dir_wall, 'DirectionWall')
        pid_dir_wall = self.pid_dir_wall(direction_wall)
        self.error_dir_wall = pid_dir_wall if pid_dir_wall is not None else direction_wall

        if (direction_path_error > 0) == (self.error_dir_path > 0):  # Avoid wind-up, set to zero when crossing 0
            self.reset_pid_i_component(self.pid_dir_path, 'DirectionPath')
        pid_dir_path = self.pid_dir_path(direction_path_error)
        self.error_dir_path = pid_dir_path if pid_dir_path is not None else direction_path_error

        if False: # (y_axis_movement > 0) == (self.y_axis_movement > 0):  # Avoid wind-up, set to zero when crossing 0
            p, i, d = self.pid_dir_wall.components
            if i > 0.002:
                self.get_logger().info(str(self.pid_y.components))
                self.pid_y.set_auto_mode(False)
                self.pid_y.set_auto_mode(True)
        pid_y = self.pid_y(y_axis_movement)
        self.error_y = pid_y if pid_y is not None else y_axis_movement

        self.publisher_output_direction_wall.publish(Float64(data=float(self.error_dir_wall)))
        self.publisher_output_direction_path.publish(Float64(data=float(self.error_dir_path)))
        self.publisher_output_y_axis.publish(Float64(data=float(self.error_y)))
        

    def mark_graph_point_after_collision_angle(self, collision_angle) -> None:
        """
        Collision angle is relative to the robot
        """
        if self.curr_node_position is None:
            return

        collision_angle = add_angles(self.abs_angle, collision_angle)

        x, y = self.curr_node_position.x, self.curr_node_position.y

        marked_distance = 1

        if collision_angle < -157.5:  # Rear
            x -= marked_distance
        elif collision_angle < -112.5:  # Rear Right
            x -= marked_distance
            y -= marked_distance
        elif collision_angle < -67.5:  # Right
            y -= marked_distance
        elif collision_angle < -22.5:  # Front Right
            x += marked_distance
            y -= marked_distance
        elif collision_angle < 22.5:  # Front
            x += marked_distance
        elif collision_angle < 67.5:  # Front Left
            x += marked_distance
            y += marked_distance
        elif collision_angle < 112.5:  # Left
            x += marked_distance
        elif collision_angle < 157.5:  # Rear Left
            x -= marked_distance
            y += marked_distance
        else:  # Rear
            x -= marked_distance

        collision_point = NodePosition(x, y)
        if self.graph.node_passable(collision_point):
            self.get_logger().info('Marking new collision!')
            # New collision
            self.graph.mark_node_unpassable(collision_point)
            
            # Recalculate due to new position
            if not self.destination:
                return

            if self.algorithm == PathfinderAlgorithm.THETA_STAR and (len(self.path) == 0 or collision_point in self.graph.line_of_sight_nodes(self.curr_node_position, self.path[0]) \
                or len(self.path) > 1 and collision_point in self.graph.line_of_sight_nodes(self.curr_node_position, self.path[1])):
                self.calculate_new_path()

            elif self.algorithm == PathfinderAlgorithm.A_STAR and collision_point in self.path:
                self.calculate_new_path()

        
    def mark_graph_point_after_collision(self, collision_directions: set[Direction], add_angle=0) -> None:
        if self.curr_node_position is None:
            return

        collision_angle_modification = 0
        if Direction.RIGHT in collision_directions:
            collision_angle_modification = -90
        elif Direction.BACKWARD in collision_directions:
            collision_angle_modification = 180
        elif Direction.LEFT in collision_directions:
            collision_angle_modification = 90

        collision_angle_modification /= len(collision_directions)

        self.mark_graph_point_after_collision_angle(collision_angle_modification + add_angle)

    def get_direction_error(self) -> float:
        """
        Positive values mean the robot is turned towards the left.
        Return value is [-1; 1]
        """
        # 
        direction = 0
        if self.whisker_rows(Direction.RIGHT) is not None:
            direction += calc_whiskers_inclination_euclid(self.whisker_rows(Direction.RIGHT))

        if self.whisker_rows(Direction.LEFT) is not None:
            direction -= calc_whiskers_inclination_euclid(self.whisker_rows(Direction.LEFT))

        if self.whisker_rows(Direction.FORWARD) is not None:
            perpendicular_direction = DIR_ERROR_FRONT_AND_REAR_AVG_WEIGHT * self.dir_p_avg[Direction.FORWARD] \
                + DIR_ERROR_FRONT_AND_REAR_MAX_WEIGHT * self.dir_p_max[Direction.FORWARD]
            if perpendicular_direction > DIR_ERROR_FRONT_AND_REAR_THRESHOLD:
                direction += -perpendicular_direction * 2 if self.tracked_wall_dir == Direction.LEFT else perpendicular_direction * 2
        
        #if self.whisker_rows(Direction.BACKWARD) is not None:
        #    perpendicular_direction = DIR_ERROR_FRONT_AND_REAR_AVG_WEIGHT * self.dir_p_avg[Direction.BACKWARD] \
        #        + DIR_ERROR_FRONT_AND_REAR_MAX_WEIGHT * self.dir_p_max[Direction.BACKWARD]
        #    if perpendicular_direction > DIR_ERROR_FRONT_AND_REAR_THRESHOLD:
        #        direction += perpendicular_direction if self.tracked_wall_dir == Direction.LEFT else -perpendicular_direction

        # Normalize to [-1; 1]
        direction = np.clip(direction / 2, -1, 1)
            
        self.publisher_error_direction_wall.publish(Float64(data=float(direction)))

        return direction

    def calc_axis_error(self, input_publisher: Publisher, addDirection: Direction, avgWeight: float, maxWeight: float) -> float:
        """
        As pressures are [0, 1], then avg and max whisker pressures are also [0, 1], so each axis total error is also [0, 1].
        """
        if addDirection is None:
            input_publisher.publish(Float64(data=0.))
            return 0.

        # TODO implement axis errors in a way that addDirection isn't needed (instead change PID targets)
        avg_component, max_component = 0., 0.
        if addDirection in self.dir_p_avg and addDirection in self.dir_p_max:
            avg_component += self.dir_p_avg[addDirection]
            max_component += self.dir_p_max[addDirection]
        
        if addDirection.opposite() in self.dir_p_avg and addDirection.opposite() in self.dir_p_max:
            avg_component -= self.dir_p_avg[addDirection.opposite()]
            max_component -= self.dir_p_max[addDirection.opposite()]

        total_movement = avgWeight * avg_component + maxWeight * max_component

        input_publisher.publish(Float64(data=float(total_movement)))

        return total_movement
    
    def calculate_new_path(self):
        f = self.curr_pathfinding_func()
        if f is None:
            return

        self.path = f(self.graph, self.curr_node_position, self.destination)

        if len(self.path) == 0:
            if not self.graph.node_passable(self.destination):
                self.destination = None
                self.get_logger().info("The destination is an obstacle, destination removed.")

    def activate_movement(self) -> None:
        self.get_logger().info("Movement activated")
        self.activate_pids()

        if self.algorithm in [PathfinderAlgorithm.A_STAR, PathfinderAlgorithm.THETA_STAR] and self.destination is not None:
            self.calculate_new_path()
        else:
            self.path = []

    def inactivate_movement(self) -> None:
        self.get_logger().info("Movement inactivated")
        self.deactivate_pids()

        if self.algorithm in [PathfinderAlgorithm.A_STAR, PathfinderAlgorithm.THETA_STAR]:
            self.path = []

    def deactivate_pids(self):
        self.deactivate_wall_following_pids()
        self.deactivate_path_planning_pids()

    def deactivate_wall_following_pids(self):
        self.enable_pid_proportional_only(self.pid_y, 'Horizontal')
        self.enable_pid_proportional_only(self.pid_dir_wall, 'DirectionWall')

    def deactivate_path_planning_pids(self):
        self.enable_pid_proportional_only(self.pid_dir_path, 'DirectionPath')

    def activate_pids(self):
        self.activate_wall_following_pids()
        self.activate_path_planning_pids()

    def activate_wall_following_pids(self):
        self.enable_all_pid_components(self.pid_y, 'Horizontal')
        self.enable_all_pid_components(self.pid_dir_wall, 'DirectionWall')

    def activate_path_planning_pids(self):
        self.enable_all_pid_components(self.pid_dir_path, 'DirectionPath')

    def publish_whisker_errors(self):
        self.publisher_output_y_axis.publish(Float64(data=float(self.error_y)))
        self.publisher_output_direction_wall.publish(Float64(data=float(self.error_dir_wall)))

    def log_surroundings(self):
        self.get_logger().info(self.graph.get_surroundings(self.curr_node_position, 20, self.path, log_line_of_sight=LOG_LINE_OF_SIGHT))
        self.get_logger().info('Pos: ' + str(self.curr_node_position))
        self.get_logger().info('Path: ' + str([str(n) for n in self.path]))

        #if len(self.path) >= 1:
        #    self.get_logger().info('LOS 0: ' + str([str(n) for n in self.graph.line_of_sight_nodes_c(self.curr_node_position, self.path[0], 0)]))
        #    self.get_logger().info('LOS -0.5: ' + str([str(n) for n in self.graph.line_of_sight_nodes_c(self.curr_node_position, self.path[0], -0.5)]))
        #    self.get_logger().info('LOS +0.5: ' + str([str(n) for n in self.graph.line_of_sight_nodes_c(self.curr_node_position, self.path[0], 0.5)]))
    
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

        twist_msg.twist.linear.x = mov_lst[0] * BASE_SPEED_X
        twist_msg.twist.linear.y = mov_lst[1] * BASE_SPEED_Y
        twist_msg.twist.angular.z = mov_lst[2] * BASE_SPEED_TURN
        self.robot_speed_pub.publish(twist_msg)

    def get_path_following_angle_error(self) -> float:
        if self.path is None or len(self.path) == 0:
            return 0

        translated_curr_pos = self.graph.translate_position_to_graph_pos_unrounded(self.curr_pose.position)

        while True:
            dist = distance_points(translated_curr_pos, self.path[0])
            if dist < DESTINATION_REACHED_DISTANCE or len(self.path) > 1 and (dist < NODE_REMOVAL_DISTANCE \
                                                                               or self.use_wall_following and dist < WALL_FOLLOW_NODE_REMOVAL_DISTANCE):
                self.path.pop(0)
                if len(self.path) == 0:
                    self.get_logger().info("Zero path left")
                    return 0
            else:
                break

        step_p = Point()
        step_p.x, step_p.y = float(self.path[0].x), float(self.path[0].y)

        self.publisher_abs_angle.publish(Float64(data=float(self.abs_angle)))

        translated_dist = distance_points(translated_curr_pos, step_p)
        self.publisher_error_translated_dist.publish(Float64(data=float(translated_dist)))

        angle_between_pos = angle_between_positions(translated_curr_pos, step_p)
        self.publisher_angle_between_pos.publish(Float64(data=float(angle_between_pos)))

        angle_error = add_angles(self.abs_angle, -angle_between_pos)
        self.publisher_error_direction_path.publish(Float64(data=float(angle_error)))
        
        return angle_error

    def determine_movement(self) -> np.ndarray:
        """
        Output: [x, y, yaw]
        """
        if self.algorithm == PathfinderAlgorithm.NONE:
            self.publish_factors()
            return NO_MOVEMENT
        elif self.algorithm in [PathfinderAlgorithm.A_STAR, PathfinderAlgorithm.THETA_STAR]:
            return self.determine_movement_path_planning()
        elif self.algorithm == PathfinderAlgorithm.LEFT_WALL_FOLLOWER:
            return self.determine_movement_wall_follower(Direction.LEFT)
        elif self.algorithm == PathfinderAlgorithm.RIGHT_WALL_FOLLOWER:
            return self.determine_movement_wall_follower(Direction.RIGHT)
        else:
            self.get_logger().error("Invalid pathfinder algorithm: " + str(self.algorithm))
            return NO_MOVEMENT
        
    def publish_factors(self, factors: List = None, hard_coll: float = 0., calibration: float = 0., move_back_to_contact: float = 0.) -> None:
        base = 0.
        wall_tracking = 0.
        forward = 0.
        direction_wall = 0.
        forward_coll_handle = 0.
        direction_path = 0.

        if factors is not None:
            for factor in factors:
                if factor.description == 'Base normalisation factor':
                    base = factor.weight
                elif factor.description == 'Wall tracking PID (push L/R)':
                    wall_tracking = factor.weight
                elif factor.description == 'Forward movement factor':
                    forward = factor.weight
                elif factor.description == 'Turn to keep level with the side walls':
                    direction_wall = factor.weight
                elif factor.description == 'Handle forward pressure':
                    forward_coll_handle = factor.weight
                elif factor.description == 'Turn to keep level with path':
                    direction_path = factor.weight

        self.pub_fac_zero.publish(Float64(data=base))
        self.pub_fac_wall_distance.publish(Float64(data=wall_tracking))
        self.pub_fac_forward.publish(Float64(data=forward))
        self.pub_fac_direction_wall.publish(Float64(data=direction_wall))
        self.pub_fac_forward_coll_handle.publish(Float64(data=forward_coll_handle))
        self.pub_fac_direction_path.publish(Float64(data=direction_path))
                
        self.pub_fac_hard_coll_handle.publish(Float64(data=float(hard_coll)))
        self.pub_fac_calibration.publish(Float64(data=float(calibration)))
        self.pub_fac_move_back_to_contact.publish(Float64(data=float(move_back_to_contact)))
        
    def collision_and_calibration_check(self, tracked_wall_dir: Direction) -> np.ndarray:
        collision_prevention_system_part = self.hard_collision_reverse_system(tracked_wall_dir)
        if collision_prevention_system_part is not None:
            if self.calibration_initiated:
                self.calibration_direction_cancelled_due_to_collision = True
            # self.log_movement('Hard collision avoidance')
            self.publish_factors(hard_coll=1)
            self.deactivate_pids()
            return collision_prevention_system_part
        
        if self.calibration_initiated:
            calibration_movement = self.calibration_movement()
            if calibration_movement is not None:
                self.log_movement('Calibration')
                self.publish_factors(calibration=1)
                self.deactivate_pids()
                return calibration_movement
        
        return None
    
    def path_planning_wall_following_mode(self, dir: Direction) -> None:
        if not (dir == Direction.LEFT or dir == Direction.RIGHT):
            self.get_logger().error("Path planning mode not left or right, not assigning")

        self.tracked_wall_dir = dir
        self.use_wall_following = True
        self.goal_orientation = None
        self.deactivate_path_planning_pids()
        self.log_movement("ASSIGN & Wall follow")        

    def determine_movement_path_planning(self, skip_pre_movement: bool=False) -> np.ndarray:
        if self.curr_node_position == self.destination or self.destination is None:
            if self.path:
                self.path = []
            self.log_movement("Reached destination!")
            return NO_MOVEMENT

        if not skip_pre_movement:
            pre_movement = self.collision_and_calibration_check(self.tracked_wall_dir)
            if pre_movement is not None:
                return pre_movement

        if ENABLE_WALL_FOLLOWING_IN_PATH_PLANNING:
            wall_following_movement = self.determine_wall_following_movement_in_path_planning()
            if wall_following_movement is not None:
                self.deactivate_path_planning_pids()
                return wall_following_movement
        
        self.activate_path_planning_pids()
        self.deactivate_wall_following_pids()

        self.log_movement('Weight factor-based movement in path planning')

        weight_factors = list()
        weight_factors.append(Factor(
            'Base normalisation factor',
            1,
            NO_MOVEMENT
        ))

        #if (self.dir_p_max[Direction.FORWARD] > SOFT_COLLISION_MAX_P_THRESHOLD):
        #    self.mark_graph_point_after_collision_angle(0)

        MAX_FORWARD_MOVEMENT = 1.0
        forward_weight = MAX_FORWARD_MOVEMENT * np.clip(1 - self.dir_p_max[Direction.FORWARD] / HARD_COLLISION_MAX_P_THRESHOLD, 0, 1)  #  * np.clip(1 - abs(self.direction_path) / 10, 0, 1)
        weight_factors.append(Factor(
            'Forward movement factor',
            forward_weight,
            Direction.FORWARD.move_twist()
        ))

        weight_factors.append(Factor(
            'Turn to keep level with path',
            abs(self.error_dir_path),
            DirectionTwist.TURN_RIGHT.twist if self.error_dir_path >= 0 else DirectionTwist.TURN_LEFT.twist
        ))
        
        if LOG_FACTORS:
            self.get_logger().info(str(weight_factors))

        normalize_factor_weights(weight_factors)
        
        self.publish_factors(weight_factors)

        return self.calculate_movement_from_factors(weight_factors)

    def determine_wall_following_movement_in_path_planning(self) -> np.ndarray:
        left_contact = self.dir_p_max[Direction.LEFT] >= SOFT_COLLISION_MAX_P_THRESHOLD
        right_contact = self.dir_p_max[Direction.RIGHT] >= SOFT_COLLISION_MAX_P_THRESHOLD
        side_contact = left_contact or right_contact

        greater_contact_side = \
            Direction.RIGHT \
            if self.dir_p_max[Direction.RIGHT] > self.dir_p_max[Direction.LEFT] \
            else Direction.LEFT

        if len(self.path) == 0 and side_contact:
            if self.tracked_wall_dir is None or self.dir_p_max[self.tracked_wall_dir] < SOFT_COLLISION_MAX_P_THRESHOLD:
                self.tracked_wall_dir = greater_contact_side
            return self.determine_movement_wall_follower(self.tracked_wall_dir, skip_pre_movement=True)

        if self.align_with_next_path_node:
            if self.error_dir_path_unclipped < -NAVIGATION_ALIGN_ANGLE_MAX_ERROR or self.error_dir_path_unclipped > NAVIGATION_ALIGN_ANGLE_MAX_ERROR:
                return None
            # don't allow wall following until the right and left side have been cleared
            elif left_contact != right_contact:
                return None
            else:
                self.align_with_next_path_node = False

        DIR_THRESHOLD_OPEN = 40
        DIR_THRESHOLD_WALL = 150
        left_inside_of_threshold = not (self.error_dir_path_unclipped < -DIR_THRESHOLD_WALL or self.error_dir_path_unclipped > DIR_THRESHOLD_OPEN)
        right_inside_of_threshold = not (self.error_dir_path_unclipped > DIR_THRESHOLD_WALL or self.error_dir_path_unclipped <= -DIR_THRESHOLD_OPEN)

        tracked_side_within_treshold = self.tracked_wall_dir == Direction.LEFT and left_contact and left_inside_of_threshold \
                or self.tracked_wall_dir == Direction.RIGHT and right_contact and right_inside_of_threshold

        if self.use_wall_following and tracked_side_within_treshold:
            self.log_movement("Within threshold, use wall follow")
            return self.determine_movement_wall_follower(self.tracked_wall_dir, skip_pre_movement=True)
        
        greater_contact_side_within_threshold = greater_contact_side == Direction.LEFT and left_inside_of_threshold \
                                                or greater_contact_side == Direction.RIGHT and right_inside_of_threshold

        # Start wall following if appropriate threshold
        if side_contact and greater_contact_side_within_threshold:
            self.log_movement("side contact within threshold, start wall following on " + str(greater_contact_side_within_threshold))
            self.path_planning_wall_following_mode(greater_contact_side)
            return self.determine_movement_wall_follower(greater_contact_side, skip_pre_movement=True)
        
        # if not within threshold
        if self.use_wall_following or self.tracked_wall_dir is not None and side_contact:
            self.use_wall_following = False
            self.tracked_wall_dir = None
            self.align_with_next_path_node = True
            self.log_movement("cancel wall following, " + str(self.tracked_wall_dir) + " " + str(self.error_dir_path_unclipped))
            return None

        front_contact = self.dir_p_max[Direction.FORWARD] >= SOFT_COLLISION_MAX_P_THRESHOLD
        
        # Change tracked wall direction based on forward pressure if is None
        if self.tracked_wall_dir is None and not side_contact and front_contact:
            self.tracked_wall_dir = Direction.RIGHT if self.error_dir_path >= 0 else Direction.LEFT

        # Navigate to an appropriate place for wall following if it resulted from front weight
        if self.tracked_wall_dir is not None:
            if self.wall_follow_cycles_done_after_turning >= WALL_FOLLOW_CYCLES_AFTER_TURN:
                self.goal_orientation = None
                self.wall_follow_cycles_done_after_turning = 0

            if self.goal_orientation is None:
                self.goal_orientation = add_angles(self.abs_angle, 30 if self.tracked_wall_dir == Direction.RIGHT else -30)

            error_goal_orientation = add_angles(-self.abs_angle, self.goal_orientation)
            if error_goal_orientation < -NAVIGATION_ALIGN_ANGLE_MAX_ERROR or error_goal_orientation > NAVIGATION_ALIGN_ANGLE_MAX_ERROR:
                self.log_movement("Turn!")
                self.publish_factors([Factor(
                    description='Turn to keep level with the side walls',
                    weight=1.0,
                    movement=DirectionTwist.TURN_LEFT.twist * (1 if error_goal_orientation < 0 else -1)
                )])
                                
                self.deactivate_pids()

                self.wall_follow_cycles_done_after_turning = 0

                return DirectionTwist.TURN_LEFT.twist * (1 if error_goal_orientation < 0 else -1)
            
            # try if there's a wall on the tracked side after turning
            if -NAVIGATION_ALIGN_ANGLE_MAX_ERROR < error_goal_orientation  < NAVIGATION_ALIGN_ANGLE_MAX_ERROR:
                self.wall_follow_cycles_done_after_turning += 1
                self.log_movement("Ready to wall follow (fake): " + str(self.tracked_wall_dir))
                return self.determine_movement_wall_follower(self.tracked_wall_dir, skip_pre_movement=True)
            
            self.get_logger().info("Mark goal as None")
            self.goal_orientation = None

        return None

    def determine_movement_wall_follower(self, tracked_wall_dir : Direction, skip_pre_movement: bool=False) -> np.ndarray:
        if tracked_wall_dir not in [Direction.LEFT, Direction.RIGHT]:
            self.get_logger().error("Tracked direction is " + str(tracked_wall_dir) + ", not left or right")
            self.publish_factors()
            return NO_MOVEMENT

        skip_forward = False
        
        if not skip_pre_movement:
            pre_movement = self.collision_and_calibration_check(tracked_wall_dir)
            if pre_movement is not None:
                return pre_movement
        
        if self.dir_p_avg[tracked_wall_dir] < SOFT_COLLISION_AVG_P_THRESHOLD:
            self.publish_factors(move_back_to_contact=1.)            
            skip_forward = True
            self.enable_pid_proportional_only(self.pid_y, 'Horizontal')


        #self.log_movement('Weight factor-based movement')
        weight_factors = list()

        self.activate_wall_following_pids()

        weight_factors.append(Factor(
            'Base normalisation factor',
            1 if not skip_forward else 2,
            NO_MOVEMENT
        ))

        # Don't try to get nearer to the wall if stuck between walls
        weight_factors.append(Factor(
            'Wall tracking PID (push L/R)',
            abs(self.error_y),
            tracked_wall_dir.move_twist() if self.error_y >= 0.0 else tracked_wall_dir.opposite().move_twist()
        ))
        
        # Reduce forward speed when far from wall
        forward_movespeed_percentage_y = np.clip(1 - abs(self.error_y) / 0.5, 0, 1)
        forward_movespeed_percentage_dir = np.clip(1 - abs(self.error_dir_wall) / 0.8, 0, 1)

        MAX_FORWARD_MOVEMENT = 1.0
        forward_weight = MAX_FORWARD_MOVEMENT * forward_movespeed_percentage_y * forward_movespeed_percentage_dir
        weight_factors.append(Factor(
            'Forward movement factor',
            forward_weight if not skip_forward else 0,
            Direction.FORWARD.move_twist()
        ))

        weight_factors.append(Factor(
            'Turn to keep level with the side walls',
            abs(self.error_dir_wall),
            DirectionTwist.TURN_LEFT.twist if self.error_dir_wall >= 0 else DirectionTwist.TURN_RIGHT.twist
        ))

        return self.process_weight_factors(weight_factors)

    def process_weight_factors(self, weight_factors: List[Factor]) -> np.ndarray:
        if LOG_FACTORS:
            self.get_logger().info(str(weight_factors))

        normalize_factor_weights(weight_factors)
        
        self.publish_factors(weight_factors)

        return self.calculate_movement_from_factors(weight_factors)

    def calculate_movement_from_factors(self, factors: List[Factor]) -> np.ndarray:
        output = np.zeros(3)
        for factor in factors:
            output += factor.movement * factor.weight
        return output
        
    def calibration_movement_dir(self, direction: Direction) -> np.ndarray:
        if direction in self.directions_calibrated:
            return

        self.calibration_p_max_deque.append(self.dir_p_max[direction])

        if self.calibration_direction_cancelled_due_to_collision:
            self.get_logger().info("Calibration skipped due to hard collision for: " + str(direction))
            self.directions_calibrated.add(direction)
            
            self.calibration_p_max_deque.clear()
            self.bias_matrices.clear()
            self.calibration_cancelled_due_to_collision = False
        
        if len(self.calibration_p_max_deque) < self.calibration_p_max_deque.maxlen \
            or np.array(self.calibration_p_max_deque).std() > CALIBRATION_P_MAX_STD_DEV:
            if direction in [Direction.BACKWARD, Direction.FORWARD]:
                return direction.opposite().move_twist() * CALIBRATION_MOVEMENT_MULTIPLIER_BACKWARD_AND_FORWARD
            return direction.opposite().move_twist() * CALIBRATION_MOVEMENT_MULTIPLIER_LEFT_AND_RIGHT

        self.bias_matrices.append(create_bias_matrix(self.whiskers))
        if len(self.bias_matrices) == self.bias_matrices.maxlen:
            self.get_logger().info("Calibrated: " + str(direction))
            self.get_logger().info("Calibratable standard dev: " + str(np.array(self.calibration_p_max_deque).std()))
            
            avg_bias_matrix = create_averaged_bias_matrix(self.bias_matrices)
            for row_num in self.direction_to_whisker_row_map[direction]:
                self.final_bias_matrix[row_num] += avg_bias_matrix[row_num]
            
            self.directions_calibrated.add(direction)
            
            # reset for next direction
            self.calibration_p_max_deque.clear()
            self.bias_matrices.clear()
            self.calibration_cancelled_due_to_collision = False
            
            return None

        return NO_MOVEMENT
    
    def calibration_movement(self) -> np.ndarray:
        if self.tracked_wall_dir is not None:
            dirs = [self.tracked_wall_dir, Direction.FORWARD, Direction.BACKWARD, self.tracked_wall_dir.opposite()]
        else:
            dirs = [Direction.LEFT, Direction.RIGHT, Direction.FORWARD, Direction.BACKWARD]

        for direction in dirs:
            mov_lst = self.calibration_movement_dir(direction)
            if mov_lst is not None:
                return mov_lst

        # move back to wall if it was previously used
        if not self.calibration_cancelled_due_to_collision and self.use_wall_following and self.dir_p_avg[self.tracked_wall_dir] < SOFT_COLLISION_AVG_P_THRESHOLD:
                smoothing = np.clip(1 - self.dir_p_avg[self.tracked_wall_dir] / SOFT_COLLISION_AVG_P_THRESHOLD, 0.5, 1)
                return self.tracked_wall_dir.move_twist() * smoothing * CALIBRATION_FINAL_MOVE_BACK_MAX_SPEED

        self.calibration_initiated = False
        self.calibration_cancelled_due_to_collision = False
        self.curr_cycles_until_calibration = CYCLES_UNTIL_RECALIBRATION
        self.directions_calibrated = set()
        return None

    def hard_collision_reverse_system(self, tracked_wall_dir: Direction) -> np.ndarray:
        """
        Check if robot is near collision and return an appropriate twist
        """
        try:
            max_avg_pressure_direction = max(self.dir_p_avg, key=self.dir_p_avg.get)
            max_max_pressure_direction = max(self.dir_p_max, key=self.dir_p_max.get)

            if self.dir_p_avg[max_avg_pressure_direction] > HARD_COLLISION_AVG_P_THRESHOLD:
                moveDirection = max_avg_pressure_direction.opposite()
            elif self.dir_p_max[max_max_pressure_direction] > HARD_COLLISION_MAX_P_THRESHOLD:
                moveDirection = max_max_pressure_direction.opposite()
            else:
                return None
                
            # Fix getting stuck front-back when wall-following
            tr = HARD_COLLISION_STUCK_FIX_THRESHOLD_REDUCTION

            forward_backward_stuck = moveDirection in [Direction.FORWARD, Direction.BACKWARD] \
                and (self.dir_p_max[Direction.FORWARD] >= HARD_COLLISION_MAX_P_THRESHOLD - tr or self.dir_p_avg[Direction.FORWARD] >= HARD_COLLISION_AVG_P_THRESHOLD - tr) \
                and (self.dir_p_max[Direction.BACKWARD] >= HARD_COLLISION_MAX_P_THRESHOLD - tr or self.dir_p_avg[Direction.BACKWARD] >= HARD_COLLISION_AVG_P_THRESHOLD - tr)
            
            forward_backward_stuck_in_a_row = moveDirection in [Direction.FORWARD, Direction.BACKWARD] \
                and self.prev_hard_collision in [Direction.FORWARD, Direction.BACKWARD] and moveDirection != self.prev_hard_collision
            
            left_right_stuck = moveDirection in [Direction.LEFT, Direction.RIGHT] \
                and (self.dir_p_max[Direction.LEFT] >= HARD_COLLISION_MAX_P_THRESHOLD - tr or self.dir_p_avg[Direction.LEFT] >= HARD_COLLISION_AVG_P_THRESHOLD - tr) \
                and (self.dir_p_max[Direction.RIGHT] >= HARD_COLLISION_MAX_P_THRESHOLD - tr or self.dir_p_avg[Direction.RIGHT] >= HARD_COLLISION_AVG_P_THRESHOLD - tr)
            
            left_right_stuck_in_a_row = moveDirection in [Direction.LEFT, Direction.RIGHT] \
                and self.prev_hard_collision in [Direction.LEFT, Direction.RIGHT] and moveDirection != self.prev_hard_collision
            
            #self.get_logger().info(str(forward_backward_stuck) + " " + str(forward_backward_stuck_in_a_row) + " " + str(left_right_stuck) + " " + str(left_right_stuck_in_a_row))
            #self.get_logger().info(str(moveDirection in [Direction.FORWARD, Direction.BACKWARD]) + " " + str(self.prev_hard_collision in [Direction.FORWARD, Direction.BACKWARD]) + " " + str(moveDirection != self.prev_hard_collision) + " " + str(moveDirection) + str(self.prev_hard_collision))

            if forward_backward_stuck or forward_backward_stuck_in_a_row or left_right_stuck or left_right_stuck_in_a_row:
                # move away from high pressure direction, also rotating if there's a tracked direction
                if tracked_wall_dir is None:
                    if forward_backward_stuck or forward_backward_stuck_in_a_row:
                        total_inclination = calc_whiskers_inclination_euclid(self.whisker_rows(Direction.FORWARD)) + calc_whiskers_inclination_euclid(self.whisker_rows(Direction.BACKWARD))
                        move_dir = Direction.RIGHT if total_inclination >= 0 else Direction.LEFT
                    else:  # left-right stuck
                        total_inclination = calc_whiskers_inclination_euclid(self.whisker_rows(Direction.LEFT)) + calc_whiskers_inclination_euclid(self.whisker_rows(Direction.RIGHT))
                        move_dir = Direction.BACKWARD if total_inclination >= 0 else Direction.FORWARD

                if forward_backward_stuck_in_a_row and not forward_backward_stuck or left_right_stuck_in_a_row and not left_right_stuck:
                    self.log_movement("HARDCOLL: fix getting stuck between two sides (in a row!)")
                else:
                    self.log_movement("HARDCOLL: fix getting stuck between two sides")
                mov_lst = tracked_wall_dir.opposite().move_twist() * HARD_COLLISION_MOVEMENT_SPEED if tracked_wall_dir is not None else move_dir.move_twist() * HARD_COLLISION_MOVEMENT_SPEED
                if tracked_wall_dir is not None:
                    rotation_sign = 1 if tracked_wall_dir == Direction.RIGHT else -1
                    mov_lst[2] = rotation_sign * HARD_COLLISION_ROTATION_VELOCITY
                return mov_lst

            self.log_movement("HARDCOLL: regular, to dir" + str(moveDirection))
            self.prev_hard_collision = moveDirection
            return moveDirection.move_twist() * HARD_COLLISION_MOVEMENT_SPEED
        except ValueError:
            self.log_movement("HARDCOLL: ValError!")
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


def add_angles(a1: float, a2: float) -> float:
    """
    Adds two angles. Inputs and output are in degrees.
    """
    angle = (a1 + a2) % 360
    if angle > 180:
        angle -=360
    return angle

def angle_between_positions(p1: Point, p2: Point) -> float:
    """Returns angle between two points in degrees"""
    return math.degrees(math.atan2(p2.y - p1.y, p2.x - p1.x))


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return ((y2 - y1)**2 + (x2 - x1)**2)**0.5


def distance_points(p1: Point, p2: Point) -> float:
    return distance(p1.x, p1.y, p2.x, p2.y)


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
