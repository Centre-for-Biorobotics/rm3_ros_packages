"""
Pathfinder
Node that can do wall-following or A* pathfinding to a goal

@author: Tanel Kossas
@contact: tanel.kossas@gmail.com
"""

from __future__ import annotations

import math
import numpy as np
import os
import yaml

import rclpy
from rclpy.node import Node
from typing import List

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, Point32, TwistStamped
from robominer_msgs.msg import WhiskerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32, String

from dataclasses import dataclass
from simple_pid import PID

from .a_star.a_star import a_star

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
    create_whisker_matrix,
    create_bias_matrix,
    create_averaged_bias_matrix,
    create_adjusted_whisker_matrix,
    whiskers_add_noise,
    whiskers_create_simulated_bias_matrix,
    whiskers_apply_simulated_bias
)

@dataclass
class Factor:
    name: str
    weight: float
    twist: list


NO_MOVEMENT = [0, 0, 0]

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

        self.simulated_bias_matrix = None
        self.final_bias_matrix = None
        self.bias_matrices = []

        self.control_vel : float = 1.
        self.pub_vel = self.create_subscription(Float32, '/cmd_pathfinder_vel', self.on_velocity, 10)

        self.curr_algorithm : PathfinderAlgorithm = PathfinderAlgorithm.NONE
        self.pub_algo = self.create_subscription(String, '/cmd_pathfinder_algorithm', self.on_algorithm_select, 10)

        # TODO get rid of
        self.tracked_wall_direction = Direction.RIGHT

        self.destination : Point32 = None
        self.pub_dest = self.create_subscription(Point32, '/cmd_pathfinder_destination', self.on_destination, 10)

        self.max_z = 0
        self.min_z = 0

        self.direction = 0
        self.kalman_filter = kalman.KalmanFilter(dim_x=3, dim_z=3)  # direction, y_axis, x_axis

        self.kalman_filter.x = np.array([[0.], [0.], [0.]])  # current value
        self.kalman_filter.F = np.array([[1., 0., 0.],
                                         [0., 1., 0.],
                                         [0., 0., 1.]])
        self.kalman_filter.H = np.array([[1., 0., 0.],
                                         [0., 1., 0.],
                                         [0., 0., 1.]])
        self.kalman_filter.R = np.array([[100, 0., 0.],
                                         [0., 100, 0.],
                                         [0., 0., 100]])
        self.kalman_filter.alpha = 1.02

        self.y_axis_movement = 0
        self.x_axis_movement = 0


        self.x_axis_turning = 0

        self.abs_angle = 0.0

        self.graph = Graph()
        self.path = []

        self.prev_movement_log = ""

        self.whisker_pressures_avg = {}
        self.whisker_pressures_max = {}

        self.direction_pid = self.create_pid("Direction")
        self.horizontal_pid = self.create_pid("Horizontal")
        self.x_axis_pid = self.create_pid("XAxis")
        self.x_axis_turning_pid = self.create_pid("XAxisTurning")

        ref_frame = self.pathfinder_params["Pathfinder"]["ReferenceFrame"]
        self.sub_odom = self.create_subscription(Odometry, ref_frame, self.on_odometry, 10)

        self.sub_whisker = self.create_subscription(WhiskerArray, '/WhiskerStates', self.on_whisker, 10)

        self.robot_speed_pub = self.create_publisher(TwistStamped, '/move_cmd_vel', 10)

        # self.create_timer(5, self.log_surroundings)  # every 5 seconds

        # For monitoring only
        self.publisher_error_direction = self.create_publisher(Float64, '/whiskerErrors/direction', 10)
        self.publisher_output_direction = self.create_publisher(Float64, '/whiskerErrors/direction_out', 10)
        self.publisher_error_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis', 10)
        self.publisher_output_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis_out', 10)
        self.publisher_error_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis', 10)
        self.publisher_output_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis_out', 10)
        self.publisher_weight = self.create_publisher(Float64, '/whiskerErrors/direction', 10)
    
    def whisker_row(self, direction : Direction):
        return self.whisker_matrix[self.direction_to_whisker_row_map[direction]]
    
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
        self.curr_pose = msg.pose.pose

        self.abs_angle = self.calc_abs_angle()

        self.curr_node_position = self.graph.translate_position_to_graph_pos(self.curr_pose.position)

        if not self.graph.node_exists(self.curr_node_position):
            self.graph.add_node(self.curr_node_position, passable=True)

    def calc_abs_angle(self):
        """
        Orientation.z is approximately -1 to 1, likely ]-1;1].
        """
        return self.curr_pose.orientation.z * 180
    
    def on_algorithm_select(self, msg: String):
        try:
            self.curr_algorithm = PathfinderAlgorithm._value2member_map_[msg.data]
            if self.curr_algorithm == PathfinderAlgorithm.NONE:
                self.inactivate_movement()
            else:
                self.activate_movement()

            if self.curr_algorithm == PathfinderAlgorithm.LEFT_WALL_FOLLOWER:
                self.tracked_wall_direction = Direction.LEFT
            elif self.curr_algorithm == PathfinderAlgorithm.RIGHT_WALL_FOLLOWER:
                self.tracked_wall_direction = Direction.RIGHT

            self.get_logger().info("Algorithm changed to: " + str(self.curr_algorithm))
        except ValueError:
            self.get_logger().error("Invalid algorithm passed with value: " + msg.data)

    def on_velocity(self, msg: Float32):
        self.control_vel = msg.data
        self.get_logger().info("Velocity changed to: " + str(self.control_vel))
        self.get_logger().info("Velocity thing passed as type: " + str(type(msg.data)))

    def on_destination(self, msg: Point32):
        # TODO implement float destinations
        self.destination = NodePosition(int(round(msg.x)), int(round(msg.y)))

        self.get_logger().info("Destination changed to: " + str(self.destination))

        # recalculate path
        self.path = a_star(self.graph, self.curr_node_position, self.destination)

    def on_whisker(self, msg: WhiskerArray):
        """
        Process whisker output for movement input.
        """
        if self.pathfinder_params["Whiskers"]["AddNoise"] == "enabled":
            whiskers_add_noise(msg.whiskers, -np.pi / 2, np.pi /2)

        if self.pathfinder_params["Whiskers"]["AddSimulatedBias"] == "enabled":
            if self.simulated_bias_matrix is None:
                self.simulated_bias_matrix = whiskers_create_simulated_bias_matrix()
                self.get_logger().info("SIMULATED BIAS MATRIX")
                self.get_logger().info(str(self.simulated_bias_matrix))
            whiskers_apply_simulated_bias(msg.whiskers, self.simulated_bias_matrix)

        if self.final_bias_matrix is None:
            self.bias_matrices.append(create_bias_matrix(msg.whiskers))
            if len(self.bias_matrices) >= self.pathfinder_params["Whiskers"]["BiasNoiseSamples"]:
                self.final_bias_matrix = create_averaged_bias_matrix(self.bias_matrices)
                self.get_logger().info("FINAL BIAS MATRIX")
                self.get_logger().info(str(self.final_bias_matrix))
            return

        # self.get_logger().error(str(self.offset_matrix[1][1][1]))

        self.whisker_matrix = create_adjusted_whisker_matrix(msg.whiskers, self.final_bias_matrix)

        # self.get_logger().error("x AXIS MOVEMENT {:.2f}; y AXIS MOVEMENT {:.2f}".format(self.whisker_row(Direction.RIGHT)[4].x, self.whisker_row(Direction.RIGHT)[4].y))

        whisker_pressures_avg_tmp = {}
        whisker_pressures_max_tmp = {}
        for direction in Direction:
            if self.whisker_row(direction) is None:
                continue

            whisker_pressures_avg_tmp[direction] = calc_whisker_pressure_avg(self.whisker_row(direction))
            whisker_pressures_max_tmp[direction] = calc_whisker_pressure_max(self.whisker_row(direction))

            if abs(whisker_pressures_max_tmp[direction]) > 0.1:
                self.mark_graph_point_after_collision(direction)

                # self.get_logger().error("Detected max : " + str(whisker_pressures_max_tmp[direction]) + " " + str(direction))

        self.whisker_pressures_avg = whisker_pressures_avg_tmp
        self.whisker_pressures_max = whisker_pressures_max_tmp

        direction = self.get_direction_error()
        y_axis_movement = self.calcAxisError(self.horizontal_pid, self.publisher_error_y_axis, self.tracked_wall_direction, 0.7, 0.3)
        x_axis_movement = self.calcAxisError(self.x_axis_pid, self.publisher_error_x_axis, Direction.FORWARD, 0.3, 0.7)

        self.publisher_error_direction.publish(Float64(data=float(direction)))
        self.publisher_error_y_axis.publish(Float64(data=float(y_axis_movement)))
        self.publisher_error_x_axis.publish(Float64(data=float(x_axis_movement)))

        self.kalman_filter.predict()
        self.kalman_filter.update(np.array([[direction, y_axis_movement, x_axis_movement]]))

        direction, y_axis_movement, x_axis_movement = self.kalman_filter.x

        self.publisher_output_direction.publish(Float64(data=float(direction)))
        self.publisher_output_y_axis.publish(Float64(data=float(y_axis_movement)))
        self.publisher_output_x_axis.publish(Float64(data=float(x_axis_movement)))

        self.assign_pid_values(direction, y_axis_movement, x_axis_movement)

        # self.get_logger().error("Y AXIS MOVEMENT {:.2f}; X AXIS MOVEMENT {:.2f}".format(self.y_axis_movement, self.x_axis_movement))

        if not self.active and any(p > 0.1 for p in self.whisker_pressures_max.values()):
            self.activate_movement()

        # self.publish_errors()

        self.determine_and_publish_movement()

    def assign_pid_values(self, direction, y_axis_movement, x_axis_movement):
        pid_dir = self.direction_pid(direction)
        self.direction = pid_dir if pid_dir is not None else direction

        pid_y = self.horizontal_pid(y_axis_movement)
        self.y_axis_movement = pid_y if pid_y is not None else y_axis_movement

        pid_x = self.x_axis_pid(x_axis_movement)
        self.x_axis_movement = pid_x if pid_x is not None else x_axis_movement
        

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
                self.path = a_star(self.graph, self.curr_node_position, self.destination)

    def get_direction_error(self):
        # towards left
        direction = 0
        if self.whisker_row(Direction.RIGHT) is not None:
            direction += calc_whiskers_inclination_euclid(self.whisker_row(Direction.RIGHT))

        if self.whisker_row(Direction.LEFT) is not None:
            direction -= calc_whiskers_inclination_euclid(self.whisker_row(Direction.LEFT))
        
        if self.whisker_row(Direction.FORWARD) is not None:
            perpendicular_direction = calc_whisker_pressure_avg(self.whisker_row(Direction.FORWARD))
            direction += -3 * perpendicular_direction if self.tracked_wall_direction == Direction.LEFT else 3 * perpendicular_direction
        
        #if self.whisker_row(Direction.BACKWARD) is not None:
        #    perpendicular_direction = calc_whisker_pressure_avg(self.whisker_row(Direction.BACKWARD))
        #    direction += 3 * perpendicular_direction if self.tracked_wall_direction == Direction.LEFT else -3 * perpendicular_direction
        

        return direction

        # self.get_logger().info("Direction error: {:.2f}, KALMAN {:.2f}".format(self.direction, kalman_direction))

    def calcAxisError(self, pid, input_publisher, addDirection, avgWeight, maxWeight):
        # TODO implement axis errors in a way that addDirection isn't needed (instead change PID targets)
        avg_component, max_component = 0., 0.
        if addDirection in self.whisker_pressures_avg and addDirection in self.whisker_pressures_max:
            avg_component += self.whisker_pressures_avg[addDirection]
            max_component += self.whisker_pressures_max[addDirection]
        
        if addDirection.opposite() in self.whisker_pressures_avg and addDirection.opposite() in self.whisker_pressures_max:
            avg_component -= self.whisker_pressures_avg[addDirection.opposite()]
            max_component -= self.whisker_pressures_max[addDirection.opposite()]

        total_movement = avgWeight * avg_component + maxWeight * max_component

        return total_movement


    def activate_movement(self):
        self.get_logger().info("Movement activated")
        self.active = True
        self.direction_pid.set_auto_mode(True, self.direction)
        self.horizontal_pid.set_auto_mode(True, self.y_axis_movement)
        self.x_axis_pid.set_auto_mode(True, self.x_axis_movement)
        self.x_axis_turning_pid.set_auto_mode(True, self.x_axis_movement)

        if self.curr_algorithm == PathfinderAlgorithm.A_STAR and self.destination is not None:
            self.path = a_star(self.graph, self.curr_node_position, self.destination)
        else:
            self.path = []

    def inactivate_movement(self):
        self.get_logger().info("Movement inactivated")
        self.active = False
        self.direction_pid.set_auto_mode(False, self.direction)
        self.horizontal_pid.set_auto_mode(False, self.y_axis_movement)
        self.x_axis_pid.set_auto_mode(False, self.x_axis_movement)
        self.x_axis_turning_pid.set_auto_mode(False, self.x_axis_movement)

        if self.curr_algorithm == PathfinderAlgorithm.A_STAR:
            self.path = []

    def publish_errors(self):
        self.publisher_output_x_axis.publish(Float64(data=float(self.x_axis_movement)))
        self.publisher_output_y_axis.publish(Float64(data=float(self.y_axis_movement)))
        self.publisher_output_direction.publish(Float64(data=float(self.direction)))

    def log_surroundings(self):
        self.get_logger().info(self.graph.get_surroundings(self.curr_node_position, 20, self.path))
        self.get_logger().info('Pos: ' + str(self.curr_node_position))
        self.get_logger().info('Path: ' + str([str(n) for n in self.path]))
    
    def determine_and_publish_movement(self) -> None:
        mov_lst = np.array(self.determine_movement()) * self.control_vel
        self.publish_movement(mov_lst)

    def publish_movement(self, mov_lst):
        """
        lst has to be an iterable with three values [x, y, z]
        """
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = float(mov_lst[0])
        twist_msg.twist.linear.y = float(mov_lst[1])
        twist_msg.twist.angular.z = float(mov_lst[2])

        # self.get_logger().info("published: " + str(twist_msg))

        self.robot_speed_pub.publish(twist_msg)

    def get_path_following_angle_error(self):
        if self.path is None or len(self.path) == 0:
            return None

        translated_curr_pos = self.graph.translate_position_to_graph_pos_unrounded(self.curr_pose.position)
        
        if distance(translated_curr_pos.x, translated_curr_pos.y, self.path[0].x, self.path[0].y) < 0.5:
            # self.get_logger().warn('Removed first!')
            self.path.pop(0)

        step_p = Point()
        step_p.x, step_p.y = float(self.path[0].x), float(self.path[0].y)

        translated_curr_pos = self.graph.translate_position_to_graph_pos_unrounded(self.curr_pose.position)

        return angle_between_positions(translated_curr_pos, step_p) - self.abs_angle

    def determine_movement(self) -> List[float]:
        """
        Output: [x, y, yaw]
        """
        if self.curr_algorithm == PathfinderAlgorithm.NONE:
            return NO_MOVEMENT
        elif self.curr_algorithm == PathfinderAlgorithm.A_STAR:
            return self.determine_movement_a_star()
        elif self.curr_algorithm == PathfinderAlgorithm.LEFT_WALL_FOLLOWER:
            return self.determine_movement_wall_follower(Direction.LEFT)
        elif self.curr_algorithm == PathfinderAlgorithm.RIGHT_WALL_FOLLOWER:
            return self.determine_movement_wall_follower(Direction.RIGHT)
        else:
            self.get_logger().error("Invalid pathfinder algorithm: " + str(self.curr_algorithm))
            return NO_MOVEMENT

    def determine_movement_a_star(self):
        if self.curr_node_position == self.destination or self.destination is None:
            return NO_MOVEMENT

        collision_prevention_system_part = self.hard_collision_reverse_system(hard_collision_avg_threshold=0.5, hard_collision_max_threshold=1)
        if collision_prevention_system_part != None:
            #self.log_movement('Hard collision avoidance')
            return collision_prevention_system_part

        if self.path is not None and len(self.path) != 0:
            angle_error = self.get_path_following_angle_error()

            if abs(angle_error) > 2:
                #self.log_movement('A-star rotate')
                return DirectionTwist.TURN_RIGHT.value if angle_error >= 0 else DirectionTwist.TURN_LEFT.value

            #self.log_movement('A-star forward')
            return [1, 0, 0]

        #self.log_movement('A-star avoidance')
        return [-1, 0, 0]


    def determine_movement_wall_follower(self, tracked_wall_direction : Direction):
        if tracked_wall_direction not in [Direction.LEFT, Direction.RIGHT]:
            self.get_logger().error("Tracked direction is " + str(tracked_wall_direction) + ", not left or right")
            return NO_MOVEMENT

        if not self.active:
            return tracked_wall_direction.move_twist() if self.curr_pose is not None else NO_MOVEMENT

        factors = list()

        collision_prevention_system_part = self.hard_collision_reverse_system(hard_collision_avg_threshold=0.4, hard_collision_max_threshold=0.9)
        if collision_prevention_system_part != None:
            self.log_movement('Hard collision avoidance')
            return collision_prevention_system_part
        
        self.log_movement('Factor-based movement')
        
        factors.append(Factor(
            'Handle forward pressure',
            100 if Direction.FORWARD in self.whisker_pressures_max and self.whisker_pressures_max[Direction.FORWARD] > 0.3 else 0,
            [0, -0.4, -1] if tracked_wall_direction == Direction.LEFT else [0, 0.4, 1]
        ))

        factors.append(Factor(
            'Handle backward pressure',
            100 if Direction.BACKWARD in self.whisker_pressures_max and self.whisker_pressures_max[Direction.BACKWARD] > 0.3 else 0,
            [0, 0.4, 1] if tracked_wall_direction == Direction.LEFT else [0, -0.4, -1]
        ))

        # if self.whisker_pressures_max[Direction.FORWARD] > self.whisker_pressures_max[Direction.BACKWARD] \
        #     and abs(self.whisker_pressures_max[Direction.FORWARD] - self.whisker_pressures_max[Direction.BACKWARD]) > 0.1 :
        #     if tracked_wall_direction == Direction.LEFT:
        #         vec = [0, -0.4, -1]
        #     else:
        #         vec = [0, 0.4, 1]
        # else: # Backward > Forward
        #     if tracked_wall_direction == Direction.LEFT:
        #         pass
        #         vec = [0, 0, -1]
        #     else:
        #         pass
        #         vec = [0, 0, 1]

        """
        factors.append(Factor(
            'Handle getting stuck front/back',
            abs(self.x_axis_movement),
            [0, -0.4, -1] if (self.x_axis_movement >= 0) == (tracked_wall_direction == Direction.LEFT) else [0, 0.4, 1]  # XAND
        ))
        """

        """
        factors.append(Factor(
            'Push PID (push F/B)',
            abs(self.x_axis_movement),
            Direction.FORWARD.move_twist() if self.horizontal_movement >= 0.0 else Direction.FORWARD.opposite().move_twist()
        ))
        """
        """
        factors.append(Factor(
            'Rotate towards tracked wall when pushing away from forward/backward wall',
            abs(self.x_axis_turning),
            tracked_wall_direction.turn_twist() if self.horizontal_movement >= 0.0 else tracked_wall_direction.opposite().turn_twist()
        ))
        """

        factors.append(Factor(
            'Wall tracking PID (push L/R)',
            abs(self.y_axis_movement),
            tracked_wall_direction.move_twist() if self.y_axis_movement >= 0.0 else tracked_wall_direction.opposite().move_twist()
        ))

        factors.append(Factor(
            'Turn to keep level with the side walls',
            abs(self.direction),
            DirectionTwist.TURN_LEFT.value if self.direction >= 0 else DirectionTwist.TURN_RIGHT.value
        ))

        factors.append(Factor(
            'Forward movement factor',
            np.clip(1 - abs(self.y_axis_movement), 0, 1),
            Direction.FORWARD.move_twist()
        ))
        """
        factors.append(Factor(
            'Backward movement factor',
            1 if self.whisker_pressures_max[Direction.BACKWARD] < 0.3 and self.whisker_pressures_max[Direction.FORWARD] > 0.1 else 0,
            Direction.FORWARD.move_twist()
        ))
        """

        self.get_logger().info(str(factors))

        return calculate_movement_from_factors(factors)

    def hard_collision_reverse_system(self, hard_collision_avg_threshold, hard_collision_max_threshold):
        """
        Check if robot is near collision and return an appropriate twist
        """
        try:
            max_avg_pressure_direction = max(self.whisker_pressures_avg, key=self.whisker_pressures_avg.get)
            max_max_pressure_direction = max(self.whisker_pressures_max, key=self.whisker_pressures_max.get)

            if self.whisker_pressures_avg[max_avg_pressure_direction] > hard_collision_avg_threshold:
                moveDirection = max_avg_pressure_direction.opposite()
            elif self.whisker_pressures_max[max_max_pressure_direction] > hard_collision_max_threshold:
                moveDirection = max_max_pressure_direction.opposite()
            else:
                return None

            return multiply_list_with_scalar(moveDirection.move_twist(), 0.35)
        except ValueError:
            return NO_MOVEMENT

    def log_movement(self, new_movement) -> None:
        if new_movement != self.prev_movement_log:
            self.get_logger().info(new_movement)
            self.prev_movement_log = new_movement


def multiply_list_with_scalar(_list: List, num) -> List:
    return [i * num for i in _list]


def normalize_factor_weights(factors: List[Factor]) -> None:
    """
    Normalize factor weights to sum up to 1.
    """
    total_sum = sum(factor.weight for factor in factors)
    for factor in factors:
        factor.weight /= total_sum


def calculate_movement_from_factors(factors: List[Factor]):
    normalize_factor_weights(factors)

    out_list = [0] * 3
    for factor in factors:
        factor.twist = multiply_list_with_scalar(factor.twist, factor.weight)
        for i in range(3):
            out_list[i] += factor.twist[i]

    return out_list


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
        Direction.LEFT: 6,
        Direction.RIGHT: 7,
        Direction.FORWARD: 8,
        Direction.BACKWARD: 9
    }

    if sim_params['sensors']['whiskers']['enable_bottom_whiskers'] != 'enable':
        for dir in whisker_rows:
            whisker_rows[dir] -= 6

    if sim_params['sensors']['whiskers']['enable_side_whiskers'] != 'enable':
        for dir in whisker_rows:
            whisker_rows[dir] -= 2

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
