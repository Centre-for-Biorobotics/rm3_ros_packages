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

from geometry_msgs.msg import TwistStamped
from robominer_msgs.msg import WhiskerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from dataclasses import dataclass
from simple_pid import PID

from .a_star.a_star import a_star

from .direction.direction import Direction
from .direction.direction_twist import DirectionTwist

from .graph.node_position import NodePosition
from .graph.graph import GraphNode
from .graph.graph import Graph

from .util.whisker_helpers import (
    calc_whisker_pressure_avg,
    calc_whisker_pressure_max,
    calc_whiskers_inclination_euclid,
    create_whisker_matrix
)

@dataclass
class Factor:
    name: str
    weight: float
    twist: list


WHISKER_ROWS = {
    Direction.LEFT: 6,
    Direction.RIGHT: 7,
    Direction.FORWARD: 8,
    Direction.BACKWARD: 9
}

TRACKED_WALL_DIRECTION: Direction = Direction.RIGHT

GOAL = NodePosition(-40, 0)


class RM3Pathfinder(Node):
    def __init__(self, config_params : dict):
        super().__init__('RM3Pathfinder')
        self.pathfinder_params = config_params

        self.has_had_contact = False
        self.curr_node_position = None
        self.curr_pose = None

        self.direction = 0
        self.horizontal_movement = 0
        self.x_axis_movement = 0
        self.x_axis_turning = 0

        self.abs_angle = None

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
        self.sub_odom = self.create_subscription(Odometry, ref_frame, self.onOdometry, 10)

        self.sub_whisker = self.create_subscription(WhiskerArray, '/WhiskerStates', self.onWhisker, 10)

        self.robot_speed_pub = self.create_publisher(TwistStamped, '/move_cmd_vel', 10)

        # For monitoring only
        self.publisher_error_direction = self.create_publisher(Float64, '/whiskerErrors/direction', 10)
        self.publisher_output_direction = self.create_publisher(Float64, '/whiskerErrors/direction_out', 10)
        self.publisher_error_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis', 10)
        self.publisher_output_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis_out', 10)
        self.publisher_error_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis', 10)
        self.publisher_output_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis_out', 10)
        self.publisher_weight = self.create_publisher(Float64, '/whiskerErrors/direction', 10)

    def create_pid(self, pid_name):
        pid_params = self.pathfinder_params["Control"]["PID"][pid_name]

        return PID(Kp=pid_params["Kp"],
                   Ki=pid_params["Ki"],
                   Kd=pid_params["Kd"],
                   setpoint=pid_params["Setpoint"],
                   output_limits=pid_params["Saturation"],
                   auto_mode=False
        )

    def onOdometry(self, msg: Odometry):
        self.curr_pose = msg.pose.pose

        self.abs_angle = self.calc_abs_angle()

        self.curr_node_position = self.graph.translate_position_to_graph_pos(self.curr_pose.position)

        if not self.graph.node_exists(self.curr_node_position):
            self.graph.add_node(self.curr_node_position, passable=True)

    def calc_abs_angle(self):
        angle = math.degrees(self.curr_pose.orientation.z)
        return angle if angle <= 180 else angle - 360

    def onWhisker(self, msg: WhiskerArray):
        """
        Process whisker output for movement input.
        """
        # self.get_logger().info("-----")
        # self.get_logger().info(self.get_surroundings(self.curr_node_position, 10))
        # self.get_logger().info('Pos: ' + str(self.curr_node_position))

        self.whisker_matrix = create_whisker_matrix(msg.whiskers)

        whisker_pressures_avg_tmp = {}
        whisker_pressures_max_tmp = {}
        for direction in Direction:
            whisker_pressures_avg_tmp[direction] = calc_whisker_pressure_avg(self.whisker_matrix[WHISKER_ROWS[direction]])
            whisker_pressures_max_tmp[direction] = calc_whisker_pressure_max(self.whisker_matrix[WHISKER_ROWS[direction]])

            if whisker_pressures_max_tmp[direction] > 0.05:
                self.mark_graph_point_after_collision(direction)

        self.whisker_pressures_avg = whisker_pressures_avg_tmp
        self.whisker_pressures_max = whisker_pressures_max_tmp

        self.assign_direction_error()

        self.horizontal_movement = self.calcAxisError(self.horizontal_pid, self.publisher_error_y_axis, TRACKED_WALL_DIRECTION, 0.7, 0.3)
        self.x_axis_movement = self.calcAxisError(self.x_axis_pid, self.publisher_error_x_axis, Direction.FORWARD, 0.3, 0.7)

        if not self.has_had_contact and any(p > 0.1 for p in self.whisker_pressures_max.values()):
            self.activate_movement()

        self.publish_errors()

        self.determine_and_publish_movement()

    def mark_graph_point_after_collision(self, collision_direction: Direction):
        # self.get_logger().info('Mark collision')
        collision_angle_modification = 0
        if collision_direction == Direction.RIGHT:
            collision_angle_modification = 90
        elif collision_direction == Direction.BACKWARD:
            collision_angle_modification = 180
        elif collision_direction == Direction.BACKWARD:
            collision_angle_modification = 270

        collision_angle = add_angles(self.abs_angle, collision_angle_modification)

        x, y = self.curr_node_position.x, self.curr_node_position.y

        if collision_angle < -135:
            y -= 1
        elif collision_angle < -45:
            x -= 1
        elif collision_angle < 45:
            y += 1
        elif collision_angle < 135:
            x += 1
        else:
            y -= 1

        collision_point = NodePosition(x, y)
        if self.graph.node_passable(collision_point):
            # New collision
            self.graph.mark_node_unpassable(collision_point)
            # Recalculate due to new position
            if self.path is None or self.path is not None and collision_point in self.path:
                self.path = a_star(self.graph, self.curr_node_position, GOAL)

    def assign_direction_error(self):
        # towards left
        direction = calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[Direction.RIGHT]]) - calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[Direction.LEFT]])
        direction += 4*(calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[Direction.BACKWARD]]) - calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[Direction.FORWARD]]))

        self.publisher_error_direction.publish(Float64(data=float(direction)))

        pid_dir = self.direction_pid(direction)

        if pid_dir is not None:
            self.direction = pid_dir
        else:
            self.direction = direction

    def calcAxisError(self, pid, input_publisher, addDirection, avgWeight, maxWeight):
        avg_component = self.whisker_pressures_avg[addDirection] - self.whisker_pressures_avg[addDirection.opposite()]
        max_component = self.whisker_pressures_max[addDirection] - self.whisker_pressures_max[addDirection.opposite()]
        total_movement = avgWeight * avg_component + maxWeight * max_component

        input_publisher.publish(Float64(data=float(total_movement)))

        pid_movement = pid(total_movement)

        return pid_movement if pid_movement is not None else total_movement

    def activate_movement(self):
            self.has_had_contact = True
            self.direction_pid.set_auto_mode(True, self.direction)
            self.horizontal_pid.set_auto_mode(True, self.horizontal_movement)
            self.x_axis_pid.set_auto_mode(True, self.x_axis_movement)
            self.x_axis_turning_pid.set_auto_mode(True, self.x_axis_movement)

    def publish_errors(self):
        self.publisher_output_x_axis.publish(Float64(data=float(self.x_axis_movement)))
        self.publisher_output_y_axis.publish(Float64(data=float(self.horizontal_movement)))
        self.publisher_output_direction.publish(Float64(data=float(self.direction)))
    
    def determine_and_publish_movement(self) -> None:
        twist_msg = TwistStamped()

        mov_lst = self.determine_movement()

        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = float(mov_lst[0])
        twist_msg.twist.linear.y = float(mov_lst[1])
        twist_msg.twist.angular.z = float(mov_lst[2])

        self.robot_speed_pub.publish(twist_msg)

    def determine_movement(self) -> List[float]:
        """
        Output: [x, y, yaw]
        """
        if not self.has_had_contact:
            return DirectionTwist.MOVE_RIGHT.value if self.curr_pose is not None else [0, 0, 0]

        factors = list()
        
        HARD_COLLISION_AVG_THRESHOLD = 0.4
        HARD_COLLISION_MAX_THRESHOLD = 0.8
        collision_prevention_system_part = self.hard_collision_reverse_system(HARD_COLLISION_AVG_THRESHOLD, HARD_COLLISION_MAX_THRESHOLD)
        # collision_prevention_weight = 0
        if collision_prevention_system_part != None:
            self.log_movement('Hard collision avoidance')
            return collision_prevention_system_part
        
        factors.append(Factor(
            'Handle forward pressure',
            100 if self.whisker_pressures_max[Direction.FORWARD] > 0.1 else 0,
            # [0, -0.4, 0] if TRACKED_WALL_DIRECTION == LEFT else [0, 0.4, 0]
            [0, -0.4, -1] if TRACKED_WALL_DIRECTION == Direction.LEFT else [0, 0.4, 1]
        ))

        factors.append(Factor(
            'Handle backward pressure',
            100 if self.whisker_pressures_max[Direction.BACKWARD] > 0.1 else 0,
            [0, 0.4, 1] if TRACKED_WALL_DIRECTION == Direction.LEFT else [0, -0.4, -1]
            # [0, 0.4, 0] if TRACKED_WALL_DIRECTION == LEFT else [0, -0.4, 0]
        ))

        """
        factors.append(Factor(
            'Push front/back if colliding',
            400 if max(self.whisker_pressures_max[BACKWARD], self.whisker_pressures_max[FORWARD]) > 0.02 else 0,
            
        ))
        """
        
        """
        factors.append(Factor(
            'Push PID (push F/B)',
            abs(self.x_axis_movement),
            FORWARD.move_twist() if self.horizontal_movement >= 0.0 else FORWARD.opposite().move_twist()
        ))

        factors.append(Factor(
            'Rotate towards tracked wall when pushing away from forward/backward wall',
            abs(self.x_axis_turning),
            TRACKED_WALL_DIRECTION.turn_twist() if self.horizontal_movement >= 0.0 else TRACKED_WALL_DIRECTION.opposite().turn_twist()
        ))
        """

        factors.append(Factor(
            'Wall tracking PID (push L/R)',
            abs(self.horizontal_movement),
            TRACKED_WALL_DIRECTION.move_twist() if self.horizontal_movement >= 0.0 else TRACKED_WALL_DIRECTION.opposite().move_twist()
        ))

        factors.append(Factor(
            'Turn to keep level with the side walls',
            abs(self.direction),
            DirectionTwist.TURN_LEFT.value if self.direction >= 0 else DirectionTwist.TURN_RIGHT.value
        ))
        

        factors.append(Factor(
            'Straight movement factor',
            1,
            Direction.FORWARD.move_twist()
        ))

        return calculate_movement_from_factors(factors)

    def hard_collision_reverse_system(self, hard_collision_avg_threshold, hard_collision_max_threshold):
        """
        Check if robot is near collision and return an appropriate twist
        """
        max_avg_pressure_direction = max(self.whisker_pressures_avg, key=self.whisker_pressures_avg.get)
        max_max_pressure_direction = max(self.whisker_pressures_max, key=self.whisker_pressures_max.get)

        if self.whisker_pressures_avg[max_avg_pressure_direction] > hard_collision_avg_threshold:
            moveDirection = max_avg_pressure_direction.opposite()
        elif self.whisker_pressures_max[max_max_pressure_direction] > hard_collision_max_threshold:
            moveDirection = max_max_pressure_direction.opposite()
        else:
            return None

        return multiply_list_with_scalar(moveDirection.move_twist(), 0.35)

    def log_movement(self, new_movement) -> None:
        if new_movement != self.prev_movement_log:
            self.get_logger().info(new_movement)
            self.prev_movement_log = new_movement

    def get_surroundings(self, center : NodePosition, distance : int) -> str:
        if center is None:
            return

        x = center.x
        y = center.y

        txt = "\n"
        for i in range(-distance, distance + 1):
            for j in range(-distance, distance + 1):
                this_pos = NodePosition(x + i, y + j)
                if self.path is not None and this_pos in self.path:
                    txt += 'G'
                    continue

                if not self.graph.node_exists(this_pos):
                    txt += 'o'
                    continue

                node: GraphNode = self.graph.nodes[this_pos]
                txt += 'O' if node.passable else 'X'

            txt += "\n"

        return txt


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
    angle = (a2 - a1) % 360
    if angle > 180:
        angle -=360
    return angle


def main(args=None):
    parameters_from_yaml = os.path.join(
            get_package_share_directory('robominer_locomotion_control'),
            'config',
            'pathfinder_parameters.yaml'
            )
    
    with open(parameters_from_yaml, 'r') as file:
        pathfinder_parameters = yaml.load(file, Loader=yaml.FullLoader)
    
    rclpy.init(args=args)
    pathfinder = RM3Pathfinder(pathfinder_parameters)

    rclpy.spin(pathfinder)

    pathfinder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
