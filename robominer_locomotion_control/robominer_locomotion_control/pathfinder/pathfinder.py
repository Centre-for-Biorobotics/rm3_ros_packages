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

from geometry_msgs.msg import Point, TwistStamped
from robominer_msgs.msg import WhiskerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from dataclasses import dataclass
from simple_pid import PID

from .a_star.a_star import a_star

from .direction.direction import Direction
from .direction.direction_twist import DirectionTwist

from .graph.node_position import NodePosition
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

TYPE = 'A_STAR'
#TYPE = 'FOLLOW_WALL'
GOAL = NodePosition(0, 0)


class RM3Pathfinder(Node):
    def __init__(self, config_params : dict):
        super().__init__('RM3Pathfinder')
        self.pathfinder_params = config_params

        self.has_had_contact = False
        self.curr_node_position = None
        self.curr_pose = None

        self.direction = 0
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
        self.sub_odom = self.create_subscription(Odometry, ref_frame, self.onOdometry, 10)

        self.sub_whisker = self.create_subscription(WhiskerArray, '/WhiskerStates', self.onWhisker, 10)

        self.robot_speed_pub = self.create_publisher(TwistStamped, '/move_cmd_vel', 10)

        self.create_timer(5, self.log_surroundings)  # every 5 seconds

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
        angle = math.degrees(self.curr_pose.orientation.z * 180 / 57) % 360
        return angle if angle <= 180 else angle - 360

    def onWhisker(self, msg: WhiskerArray):
        """
        Process whisker output for movement input.
        """
        self.whisker_matrix = create_whisker_matrix(msg.whiskers)

        whisker_pressures_avg_tmp = {}
        whisker_pressures_max_tmp = {}
        for direction in Direction:
            whisker_pressures_avg_tmp[direction] = calc_whisker_pressure_avg(self.whisker_matrix[WHISKER_ROWS[direction]])
            whisker_pressures_max_tmp[direction] = calc_whisker_pressure_max(self.whisker_matrix[WHISKER_ROWS[direction]])

            if whisker_pressures_max_tmp[direction] > 0.1:
                self.mark_graph_point_after_collision(direction)
                # self.get_logger().info(self.graph.get_surroundings(self.curr_node_position, 20, self.path))

        self.whisker_pressures_avg = whisker_pressures_avg_tmp
        self.whisker_pressures_max = whisker_pressures_max_tmp

        self.assign_direction_error()

        self.y_axis_movement = self.calcAxisError(self.horizontal_pid, self.publisher_error_y_axis, TRACKED_WALL_DIRECTION, 0.7, 0.3)
        self.x_axis_movement = self.calcAxisError(self.x_axis_pid, self.publisher_error_x_axis, Direction.FORWARD, 0.3, 0.7)

        if not self.has_had_contact and any(p > 0.1 for p in self.whisker_pressures_max.values()):
            self.activate_movement()

        self.publish_errors()

        self.determine_and_publish_movement()

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

        if collision_angle < -135:  # Backward
            x -= marked_distance
        elif collision_angle < -45:  # Right
            y -= marked_distance
        elif collision_angle < 45:  # Forward
            x += marked_distance
        elif collision_angle < 135:  # Left
            y += marked_distance
        else:  # Backward
            x -= marked_distance

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
        
        perpendicular_direction = calc_whisker_pressure_max(self.whisker_matrix[WHISKER_ROWS[Direction.FORWARD]]) # calc_whisker_pressure_avg(self.whisker_matrix[WHISKER_ROWS[Direction.FORWARD]])
        direction += -3 * perpendicular_direction if TRACKED_WALL_DIRECTION == Direction.LEFT else 3 * perpendicular_direction

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
        if not self.has_had_contact:
            self.has_had_contact = True
            self.direction_pid.set_auto_mode(True, self.direction)
            self.horizontal_pid.set_auto_mode(True, self.y_axis_movement)
            self.x_axis_pid.set_auto_mode(True, self.x_axis_movement)
            self.x_axis_turning_pid.set_auto_mode(True, self.x_axis_movement)

            if TYPE == 'A_STAR':
                self.path = a_star(self.graph, self.curr_node_position, GOAL)

    def publish_errors(self):
        self.publisher_output_x_axis.publish(Float64(data=float(self.x_axis_movement)))
        self.publisher_output_y_axis.publish(Float64(data=float(self.y_axis_movement)))
        self.publisher_output_direction.publish(Float64(data=float(self.direction)))

    def log_surroundings(self):
        self.get_logger().info(self.graph.get_surroundings(self.curr_node_position, 20, self.path))
        self.get_logger().info('Pos: ' + str(self.curr_node_position))
        self.get_logger().info('Path: ' + str([str(n) for n in self.path]))
    
    def determine_and_publish_movement(self) -> None:
        twist_msg = TwistStamped()

        mov_lst = self.determine_movement()

        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = float(mov_lst[0])
        twist_msg.twist.linear.y = float(mov_lst[1])
        twist_msg.twist.angular.z = float(mov_lst[2])

        self.robot_speed_pub.publish(twist_msg)

    def get_path_following_angle_error(self):
        if self.path is None or len(self.path) == 0:
            return None

        translated_curr_pos = self.graph.translate_position_to_graph_pos_unrounded(self.curr_pose.position)
        
        if distance(translated_curr_pos.x, translated_curr_pos.y, self.path[0].x, self.path[0].y) < 0.2:
            self.get_logger().warn('Removed first!')
            self.path.pop(0)

        step_p = Point()
        step_p.x, step_p.y = float(self.path[0].x), float(self.path[0].y)

        translated_curr_pos = self.graph.translate_position_to_graph_pos_unrounded(self.curr_pose.position)

        return angle_between_positions(translated_curr_pos, step_p) - self.abs_angle

    def determine_movement(self) -> List[float]:
        """
        Output: [x, y, yaw]
        """
        if TYPE == 'A_STAR':
            return self.determine_movement_a_star()
        elif TYPE == 'FOLLOW_WALL':
            return self.determine_movement_wall_follower()

    def determine_movement_a_star(self):
        if self.curr_node_position == GOAL:
            return [0, 0, 0]

        if self.curr_node_position is not None:
            self.activate_movement()
        else:
            return [0, 0, 0]

        collision_prevention_system_part = self.hard_collision_reverse_system(hard_collision_avg_threshold=0.3, hard_collision_max_threshold=0.7)
        if collision_prevention_system_part != None:
            self.log_movement('Hard collision avoidance')
            return collision_prevention_system_part

        if self.path is not None and len(self.path) != 0:
            self.log_movement('A-star')
            angle_error = self.get_path_following_angle_error()

            if abs(angle_error) > 2:
                return DirectionTwist.TURN_RIGHT.value if angle_error >= 0 else DirectionTwist.TURN_LEFT.value

            return [1, 0, 0]

        self.log_movement('A-star avoidance')
        return [-1, 0, 0]


    def determine_movement_wall_follower(self):
        if not self.has_had_contact:
                    return DirectionTwist.MOVE_RIGHT.value if self.curr_pose is not None else [0, 0, 0]

        factors = list()

        collision_prevention_system_part = self.hard_collision_reverse_system(hard_collision_avg_threshold=0.3, hard_collision_max_threshold=0.7)
        if collision_prevention_system_part != None:
            self.log_movement('Hard collision avoidance')
            return collision_prevention_system_part
        
        self.log_movement('Factor-based movement')
        
        factors.append(Factor(
            'Handle forward pressure',
            100 if self.whisker_pressures_max[Direction.FORWARD] > 0.1 else 0,
            [0, -0.4, -1] if TRACKED_WALL_DIRECTION == Direction.LEFT else [0, 0.4, 1]
        ))

        factors.append(Factor(
            'Handle backward pressure',
            100 if self.whisker_pressures_max[Direction.BACKWARD] > 0.1 else 0,
            [0, 0.4, 1] if TRACKED_WALL_DIRECTION == Direction.LEFT else [0, -0.4, -1]
        ))

        # if self.whisker_pressures_max[Direction.FORWARD] > self.whisker_pressures_max[Direction.BACKWARD] \
        #     and abs(self.whisker_pressures_max[Direction.FORWARD] - self.whisker_pressures_max[Direction.BACKWARD]) > 0.1 :
        #     if TRACKED_WALL_DIRECTION == Direction.LEFT:
        #         vec = [0, -0.4, -1]
        #     else:
        #         vec = [0, 0.4, 1]
        # else: # Backward > Forward
        #     if TRACKED_WALL_DIRECTION == Direction.LEFT:
        #         pass
        #         vec = [0, 0, -1]
        #     else:
        #         pass
        #         vec = [0, 0, 1]

        """
        factors.append(Factor(
            'Handle getting stuck front/back',
            abs(self.x_axis_movement),
            [0, -0.4, -1] if (self.x_axis_movement >= 0) == (TRACKED_WALL_DIRECTION == Direction.LEFT) else [0, 0.4, 1]  # XAND
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
            TRACKED_WALL_DIRECTION.turn_twist() if self.horizontal_movement >= 0.0 else TRACKED_WALL_DIRECTION.opposite().turn_twist()
        ))
        """

        factors.append(Factor(
            'Wall tracking PID (push L/R)',
            abs(self.y_axis_movement),
            TRACKED_WALL_DIRECTION.move_twist() if self.y_axis_movement >= 0.0 else TRACKED_WALL_DIRECTION.opposite().move_twist()
        ))

        factors.append(Factor(
            'Turn to keep level with the side walls',
            abs(self.direction),
            DirectionTwist.TURN_LEFT.value if self.direction >= 0 else DirectionTwist.TURN_RIGHT.value
        ))

        
        
        factors.append(Factor(
            'Forward movement factor',
            1,
            Direction.FORWARD.move_twist()
        ))
        """
        factors.append(Factor(
            'Backward movement factor',
            1 if self.whisker_pressures_max[Direction.BACKWARD] < 0.3 and self.whisker_pressures_max[Direction.FORWARD] > 0.1 else 0,
            Direction.FORWARD.move_twist()
        ))
        """

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
