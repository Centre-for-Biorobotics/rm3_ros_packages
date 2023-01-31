#!/usr/bin/python3
"""
Takes robot body velocities as input from joystick.
Calculates inverse kinematics to determine screw velocities.
Publishes screw velocities

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 29-08-2020

"""

from __future__ import annotations

import rclpy
from rclpy.parameter import Parameter

from rclpy.node import Node

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist, TwistStamped, Point
from robominer_msgs.msg import MotorModuleCommand, WhiskerArray, Whisker
from std_msgs.msg import Float64

from nav_msgs.msg import Odometry

import numpy as np
from math import tan, pi
from statistics import mean
from typing import List, Any

from dataclasses import dataclass, field

from simple_pid import PID

import time

from enum import Enum

from dataclasses import dataclass

from queue import PriorityQueue

import math


@dataclass
class Factor:
    name: str
    weight: float
    twist: list


@dataclass(order=True)
class PrioritizedItem:
    priority: int
    item: Any=field(compare=False)
    

class DirectionTwist(Enum):
    MOVE_LEFT       = [ 0,  1,  0]
    MOVE_RIGHT      = [ 0, -1,  0]
    MOVE_FORWARD    = [ 1,  0,  0]
    MOVE_BACKWARD   = [-1,  0,  0]
    TURN_LEFT       = [ 0,  0, -1]
    TURN_RIGHT      = [ 0,  0,  1]


class Direction(Enum):
    LEFT = 'L'
    RIGHT = 'R'
    FORWARD = 'F'
    BACKWARD = 'B'

    def opposite(self) -> Direction:
        if self == Direction.LEFT:
            return Direction.RIGHT
        elif self == Direction.RIGHT:
            return Direction.LEFT
        elif self == Direction.FORWARD:
            return Direction.BACKWARD
        elif self == Direction.BACKWARD:
            return Direction.FORWARD

    
    def move_twist(self) -> list:
        if self == Direction.LEFT:
            return DirectionTwist.MOVE_LEFT.value
        elif self == Direction.RIGHT:
            return DirectionTwist.MOVE_RIGHT.value
        elif self == Direction.FORWARD:
            return DirectionTwist.MOVE_FORWARD.value
        elif self == Direction.BACKWARD:
            return DirectionTwist.MOVE_BACKWARD.value

    def turn_twist(self) -> list:
        if self == Direction.LEFT:
            return DirectionTwist.TURN_LEFT.value
        elif self == Direction.RIGHT:
            return DirectionTwist.TURN_RIGHT.value
        else:
            return [0, 0, 0]

LEFT = Direction.LEFT
RIGHT = Direction.RIGHT
FORWARD = Direction.FORWARD
BACKWARD = Direction.BACKWARD

motors = np.array([
        'front_right',
        'rear_right',
        'rear_left',
        'front_left'])

motors_dict = {
    "front_right": "0",
    "rear_right": "1",
    "rear_left": "2",
    "front_left": "3"
}

WHISKER_ROW_AMOUNT = 10
WHISKERS_PER_ROW_AMOUNT = 8

WHISKER_ROWS = {
    LEFT: 6,
    RIGHT: 7,
    FORWARD: 8,
    BACKWARD: 9
}

WALL_PUSH_AWAY_THRESHOLD = 0.4
TRACKED_WALL_PUSH_IN_THRESHOLD = 0.05
WALL_THRESHOLD_MIDPOINT = (WALL_PUSH_AWAY_THRESHOLD + TRACKED_WALL_PUSH_IN_THRESHOLD) / 2

TRACKED_WALL_DIRECTION: Direction = RIGHT

GRAPH_NODE_SIZE = 0.2

# rows: motor modules {fl, fr, rl, rr}
# cols: strafing direction {F, B, L, R}

class NodePosition:
    x: int
    y: int

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return str(self.x) + ", " + str(self.y)

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other):
        return isinstance(other, NodePosition) and self.x == other.x and self.y == other.y

class GraphNode:
    position: NodePosition
    passable: bool = True

    def __init__(self, position: NodePosition, passable: bool=True):
        self.position = position

    def __str__(self):
        return str(self.position) + ": " + str(self.passable)

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other):
        return isinstance(other, GraphNode) and self.position == other.position and self.x == other.y

class Graph:
    nodes = {}

    def nodes_exists(self, position: NodePosition):
        return position in self.nodes.keys()

    def node_passable(self, position: NodePosition):
        return position not in self.nodes.keys() or self.nodes[position].passable

    def nodes_add(self, position: NodePosition, passable: bool=True):
        if position in self.nodes:
            raise RuntimeError('Node already existed!')

        self.nodes[position] = GraphNode(position, passable)

    def nodes_mark_unpassable(self, position: NodePosition):
        if position in self.nodes:
            self.nodes[position].passable = False
        else:
            self.nodes_add(position, False)

    def neighbors(self, position: NodePosition) -> List[NodePosition]:
        neighbors = []
        x, y = position.x, position.y

        potential_neighbors = [
            NodePosition(x + 1, y),
            NodePosition(x - 1, y),
            NodePosition(x, y + 1),
            NodePosition(x, y - 1)
        ]

        for pos in potential_neighbors:
            if self.node_passable(pos):
                neighbors.append(pos)

        return neighbors

    def cost(self, pos1: NodePosition, pos2: NodePosition):
        return 1


GOAL = NodePosition(-40, 0)


class RM3InverseKinematics(Node):
    def __init__(self):
        super().__init__('rm3_inverse_kinematics')
        self.lx = 0.15                          # m longitudinal distance
        self.ly = 0.3                           # m lateral distance
        self.screw_helix_angle = pi/6.0         # pi/6 for fl and rr screws, -pi/6 for fr and rl
        self.screw_radius = 0.078 	            # m
        self.radpersec_to_rpm = 60.0 / (2*pi)
        self.rpm_to_radpersec = (2*pi) / 60.0
        self.kinematics_timer_period = 0.1      # seconds
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_yaw = 0.0
        self.speed_multiplier = 0.0             # speed multiplier that is applied to the body velocity input
        self.turbo_multiplier = 0.0
        self.has_had_contact = False

        self.curr_node_position = None

        self.position = None
        self.orientation = None

        self.original_angle = None
        self.angle = None

        self.graph = Graph()

        self.path = []

        self.direction = 0
        self.horizontal_movement = 0
        self.x_axis_movement = 0
        self.x_axis_turning = 0

        self.direction_pid = PID(2, 0, 0, 0, output_limits=(-20, 20), auto_mode=False)
        self.horizontal_pid = PID(5, 0, 0, 0.2, output_limits=(-5, 5), auto_mode=False)
        self.x_axis_pid = PID(1, 0, 0, 0, output_limits=(-40, 40), auto_mode=False)
        self.x_axis_turning__pid = PID(40, 0, 0, 0, output_limits=(-40, 40), auto_mode=False)

        self.publisher_error_direction = self.create_publisher(Float64, '/whiskerErrors/direction', 10)
        self.publisher_output_direction = self.create_publisher(Float64, '/whiskerErrors/direction_out', 10)
        self.publisher_error_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis', 10)
        self.publisher_output_y_axis = self.create_publisher(Float64, '/whiskerErrors/y_axis_out', 10)
        self.publisher_error_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis', 10)
        self.publisher_output_x_axis = self.create_publisher(Float64, '/whiskerErrors/x_axis_out', 10)
        # self.publisher_weight = self.create_publisher(Float64, '/whiskerErrors/direction', 10)

        self.prev_movement_log = ""

        self.whisker_pressures_avg = {}
        self.whisker_pressures_max = {}

        self.sub_body_vel = self.create_subscription(
            TwistStamped, '/robot_body_vel', self.inputCallback, 10
        )

        # self.declare_parameter('on_robot')
        # self.on_robot = self.get_parameter('on_robot').value

        # if not self.on_robot:
        #     self.declare_parameter('which_sim')
        #     self.which_sim = self.get_parameter('which_sim').get_parameter_value().string_value
        #     self.get_logger().info(f'which simulator: {self.which_sim}')

        self.platform_kinematics = np.array([
            [-1/tan(self.screw_helix_angle), -1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [-1/tan(self.screw_helix_angle),  1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [1/tan(self.screw_helix_angle),   1, -(self.lx + self.ly / tan(self.screw_helix_angle))],
            [1/tan(self.screw_helix_angle),  -1, -(self.lx + self.ly / tan(self.screw_helix_angle))]])

        # if self.on_robot or self.which_sim=='gazebo':
        self.speed_multiplier = 1.0
            # if self.on_robot:
            #     self.get_logger().info(f'on robot')
            # else:
            #     self.get_logger().info(f'on gazebo, speed multiplier: {self.speed_multiplier}')

            #self.sub_joystick = self.create_subscription(Joy, 'joy', self.joystickCallback, 10)
            # self.sub_keyboard = self.create_subscription(Twist, 'cmd_vel_keyboard', self.keyboardCallback, 10)
        self.publisher_motor0_commands = self.create_publisher(MotorModuleCommand, '/motor0/motor_rpm_setpoint', 10)
        self.publisher_motor1_commands = self.create_publisher(MotorModuleCommand, '/motor1/motor_rpm_setpoint', 10)
        self.publisher_motor2_commands = self.create_publisher(MotorModuleCommand, '/motor2/motor_rpm_setpoint', 10)
        self.publisher_motor3_commands = self.create_publisher(MotorModuleCommand, '/motor3/motor_rpm_setpoint', 10)

        # else:
        #     self.get_logger().info(f'on vortex (probably)')
        #     self.speed_multiplier = 0.2
        #     # self.sub_keyboard = self.create_subscription(Twist, 'cmd_vel', self.keyboardCallback, 10)
        #     self.publisher_motor0_commands = self.create_publisher(Float64, '/motor0/motor_rpm_setpoint', 10)
        #     self.publisher_motor1_commands = self.create_publisher(Float64, '/motor1/motor_rpm_setpoint', 10)
        #     self.publisher_motor2_commands = self.create_publisher(Float64, '/motor2/motor_rpm_setpoint', 10)
        #     self.publisher_motor3_commands = self.create_publisher(Float64, '/motor3/motor_rpm_setpoint', 10)

        self.kinematics_timer = self.create_timer(self.kinematics_timer_period, self.inverseKinematics)

        self.sub_whisker = self.create_subscription(
            WhiskerArray, '/WhiskerStates', self.onWhisker, 100)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom/unfiltered', self.onOdometry, 100)

    def onOdometry(self, msg: Odometry):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

        self.calc_angle()

        self.curr_node_position = self.translate_position_to_graph(self.position)

        if not self.graph.nodes_exists(self.curr_node_position):
            self.graph.nodes_add(self.curr_node_position, passable=True)

    def calc_angle(self):
        angle = math.degrees(self.orientation.z)
        self.angle = angle if angle <= 180 else angle - 360

    def translate_position_to_graph(self, position: Point) -> NodePosition:
        x_temp = int(round(position.x / GRAPH_NODE_SIZE, 0))
        y_temp = int(round(position.y / GRAPH_NODE_SIZE, 0))
        return NodePosition(x_temp, y_temp)

    def add_angles(self, a1, a2):
        angle = (a2 - a1) % 360
        if angle > 180:
            angle -=360
        return angle

    def mark_graph_point_after_collision(self, collision_direction: Direction):
        # self.get_logger().info('Mark collision')
        collision_angle_modification = 0
        if collision_direction == Direction.RIGHT:
            collision_angle_modification = 90
        elif collision_direction == Direction.BACKWARD:
            collision_angle_modification = 180
        elif collision_direction == Direction.BACKWARD:
            collision_angle_modification = 270

        collision_angle = self.add_angles(self.angle, collision_angle_modification)

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

        # self.get_logger().info('Marked unpassable')

        collision_point = NodePosition(x, y)
        if self.graph.node_passable(collision_point):
            # New collision
            self.graph.nodes_mark_unpassable(collision_point)
            # Recalculate due to new position
            self.path = self.a_star(self.curr_node_position, GOAL)


    def onWhisker(self, msg: WhiskerArray):
        """
        Process whisker output for movement input.
        """
        # self.get_logger().info("-----")
        self.log_surroundings()
        # self.get_logger().info('Node amount: ' + str(len(self.graph.nodes)))
        # self.get_logger().info('Angle: ' + str(self.angle))
        self.get_logger().info('Pos: ' + str(self.curr_node_position))

        self.whisker_matrix = create_whisker_matrix(msg.whiskers)

        self.whisker_pressures_avg = {}
        self.whisker_pressures_max = {}
        for direction in Direction:
            self.whisker_pressures_avg[direction] = calc_whisker_pressure_avg(self.whisker_matrix[WHISKER_ROWS[direction]])
            self.whisker_pressures_max[direction] = calc_whisker_pressure_max(self.whisker_matrix[WHISKER_ROWS[direction]])

            if self.whisker_pressures_max[direction] > 0.05:
                self.mark_graph_point_after_collision(direction)

        if not self.has_had_contact and any(p > 0.1 for p in self.whisker_pressures_max.values()):
                self.has_had_contact = True
                self.direction_pid.set_auto_mode(True, self.direction)
                self.horizontal_pid.set_auto_mode(True, self.horizontal_movement)
                self.x_axis_pid.set_auto_mode(True, self.x_axis_movement)
                self.x_axis_turning__pid.set_auto_mode(True, self.x_axis_movement)
            

        self.assignDirectionError()

        self.horizontal_movement = self.calcAxisError(self.horizontal_pid, self.publisher_error_y_axis, TRACKED_WALL_DIRECTION, 0.7, 0.3)
        self.publisher_output_y_axis.publish(Float64(data=float(self.horizontal_movement)))

        self.x_axis_movement = self.calcAxisError(self.x_axis_pid, self.publisher_error_x_axis, FORWARD, 0.3, 0.7)
        self.publisher_output_x_axis.publish(Float64(data=float(self.x_axis_movement)))
        
    def assignDirectionError(self):
        # towards left
        direction = calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[RIGHT]]) - calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[LEFT]])
        direction += 4*(calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[BACKWARD]]) - calc_whiskers_inclination_euclid(self.whisker_matrix[WHISKER_ROWS[FORWARD]]))

        self.publisher_error_direction.publish(Float64(data=float(direction)))
        pid_dir = self.direction_pid(direction)

        if pid_dir is not None:
            self.direction = pid_dir
        else:
            self.direction = direction

        self.publisher_output_direction.publish(Float64(data=float(self.direction)))
    
    def calcAxisError(self, pid, input_publisher, addDirection, avgWeight, maxWeight):
        avg_component = self.whisker_pressures_avg[addDirection] - self.whisker_pressures_avg[addDirection.opposite()]
        max_component = self.whisker_pressures_max[addDirection] - self.whisker_pressures_max[addDirection.opposite()]
        total_movement = avgWeight * avg_component + maxWeight * max_component

        input_publisher.publish(Float64(data=float(total_movement)))

        pid_movement = pid(total_movement)

        return pid_movement if pid_movement is not None else total_movement

    def inputCallback(self, msg: Twist):
        """
        Callback function for the keyboard topic. Parses a keyboard message to body velocities in x, y, and yaw
        @param: self
        @param: msg - Twist message format
        """
        self.cmd_vel_x = msg.twist.linear.x
        self.cmd_vel_y = msg.twist.linear.y
        self.cmd_vel_yaw = msg.twist.angular.z

    # def joystickCallback(self, msg):
    #     """
    #     Callback function for the joystick topic. Parses a joystick message to body velocities in x, y, and yaw
    #     @param: self
    #     @param: msg - Joy message format
    #     """
    #     self.cmd_vel_x = msg.axes[1]
    #     self.cmd_vel_y = msg.axes[0]
    #     self.cmd_vel_yaw = msg.axes[2]
    #     self.turbo_multiplier = (msg.buttons[5] * .01)

    def inverseKinematics(self):
        """
        Solves the inverse kinematic problem to calculate actuator speeds from body velocity.
        Calls to publish the individual motor speeds.
        @param: self
        """
        speed_multiplier = 0
        speed_multiplier += self.speed_multiplier + self.turbo_multiplier

        movement_list = self.determine_movement()

        self.robot_twist = np.array(movement_list) * speed_multiplier

        self.screw_speeds = (1.0/self.screw_radius) * np.dot(self.platform_kinematics, self.robot_twist) * self.radpersec_to_rpm
        self.speedsBroadcast()

    def log_movement(self, new_movement) -> None:
        if new_movement != self.prev_movement_log:
            self.get_logger().info(new_movement)
            self.prev_movement_log = new_movement

    def log_surroundings(self):
        if self.curr_node_position is None:
            return

        pos = self.curr_node_position
        x = pos.x
        y = pos.y

        txt = "\n"
        for i in range(-10, 11):
            for j in range(-10, 11):
                this_pos = NodePosition(x + i, y + j)
                if self.path is not None and this_pos in self.path:
                    txt += 'G'
                    continue

                if not self.graph.nodes_exists(this_pos):
                    txt += 'o'
                    continue

                node: GraphNode = self.graph.nodes[this_pos]
                txt += 'O' if node.passable else 'X'

            txt += "\n"

        self.get_logger().info(txt)
        
    def determine_movement(self) -> List[float]:
        """
        Output: x, y, yaw
        """
        #self.get_logger().info('----------------------')

        if not self.has_had_contact:
            # return [self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw]

            return DirectionTwist.MOVE_RIGHT.value if self.position is not None else [0, 0, 0]


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
            100 if self.whisker_pressures_max[FORWARD] > 0.1 else 0,
            # [0, -0.4, 0] if TRACKED_WALL_DIRECTION == LEFT else [0, 0.4, 0]
            [0, -0.4, -1] if TRACKED_WALL_DIRECTION == LEFT else [0, 0.4, 1]
        ))

        factors.append(Factor(
            'Handle backward pressure',
            100 if self.whisker_pressures_max[BACKWARD] > 0.1 else 0,
            [0, 0.4, 1] if TRACKED_WALL_DIRECTION == LEFT else [0, -0.4, -1]
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

        summed_up_twist = calculate_movement_from_factors(factors)
        
        # self.get_logger().info(str(len(movement_weights)) + ' - ' + str(summed_up_twist))
        ROBOT_SPEED = 1
        return multiply_list_with_scalar(summed_up_twist, ROBOT_SPEED)

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

    def a_star(self, start, goal) -> List[NodePosition]:
        frontier: PriorityQueue[PrioritizedItem] = PriorityQueue()
        frontier.put(PrioritizedItem(0, start))
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get().item

            if current == goal:
                break
            
            for next in self.graph.neighbors(current):
                new_cost = cost_so_far[current] + self.graph.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    frontier.put(PrioritizedItem(priority, next))
                    came_from[next] = current

        if goal not in came_from:
            return []

        path = [goal]
        while path[-1] != start and path[-1] != None:
            path.append(came_from[path[-1]])

        return list(reversed(path))[1:]  # Exclude start from path


    def speedsBroadcast(self):
        '''
        Publishes the results of inverse kinematics to 4 topics, one RPM setpoint for each screw.
        @param: self
        '''
        # if self.on_robot or self.which_sim=='gazebo':
        self.motor_cmd = [MotorModuleCommand() for i in range(4)]
        for m in range(4):
            self.motor_cmd[m].header.stamp = self.get_clock().now().to_msg()
            self.motor_cmd[m].header.frame_id = motors[m]
            self.motor_cmd[m].motor_id = motors_dict[motors[m]]
            self.motor_cmd[m].motor_rpm_goal = int(self.screw_speeds[m])
        # else:
        #     self.motor_cmd = [Float64() for i in range(4)]
        #     for m in range(4):
        #         self.motor_cmd[m].data = self.screw_speeds[m]

        self.publisher_motor0_commands.publish(self.motor_cmd[0])
        self.publisher_motor1_commands.publish(self.motor_cmd[1])
        self.publisher_motor2_commands.publish(self.motor_cmd[2])
        self.publisher_motor3_commands.publish(self.motor_cmd[3])


def create_whisker_matrix(whiskers: List[Whisker]):
    whisker_matrix = [[0 for _ in range(WHISKERS_PER_ROW_AMOUNT)] for _ in range(WHISKER_ROW_AMOUNT)]

    for whisker in whiskers:
        whisker_matrix[whisker.pos.row_num][whisker.pos.col_num] = whisker

    return whisker_matrix


def whisker_multiplier(col_num: int):
    """
    Convert 4-7 to 1..4 and 0-3 to -4..-1 for finding appropriate turning radius
    Returns: Value from -4..4 excluding 0
    """
    if col_num < 4:
        return col_num - 4
    
    return col_num - 3


def calc_whiskers_inclination_x(whiskers : List[Whisker]):
    """
    Calculate an inclination for a whisker array, taking into account the position of the whiskers.
    Positive if higher columns have higher values, negative if lower columns have higher values.
    """
    return sum([abs(w.x) * whisker_multiplier(w.pos.col_num) for w in whiskers])


def calc_whiskers_inclination_y(whiskers : List[Whisker]):
    """
    Calculate an inclination for a whisker array, taking into account the position of the whiskers.
    Positive if higher columns have higher values, negative if lower columns have higher values.
    """
    return sum([abs(w.y) * whisker_multiplier(w.pos.col_num) for w in whiskers])


def calc_whiskers_inclination_euclid(whiskers : List[Whisker]):
    """
    Calculate an inclination for a whisker array, taking into account the position of the whiskers.
    Positive if higher columns have higher values, negative if lower columns have higher values.
    Max possible value is 31.4, should be capped at ~5.
    """
    return sum([whisker_euclid_dist(w) * whisker_multiplier(w.pos.col_num) for w in whiskers])


def calc_whisker_pressure_max(whiskers: List[Whisker]):
    """
    Get the maximum pressure of the whisker with most pressure in the list.
    """
    return max([whisker_euclid_dist(w) for w in whiskers])


def calc_whisker_pressure_avg(whiskers: List[Whisker]):
    """
    Get the average pressure of all whiskers in the list.
    """
    return mean([whisker_euclid_dist(w) for w in whiskers])


def whisker_euclid_dist(whisker: Whisker):
    """
    Calculate the euclidean distance of the whisker's x and y displacement.
    """
    return (whisker.x**2 + whisker.y**2)**0.5


def multiply_list_with_scalar(_list: List, num) -> List:
    return [i * num for i in _list]

def sum_up_lists(lists: List[List]) -> List:
    """Out of place summing up"""
    max_len = len(max(lists, key=lambda _list: len(_list)))
    out_list = [0] * max_len

    for _list in lists:
        for i in range(len(_list)):
            out_list[i] += _list[i]

    return out_list


def normalize_weights(_list: List) -> None:
    list_sum = sum(map(abs, _list))
    for i in range(len(_list)):
        _list[i] /= list_sum


def normalize_factor_weights(factors: List[Factor]) -> None:
    """
    Normalize factor weights to sum up to 1.
    """
    total_sum = sum(factor.weight for factor in factors)
    for factor in factors:
        factor.weight /= total_sum

def map_linear_val(val, min_in, max_in, min_out, max_out):
    val = np.clip(val, min_in, max_in)
    linear_result = (max_out - min_out) * ( (val - min_in) / (max_in - min_in) ) + min_out

    return np.clip(linear_result, min_out, max_out)


def map_linear_val_min_threshold(val, min_in, max_in, min_out, max_out, min_threshold, min_threshold_return_val=0):
    if (val < min_in + min_threshold):
        return min_threshold_return_val
    return map_linear_val(val, min_in, max_in, min_out, max_out)


def calculate_movement_from_factors(factors: List[Factor]):
    normalize_factor_weights(factors)

    out_list = [0] * 3
    for factor in factors:
        factor.twist = multiply_list_with_scalar(factor.twist, factor.weight)
        for i in range(3):
            out_list[i] += factor.twist[i]

    return out_list


def heuristic(goal: NodePosition, next: NodePosition):
    return ((goal.x - next.x)**2 + (goal.y - next.y)**2)**0.5


def main(args=None):
    rclpy.init(args=args)

    rm3_inverse_kinematics = RM3InverseKinematics()

    rclpy.spin(rm3_inverse_kinematics)

    rm3_inverse_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
