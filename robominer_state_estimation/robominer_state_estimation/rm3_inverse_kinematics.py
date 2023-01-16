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

from geometry_msgs.msg import Twist, TwistStamped
from robominer_msgs.msg import MotorModuleCommand, WhiskerArray, Whisker
from std_msgs.msg import Float64

import numpy as np
from math import tan, pi
from statistics import mean
from typing import List, Tuple

from enum import Enum

class DirectionTwist(Enum):
    LEFT_TWIST     = [ 0,  1, 0]
    RIGHT_TWIST    = [ 0, -1, 0]
    FORWARD_TWIST  = [ 1,  0, 0]
    BACKWARD_TWIST = [-1,  0, 0]


class Direction(Enum):
    LEFT = 'L'
    RIGHT = 'R'
    FORWARD = 'F'
    BACKWARD = 'B'

    @classmethod
    def opposite(cls, dir: Direction) -> Direction:
        if dir == Direction.LEFT:
            return Direction.RIGHT
        elif dir == Direction.RIGHT:
            return Direction.LEFT
        elif dir == Direction.FORWARD:
            return Direction.BACKWARD
        elif dir == Direction.BACKWARD:
            return Direction.FORWARD

    @classmethod
    def twist(cls, dir: Direction) -> list:
        if dir == Direction.LEFT:
            return DirectionTwist.LEFT_TWIST.value
        elif dir == Direction.RIGHT:
            return DirectionTwist.RIGHT_TWIST.value
        elif dir == Direction.FORWARD:
            return DirectionTwist.FORWARD_TWIST.value
        elif dir == Direction.BACKWARD:
            return DirectionTwist.BACKWARD_TWIST.value

"""
class Direction(NamedDirection, Enum):
    LEFT     = NamedDirection(short='L', twist=[ 0,  1, 0])
    RIGHT    = NamedDirection(short='R', twist=[ 0, -1, 0])
    FORWARD  = NamedDirection(short='F', twist=[ 1,  0, 0])
    BACKWARD = NamedDirection(short='B', twist=[-1,  0, 0])

    @classmethod
    def opposite(dir: Direction) -> Direction:
        if dir == Direction.LEFT:
            return Direction.RIGHT
        elif dir == Direction.RIGHT:
            return Direction.LEFT
        elif dir == Direction.FORWARD:
            return Direction.BACKWARD
        elif dir == Direction.BACKWARD:
            return Direction.FORWARD
"""




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

#WHISKER_ROW_NUM_LEFT = 6
#WHISKER_ROW_NUM_RIGHT = 7
#WHISKER_ROW_NUM_FRONT = 8
#WHISKER_ROW_NUM_BACK = 9


WALL_PUSH_AWAY_THRESHOLD = 0.4
LEFT_WALL_PUSH_IN_THRESHOLD = 0.05
WALL_THRESHOLD_MIDPOINT = (WALL_PUSH_AWAY_THRESHOLD + LEFT_WALL_PUSH_IN_THRESHOLD) / 2

# rows: motor modules {fl, fr, rl, rr}
# cols: strafing direction {F, B, L, R}

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
        self.direction = 0

        self.prev_movement_log = ""

        self.whisker_pressures_avg = {}
        self.whisker_pressures_max = {}

        self.curr_push_direction = None
        self.curr_push_threshold_dir = None
        self.curr_push_threshold_dir = 0.0

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

    def onWhisker(self, msg: WhiskerArray):
        """
        Process whisker output for movement input.
        """
        whisker_matrix = create_whisker_matrix(msg.whiskers)

        self.whisker_pressures_avg = {}
        self.whisker_pressures_max = {}
        for direction in Direction:
            self.whisker_pressures_avg[direction] = calc_whisker_pressure_avg(whisker_matrix[WHISKER_ROWS[direction]])
            self.whisker_pressures_max[direction] = calc_whisker_pressure_max(whisker_matrix[WHISKER_ROWS[direction]])

        #if self.whisker_pressure_front > 0.2:
        #    self.direction = -0.5
        #else:
        tmp_direction = calc_whiskers_inclination_euclid(whisker_matrix[WHISKER_ROWS[RIGHT]]) - calc_whiskers_inclination_euclid(whisker_matrix[WHISKER_ROWS[LEFT]])
        self.direction = np.clip(tmp_direction / 2, -1, 1)

        if any(p > 0.1 for p in self.whisker_pressures_max.values()):
            self.has_had_contact = True

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
        
    def determine_movement(self) -> List[float]:
        """
        Output: x, y, yaw
        """
        HARD_COLLISION_AVG_THRESHOLD = 0.7
        HARD_COLLISION_MAX_THRESHOLD = 0.9

        DODGE_SPEED = 0.3

        if not self.has_had_contact:
            self.log_movement('Keyboard movement')
            # Until contact don't override movement
            return [self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw]

        hard_collision_reverse_system_twist = self.hard_collision_reverse_system(HARD_COLLISION_AVG_THRESHOLD, HARD_COLLISION_MAX_THRESHOLD, DODGE_SPEED)
        if hard_collision_reverse_system_twist != None:
            self.log_movement('Hard collision avoidance')
            return hard_collision_reverse_system_twist

        if self.curr_push_direction == LEFT:
            if self.whisker_pressures_max[LEFT] < WALL_THRESHOLD_MIDPOINT:
                #  Move towards left wall if no contact with the left wall until midpoint
                self.log_movement('To threshold L')
                return multiply_list_with_scalar(DirectionTwist.LEFT_TWIST.value, DODGE_SPEED)
            else:
                self.curr_push_direction = None

        if self.curr_push_direction == RIGHT:
            if self.whisker_pressures_max[RIGHT] < WALL_THRESHOLD_MIDPOINT:
                #  Move towards left wall if no contact with the left wall until midpoint
                self.log_movement('To threshold R')
                return multiply_list_with_scalar(DirectionTwist.RIGHT_TWIST.value, DODGE_SPEED)
            else:
                self.curr_push_direction = None

        if self.direction > 0.75:
            #  If any change in yaw is needed to align with the wall, then do that
            self.log_movement('Direction high threshold')
            return [0, 0, self.direction]
        
        if self.whisker_pressures_max[FORWARD] > 0.02:
            self.log_movement('Front')
            #  Move away from the wall first if necessary and rotate right
            return [0, -0.2, -0.6]

        if self.whisker_pressures_max[BACKWARD] > 0.02:
            self.log_movement('Back')
            #  Move away from the wall first if necessary and rotate right
            return [0, -0.2, 0.6]

        if self.whisker_pressures_max[LEFT] > WALL_PUSH_AWAY_THRESHOLD and self.whisker_pressures_max[LEFT] >= self.whisker_pressures_max[RIGHT]:
            self.log_movement('Push away left')
            y = -np.clip(self.whisker_pressures_max[LEFT] - WALL_PUSH_AWAY_THRESHOLD + 0.3, 0, 0.6)
            return [0, y, 0]

        if self.whisker_pressures_max[RIGHT] > WALL_PUSH_AWAY_THRESHOLD:
            self.log_movement('push away right')
            y = np.clip(self.whisker_pressures_max[RIGHT] - WALL_PUSH_AWAY_THRESHOLD + 0.3, 0, 0.6)
            return [0, y, 0]

        if self.whisker_pressures_max[LEFT] < LEFT_WALL_PUSH_IN_THRESHOLD and self.whisker_pressures_max[FORWARD] < 0.0001:
            self.log_movement('move toward left')
            #  Move towards left wall if no contact with the left wall until midpoint
            self.curr_push_direction = LEFT

            return multiply_list_with_scalar(DirectionTwist.LEFT_TWIST.value, DODGE_SPEED)

        self.log_movement('Direction default')
        return [self.cmd_vel_x, self.cmd_vel_y, self.direction]

    def hard_collision_reverse_system(self, hard_collision_avg_threshold, hard_collision_max_threshold, dodge_speed):
        """
        Check if robot is near collision and return an appropriate twist
        """
        max_avg_pressure_direction = max(self.whisker_pressures_avg, key=self.whisker_pressures_avg.get)
        max_max_pressure_direction = max(self.whisker_pressures_max, key=self.whisker_pressures_max.get)

        if self.whisker_pressures_avg[max_avg_pressure_direction] > hard_collision_avg_threshold:
            moveDirection = Direction.opposite(max_avg_pressure_direction)
        elif self.whisker_pressures_max[max_max_pressure_direction] > hard_collision_max_threshold:
            moveDirection = Direction.opposite(max_max_pressure_direction)
        else:
            return None

        return multiply_list_with_scalar(Direction.twist(moveDirection), dodge_speed)

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


def main(args=None):
    rclpy.init(args=args)

    rm3_inverse_kinematics = RM3InverseKinematics()

    rclpy.spin(rm3_inverse_kinematics)

    rm3_inverse_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
