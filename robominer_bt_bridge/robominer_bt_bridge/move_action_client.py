#!/usr/bin/python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from robominer_msgs.action import Move


class MoveActionClient(Node):

    def __init__(self):
        super().__init__('move_action_client')
        self._action_client = ActionClient(self, Move, 'move')

        self.sub_joystick = self.create_subscription(Joy, 'joy', self.joystickCallback, 10)
        self.sub_keyboard = self.create_subscription(Twist, 'cmd_vel', self.keyboardCallback, 10)


    def joystickCallback(self, msg):
        
        self.cmd_vel_x = msg.axes[1]
        self.cmd_vel_y = msg.axes[0]
        self.cmd_vel_yaw = msg.axes[3]
        self.turbo_multiplier = (msg.buttons[5] * .01)

        self.send_goal(self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw)
	
    def keyboardCallback(self, msg):
        self.cmd_vel_x = msg.linear.x
        self.cmd_vel_y = msg.linear.y
        self.cmd_vel_yaw = msg.angular.z

        self.send_goal(self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw)

    def send_goal(self, vel_x, vel_y, vel_yaw):
        # self.get_logger().info(f'sending...')
        goal_msg = Move.Goal()
        # self.get_logger().info(f'goal type: {len(goal_msg.setpoint_velocity)}')
        goal_msg.setpoint_velocity[0] = vel_x
        goal_msg.setpoint_velocity[1] = vel_y
        goal_msg.setpoint_velocity[2] = vel_yaw

        self._action_client.wait_for_server()
        # self.get_logger().info(f'sending goal: {self.cmd_vel_x}')

        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    move_action_client = MoveActionClient()

    # future = move_action_client.send_goal(10)

    # rclpy.spin_until_future_complete(move_action_client, future)
    rclpy.spin(move_action_client)

    move_action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()