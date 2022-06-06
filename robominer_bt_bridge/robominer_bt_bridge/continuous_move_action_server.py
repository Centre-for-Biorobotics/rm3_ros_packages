#!/usr/bin/env python3
"""
@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 06-06-2022
"""
import rclpy
import time

from geometry_msgs.msg import TwistStamped

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from robominer_msgs.action import Move
from robominer_msgs.msg import MotorModuleCommand
from nav_msgs.msg import Odometry


class ContinuousMoveActionServer(Node):

    def __init__(self):
        """Handles a request for planar motion as [x, y, yaw] velocity vector.
        Each request is valid for 10 consecutive seconds of motion. After that, 
        the action returns a SUCCESS result.
        """
        super().__init__('continuous_move_action_server')

        self.goal = Move.Goal()

        self.is_moving = False          # to action feedback
        self.target = [.0, .0, .0]      # to forward action goal to low-level 

        self.result_ = False
        self.res_success_counter = 0

        # Initialise publisher 
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'move_cmd_vel', 10)

        # publisher will run on a timer, independent of the action
        self.pub_timer_period = 0.1
        self.pub_timer = self.create_timer(self.pub_timer_period, self.pub_callback)

        # Initialise subscriber: the odometry message is used to determine if the robot has moved
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom/unfiltered',
            self.odom_callback,
            10)

        # Initialise action server
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            execute_callback = self.execute_callback,
            callback_group = ReentrantCallbackGroup(),
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback)

        # set up moving/not moving parameters
        self.position_prev = [0.0, 0.0]
        self.get_logger().info("Move action server initialised.")


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """
        client request is accepted if it is over a threshold 
        (can be changed to 0.0, there's better ways to do this)
        """

        self.goal = goal_request
        
        vel_cmd = self.goal.setpoint_velocity[0] + self.goal.setpoint_velocity[1] + self.goal.setpoint_velocity[2]

        if abs(vel_cmd) >= 0.1:
            # self.get_logger().info(f'goal accepted: {self.goal.setpoint_velocity}')
            self.result_ = False
            return GoalResponse.ACCEPT
        else:
            # self.get_logger().info(f'goal rejected: {self.goal.setpoint_velocity}')
            self.target = [.0, .0, .0]
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        # no way to cancel in this particular case
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        feedback_msg = Move.Feedback()          # bool feedback: moving or not
        self.res_success_counter = 0

        # get target velocity vector from action
        self.target = goal_handle.request.setpoint_velocity

        # handle cancelation
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return Move.Result()





        # the move action request is valid for 10 seconds
        while self.res_success_counter <= 10:           # 10 seconds at 1hz rate

            # handle feedback
            feedback_msg.moving = self.is_moving
            goal_handle.publish_feedback(feedback_msg)
            
            # handle result
            if self.is_moving:
                # self.get_logger().info(f'res counter: {self.res_success_counter}')
                self.res_success_counter += 1
            else:
                self.get_logger().info(f'res counter reset')
                self.res_success_counter = 0

            # Process rate
            time.sleep(1)  # unit: s


        # after 10 seconds of motion have passed, successful result is returned

        goal_handle.succeed()
        result = Move.Result()
        result.moved = True
        return result

    def pub_callback(self):
        # publish velocity setpoint vector to kinematics node
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = self.target[0]
        twist_msg.twist.linear.y = self.target[1]
        twist_msg.twist.angular.z = self.target[2]
        self.cmd_vel_pub.publish(twist_msg)

    def odom_callback(self, msg):
        """
        Determine if robot is moving by checking the odometry message
        """
        if msg.pose.pose.position.x != self.position_prev[0] or msg.pose.pose.position.y != self.position_prev[1]:
            self.is_moving = True
        else:
            self.is_moving = False

        self.position_prev = [msg.pose.pose.position.x, msg.pose.pose.position.y]


def main(args=None):
    rclpy.init(args=args)

    continuous_move_action_server = ContinuousMoveActionServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(continuous_move_action_server, executor=executor)

    continuous_move_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()