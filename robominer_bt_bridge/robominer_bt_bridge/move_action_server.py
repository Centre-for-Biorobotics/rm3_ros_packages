#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import TwistStamped

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from robominer_msgs.action import Move
from robominer_msgs.msg import MotorModuleCommand


class MoveActionServer(Node):

    def __init__(self):
        super().__init__('move_action_server')

        self.goal = Move.Goal()

        self.is_moving = False          # to action feedback
        self.target = [.0, .0, .0]      # to forward action goal to low-level 

        self.result_ = False

        # Initialise publisher 
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'move_cmd_vel', 10)

        # publisher will run on a timer, independent of the action
        self.pub_timer_period = 0.1
        self.pub_timer = self.create_timer(self.pub_timer_period, self.pub_callback)

        # Initialise subscriber
        self.is_moving_sub = self.create_subscription(
            MotorModuleCommand,
            'motor0/motor_rpm_setpoint',
            self.is_moving_callback,
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

        # get target velocity vector from action
        self.target = goal_handle.request.setpoint_velocity

        # Execute the action:

        # handle cancelation
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return Move.Result()

        # handle feedback
        feedback_msg.moving = self.is_moving
        goal_handle.publish_feedback(feedback_msg)

        # handle result
        goal_handle.succeed()
        result = Move.Result()
        # result is successful if the robot moves at any point
        # this makes more sense if the task is long-term
        if self.is_moving:
            result.moved = True
        else:
            result.moved = False

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
    
    def is_moving_callback(self, msg):
        """
        Determine if robot is moving by checking if a motor got an RPM setpoint.
        Definitlely not the right way to do this.
        """
        if msg.motor_rpm_goal != 0:
            self.is_moving = True
        else:
            self.is_moving = False


def main(args=None):
    rclpy.init(args=args)

    move_action_server = MoveActionServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(move_action_server, executor=executor)

    move_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()