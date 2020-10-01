#!/usr/bin/python3
""" 

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@creation date: 29-08-2020 (started)

"""

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from robominer_msgs.msg import MotorModuleCommand
# from robominer_msgs.msg import AllMotorModuleCommand

import numpy as np

motors = np.array(['front_right',
			'rear_right',
			'rear_left',
			'front_left'])

# rows: motor modules {fl, fr, rl, rr}
# cols: strafing direction {F, B, L, R}			
motor_multiplier = np.array([[1,-1,-1,1],
					[-1, 1, -1, 1],
					[1, -1, 1, -1],
					[-1, 1, 1, -1] ])

class OpenLoopSteering(Node):
	def __init__(self):
		super().__init__('open_loop_steering')
		self.sub_joystick = self.create_subscription(Twist, 'cmd_vel', self.joystickCallback, 10)
		self.publisher_motor_commands = self.create_publisher(MotorModuleCommand, 'motor_module_cmd', 10)


	def joystickCallback(self, msg):
		self.get_logger().info('twist data: "%d"' %msg.linear.x )

		self.motor_cmd = MotorModuleCommand()
		for m in motors:
			self.motor_cmd.motor_id = motors[m]
			self.motor_cmd.motor_rpm_goal = msg.linear.x * motor_multiplier[:, 0] + msg.linear.x * motor_multiplier[:, 1] + msg.linear.y * motor_multiplier[:, 2] + msg.linear.y * motor_multiplier[:, 3]
			self.publisher_motor_commands.publish(self.motor_cmd)



	def speedsBroadcast(self):
		return







def main(args=None):
	rclpy.init(args=args)
	open_loop_steering = OpenLoopSteering()

	rclpy.spin(open_loop_steering)


	open_loop_steering.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()