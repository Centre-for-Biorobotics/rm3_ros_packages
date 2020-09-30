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



class OpenLoopSteering(Node):
	def __init__(self):
		super().__init__('open_loop_steering')
		self.sub_joystick = self.create_subscription(Twist, 'cmd_vel', self.joystickCallback, 10)
		self.publisher_motor_commands = self.create_publisher(MotorModule, 'motor_module', 10)


	def joystickCallback(self, msg):
		self.get_logger().info('twist data: "%d"' %msg.linear.x )

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