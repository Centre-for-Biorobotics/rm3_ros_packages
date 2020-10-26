#!/usr/bin/python3
""" 

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@creation date: 29-08-2020 (started)

"""

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist
from robominer_msgs.msg import MotorModuleCommand
# from robominer_msgs.msg import AllMotorModuleCommand

import numpy as np

# motors = np.array(['front_right',
# 			'rear_right',
# 			'rear_left',
# 			'front_left'])


# rows: motor modules {fl, fr, rl, rr}
# cols: strafing direction {F, B, L, R}			
# motor_multiplier = np.array([[1,-1,-1,1],
# 							[-1, 1, -1, 1],
# 							[1, -1, 1, -1],
# 							[-1, 1, 1, -1] ])


class OpenLoopSteering(Node):
	def __init__(self):
		super().__init__('open_loop_steering')
		self.lx = 100 			# mm longitudinal distance
		self.ly = 300 			# mm lateral distance
		self.screw_radius = 55 	# mm

		# fr rr rl fl
		self.platform_kinematics = np.array([
			[-1,  1,  (self.lx + self.ly)],
            [-1, -1,  (self.lx + self.ly)],
            [ 1, -1, -(self.lx + self.ly)],
            [ 1,  1, -(self.lx + self.ly)]])
		
		self.sub_joystick = self.create_subscription(Joy, 'joy', self.joystickCallback, 10)
		self.publisher_motor0_commands = self.create_publisher(MotorModuleCommand, '/motor0/motor_rpm_setpoint', 10)
		self.publisher_motor1_commands = self.create_publisher(MotorModuleCommand, '/motor1/motor_rpm_setpoint', 10)


	def joystickCallback(self, msg):
		# self.get_logger().info('joy data (x)): "%d"' %msg.buttons[2])
		# self.get_logger().info('joy data (fb)): "%f"' %msg.axes[1])
		# self.get_logger().info('joy data (rl)): "%f"' %msg.axes[0])
		# self.get_logger().info('joy data (yaw)): "%f"' %msg.axes[2])

		self.cmd_vel_x = msg.axes[1]
		self.cmd_vel_y = msg.axes[0]
		self.cmd_vel_yaw = msg.axes[2]
		self.robot_twist = [self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw]

		self.screw_speeds = 10 * 1/self.screw_radius * np.dot(self.platform_kinematics, self.robot_twist)
		
		# self.get_logger().info('x: "%f", y: "%f", yaw: "%f"' %( self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw))

		# self.get_logger().info('fr: "%f", rr: "%f", rl: "%f", fl: "%f"' %( self.screw_speeds[0], self.screw_speeds[1], self.screw_speeds[2], self.screw_speeds[3]))
		for motor in range(4):
			self.motor_cmd = MotorModuleCommand()

			self.motor_cmd.motor_rpm_goal = self.screw_speeds[motor]
			# self.motor_cmds.rr_motor_rpm_goal = self.screw_speeds[1]
			# self.motor_cmds.rl_motor_rpm_goal = self.screw_speeds[2]
			# self.motor_cmds.fl_motor_rpm_goal = self.screw_speeds[3]
			self.publisher_motor0_commands.publish(self.motor_cmd)

	def inverseKinematics(self, x, y, yaw):
		return


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