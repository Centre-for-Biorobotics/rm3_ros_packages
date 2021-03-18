#!/usr/bin/python3
""" 
Takes robot body velocities as input from joystick. 
Calculates inverse kinematics to determine screw velocities.
Publishes screw velocities

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

motors = np.array(['front_right',
			'rear_right',
			'rear_left',
			'front_left'])

motors_dict = {
	"front_right": "0",
	"rear_right": "1",
	"rear_left": "2",
	"front_left": "3"
}

# rows: motor modules {fl, fr, rl, rr}
# cols: strafing direction {F, B, L, R}			
# motor_multiplier = np.array([[1,-1,-1,1],
# 							[-1, 1, -1, 1],
# 							[1, -1, 1, -1],
# 							[-1, 1, 1, -1] ])


class OpenLoopSteering(Node):
	def __init__(self):
		super().__init__('open_loop_steering')
		self.lx = 0.1 			# m longitudinal distance
		self.ly = 0.3 			# m lateral distance
		self.screw_radius = 0.055 	# m
		self.kinematics_timer_period = 0.1 # seconds
		self.cmd_vel_x = 0
		self.cmd_vel_y = 0
		self.cmd_vel_yaw = 0


		# fr rr rl fl
		self.platform_kinematics = np.array([
			[ 1,  1, -(self.lx + self.ly)*2],
            		[-1,  1, -(self.lx + self.ly)*2],
            		[-1, -1, -(self.lx + self.ly)*2],
			[ 1, -1, -(self.lx + self.ly)*2]])
		
		self.sub_joystick = self.create_subscription(Joy, 'joy', self.joystickCallback, 10)
		self.publisher_motor0_commands = self.create_publisher(MotorModuleCommand, '/motor0/motor_rpm_setpoint', 10)
		self.publisher_motor1_commands = self.create_publisher(MotorModuleCommand, '/motor1/motor_rpm_setpoint', 10)
		self.publisher_motor2_commands = self.create_publisher(MotorModuleCommand, '/motor2/motor_rpm_setpoint', 10)
		self.publisher_motor3_commands = self.create_publisher(MotorModuleCommand, '/motor3/motor_rpm_setpoint', 10)
		self.kinematics_timer = self.create_timer(self.kinematics_timer_period, self.inverseKinematics)


	def joystickCallback(self, msg):
		# self.get_logger().info('joy data (x)): "%d"' %msg.buttons[2])
		# self.get_logger().info('joy data (fb)): "%f"' %msg.axes[1])
		# self.get_logger().info('joy data (rl)): "%f"' %msg.axes[0])
		# self.get_logger().info('joy data (yaw)): "%f"' %msg.axes[2])

		self.cmd_vel_x = msg.axes[1]
		self.cmd_vel_y = msg.axes[0]
		self.cmd_vel_yaw = msg.axes[2]


	def inverseKinematics(self):
		self.robot_twist = [self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw]

		self.screw_speeds = 1/self.screw_radius * np.dot(self.platform_kinematics, self.robot_twist)
		
		# self.get_logger().info('x: "%f", y: "%f", yaw: "%f"' %( self.cmd_vel_x, self.cmd_vel_y, self.cmd_vel_yaw))
		self.get_logger().info('fr: "%f", rr: "%f", rl: "%f", fl: "%f"' %( self.screw_speeds[0], self.screw_speeds[1], self.screw_speeds[2], self.screw_speeds[3]))
		
		self.speedsBroadcast()


	def speedsBroadcast(self):
		'''
		Publishes the results of inv.kin to 4 topics, one for each screw
		'''
		self.motor_cmd = [MotorModuleCommand() for i in range (4)]
		# self.get_logger().info(str(self.motor_cmd))
		for m in range(4):
			self.motor_cmd[m].motor_id = motors_dict[motors[m]]
			self.motor_cmd[m].motor_rpm_goal = int(self.screw_speeds[m])
			# self.get_logger().info(str(self.motor_cmd[m]))
		self.publisher_motor0_commands.publish(self.motor_cmd[0])
		self.publisher_motor1_commands.publish(self.motor_cmd[1])
		self.publisher_motor2_commands.publish(self.motor_cmd[2])
		self.publisher_motor3_commands.publish(self.motor_cmd[3])







def main(args=None):
	rclpy.init(args=args)
	open_loop_steering = OpenLoopSteering()

	rclpy.spin(open_loop_steering)


	open_loop_steering.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
