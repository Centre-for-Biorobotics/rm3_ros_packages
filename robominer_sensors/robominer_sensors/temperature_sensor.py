#!/usr/bin/python3
"""
Reads the MCP9808 temperature sensor
Current version reads it from a file in Olimex Ubuntu Linux filesystem

@author: Jaan Rebane
@contact: jaan.rebane@ttu.ee
@creation date: 2021-03-24

"""

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Temperature

path = '/sys/class/hwmon/hwmon6/temp1_input'

class TemperatureSensor(Node):
	def __init__(self):
		super().__init__('temperature_sensor')
		self.temperature = 0
		self.publisher_temperature = self.create_publisher(Temperature, '/temperature_sensor/temperature', 10)
		timer_period = 1.0  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def timer_callback(self):
		msg = Temperature()
		try:
			tempfile = open(path,'r')
			tempValues = tempfile.readlines()
			tempValue = int(str(tempValues[0]))
			tempfile.close()

			msg.temperature = tempValue/1000.0
			self.publisher_temperature.publish(msg)
			self.get_logger().info('Publishing. Raw value: %s' % tempValue)

		except OSError:
			self.get_logger().error('Temperature data file not found!')
			# If we would like to destroy the node:
			# temperature_sensor.destroy_node()
			# rclpy.shutdown()
			# But we don't, we try again.

def main(args=None):
	rclpy.init(args=args)
	temperature_sensor = TemperatureSensor()

	rclpy.spin(temperature_sensor)

	temperature_sensor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
