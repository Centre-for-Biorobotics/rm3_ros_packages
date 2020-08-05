#!/usr/bin/python3
""" Serial interface between Olimex sbc and arduino nano

Handles bidirectional communications between two devices.

"""


import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String

import serial
import sys
import struct

first_byte = 3
second_byte = 6

# port = "/dev/ttyACM0"


class SerialInterface(Node):
	def __init__(self):
		super().__init__('serial_interface')
		self.declare_parameter('port')

		port = self.get_parameter('port').value
		self.motorID = int(port[11])
		self.get_logger().info( 'Port: %s, Motor ID: %d' %( str(port), self.motorID))

		
		self.ser = serial.Serial(port, 115200, timeout = 0)
		self.publisher_ = self.create_publisher(String, 'received_data', 10)

		timer_period = 1.0 # seconds

		self.timer = self.create_timer(timer_period, self.sendToArduino)
		self.timer2 = self.create_timer(timer_period, self.readFromArduino)


	def readFromArduino(self):
		# if len(sys.argv) < 2:
		# 	print("wrong number of arguments to serial_interface")
		# 	print(sys.argv)
		# 	return
		# while not rospy.is_shutdown():
		msg = String()
		data = self.ser.read(999)
		self.get_logger().info('Received: "%s"' % data)
		# print(str(sys.argv[1]))

		msg.data = str(data)
		self.publisher_.publish(msg)



	def sendToArduino(self):

		outbuffer = 'sync'.encode('UTF-8')

		outbuffer += struct.pack('b', self.motorID)
		outbuffer += struct.pack('b', second_byte)

		outbuffer += '\n'.encode('UTF-8')

		self.get_logger().info('Sending: "%s"' % outbuffer)

		self.ser.write(outbuffer) # writes to serial port





def main(args=None):
	rclpy.init(args=args)
	serial_interface = SerialInterface()

	rclpy.spin(serial_interface)


	# Destroy the node expilicitly 
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	serial_interface.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

