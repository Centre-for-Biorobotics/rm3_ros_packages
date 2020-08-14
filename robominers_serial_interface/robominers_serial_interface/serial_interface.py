#!/usr/bin/python3
""" Serial interface between Olimex sbc and arduino nano

Handles bidirectional communications between the two devices.

"""


import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String

import serial
import sys
import struct

RPM_goal = 30
rpm_est = 0
# second_byte = 6

# port = "/dev/ttyACM0"


class SerialInterface(Node):
	def __init__(self):
		super().__init__('serial_interface')
		self.declare_parameter('port')

		port = self.get_parameter('port').value
		self.motor_ID = int(port[11])
		self.get_logger().info( 'Port: %s, Motor ID: %d' %( str(port), self.motor_ID))

		
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
		packet = self.ser.read(999)

		# compute checksum
		# checksum = 0
		if len(packet) >3:
			rpm_est = struct.unpack('f', packet[5:9])[0]
			self.get_logger().info('Received estimated RPM: "%f"' % rpm_est)
			self.get_logger().info('Received: motor ID: %d, RPM: %d' % (packet[4], packet[5]))
		# for data in packet[4:6]: 						# <---- set range of checksum 
		# 	checksum ^= data(data)

		# self.get_logger().info('Received: "%s"' % packet)
		# self.get_logger().info('Checksum: "%d"' % checksum)
		# print(str(sys.argv[1]))

		msg.data = str(packet)
		self.publisher_.publish(msg)
		self.get_logger().info('------------------------')



	def sendToArduino(self):

		outbuffer = 'sync'.encode('UTF-8')

		outbuffer += struct.pack('b', self.motor_ID)
		outbuffer += struct.pack('b', RPM_goal)

		outbuffer += '\n'.encode('UTF-8')

		# self.get_logger().info('Sending: "%s"' % outbuffer)
		self.get_logger().info( 'Sending: Motor ID: %d, RPM_goal: %d' %(self.motor_ID, RPM_goal))
		

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

