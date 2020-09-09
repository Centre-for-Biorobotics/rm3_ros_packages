#!/usr/bin/python3
""" Serial interface between Olimex sbc and arduino nano

Handles bidirectional communications between the two devices.
@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@creation date: 07-07-2020 (started)

"""


import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String

import serial
import sys
import struct

RPM_goal = 15
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

		self.sending_timer = self.create_timer(timer_period, self.sendToArduino)
		self.receiving_timer = self.create_timer(timer_period, self.readFromArduino)


	def readFromArduino(self):
		"""
		Reads data from serial port
		"""

		# if len(sys.argv) < 2:
		# 	print("wrong number of arguments to serial_interface")
		# 	print(sys.argv)
		# 	return
		# while not rospy.is_shutdown():
		msg = String()
		packet = self.ser.read(999)

		self.get_logger().info('Received: "%s"' % packet)

		# need to find start and end of packet, isolate the data to use for checksum

		if len(packet) >3:
			# sync_string = struct.unpack('s', bytes(packet[0:4]))[0]
			motor_arduino_ID = struct.unpack('b', bytes(packet[4:5]))[0]
			rpm_est = struct.unpack('f', packet[5:9])[0]
			rpm_est = struct.unpack('f', packet[9:13])[0]
			checksum_byte = struct.unpack('b', bytes(packet[13:14]))[0]
			self.get_logger().info('Received motor ID: "%d" estimated RPM: "%f"' % (motor_arduino_ID, rpm_est))
			# self.get_logger().info('Received motor ID: "%d" current: "%f"' % (motor_arduino_ID, rpm_est))
			# self.get_logger().info('Received: motor ID: %d, RPM: %d' % (packet[4], packet[5]))
			checksum = 0
			for data in packet[4:14]: 						# <---- set range of checksum 
				checksum ^= data

			# self.get_logger().info('Received 01: "%d"' % (ord(packet[4:5])))
			# self.get_logger().info('Received 02: "%d"' % (ord(packet[5:6])))
			# self.get_logger().info('Received 03: "%d"' % (ord(packet[6:7])))
			# self.get_logger().info('Received 04: "%d"' % (ord(packet[7:8])))
			self.get_logger().info('Received checksum: "%s"' % (checksum_byte))

			self.get_logger().info('checked (0 is good): "%s"' % hex(checksum))	

		
		# self.get_logger().info('Checksum: "%d"' % checksum)
		# print(str(sys.argv[1]))

		msg.data = str(packet)
		self.publisher_.publish(msg)
		self.get_logger().info('------------------------')



	def sendToArduino(self):
		"""
		Packs data into array of bytes to form a packet. Writes packet to serial port.
		"""

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

