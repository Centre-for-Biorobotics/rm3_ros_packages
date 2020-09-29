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


class SerialInterface(Node):
	def __init__(self):
		super().__init__('serial_interface')
		self.declare_parameter('port')

		self.port = self.get_parameter('port').value
		self.motor_ID = int(self.port[11])
		self.get_logger().info( 'Port: %s, Motor ID: %d' %( str(self.port), self.motor_ID))

		
		self.ser = serial.Serial(self.port, 115200, timeout = 0)
		self.publisher_ = self.create_publisher(String, 'received_data', 10)

		self.timer_period = 1.0 # seconds

		self.sending_timer = self.create_timer(self.timer_period, self.sendToArduino)
		self.receiving_timer = self.create_timer(self.timer_period, self.readFromArduino)


	def readFromArduino(self):
		"""
		Reads data from serial port
		"""

		self.msg = String()
		self.packet = self.ser.read(999)

		self.msg_start = -1
		self.msg_end = -1

		# self.get_logger().info('Received packet: "%s"' % self.packet)

		if len(self.packet) >3: # and len(self.packet) < 31:
			# find indices of header and end character:
			self.msg_start = self.packet.index('SYNC'.encode('UTF-8')) if 'SYNC'.encode('UTF-8') in self.packet else None
			self.msg_end = self.packet.index('\n'.encode('UTF-8')) if '\n'.encode('UTF-8') in self.packet else None
			
			# self.get_logger().info('msg start: "%d"' % self.msg_start)
			# self.get_logger().info('msg end: "%d"' % self.msg_end)

			# isolate data array
			self.data_packet = self.packet[ self.msg_start : self.msg_end ]
			if self.msg_start < self.msg_end:
				self.extractMessage(self.data_packet)
	

		# fix (shouldn't be the msg being published):
		self.msg.data = str(self.packet)
		self.publisher_.publish(self.msg)
		self.get_logger().info('------------------------')

	def extractMessage(self, data_packet):
		# self.get_logger().info('extracting from: "%s"' % self.data_packet)
		self.packet_length = len(data_packet)
		# self.get_logger().info(str(self.packet_length))
		self.header_length = 4

		# calculate XOR checksum of 'data' part of packet
		self.chk = self.calculateChecksum(data_packet[self.msg_start +4: self.msg_end ])

		# unpack packet:
		motor_arduino_ID = struct.unpack('b', bytes(data_packet[0+self.header_length:1+self.header_length]))[0]
		data_array_length = struct.unpack('b', bytes(data_packet[1+self.header_length:2+self.header_length]))[0]
		rpm_est = struct.unpack('f', data_packet[2+self.header_length:6+self.header_length])[0]
		if self.packet_length == 11: # no current measurement
			checksum_byte = struct.unpack('b', bytes(data_packet[6+self.header_length:7+self.header_length]))[0]
		elif self.packet_length == 15: # with current measurement
			motor_current = struct.unpack('f', data_packet[6+self.header_length:10+self.header_length])[0]
			checksum_byte = struct.unpack('b', bytes(data_packet[10+self.header_length:11+self.header_length]))[0]

		# self.get_logger().info('Received motor ID: "%d" estimated RPM: "%f"' % (motor_arduino_ID, rpm_est))
		self.get_logger().info('Received motor ID: "%d", checksum (0x00 is good): "%f"' % (motor_arduino_ID, self.chk))
		# self.get_logger().info('Received checksum: "%s"' % (checksum_byte))
		# self.get_logger().info('Calculated checksum (0 is good): "%s"' % hex(self.chk))


	def calculateChecksum(self, packet):
		# self.get_logger().info('packet: "%s"' % str(packet))
		checksum = 0
		for data in packet:
			checksum ^= data
		return checksum

	
	def sendToArduino(self):
		"""
		Packs data into array of bytes to form a packet. Writes packet to serial port.
		"""

		outbuffer = 'sync'.encode('UTF-8')

		outbuffer += struct.pack('b', self.motor_ID)
		outbuffer += struct.pack('b', RPM_goal)

		outbuffer += '\n'.encode('UTF-8')

		# self.get_logger().info( 'Sending: Motor ID: %d, RPM_goal: %d' %(self.motor_ID, RPM_goal))
		

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

