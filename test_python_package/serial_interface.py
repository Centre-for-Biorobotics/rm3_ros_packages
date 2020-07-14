import rclpy

from rclpy.node import Node

from std_msgs.msg import String

import serial
import sys
import struct

first_byte = 3
second_byte = 2

port = "/dev/ttyACM0"
# ser = serial.Serial(port, 115200, timeout = 1)

class SerialInterface(Node):
	def __init__(self):
		super().__init__('serial_interface')
		self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 0)
		self.publisher_ = self.create_publisher(String, 'received_data', 10)
		timer_period = 1.0 # seconds
		print("new node")
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
    	# self.i += 1
		msg.data = str(data)
		self.publisher_.publish(msg)

		# print(data)



	def sendToArduino(self):
		# global ser, first_byte, second_byte
		# port = sys.argv[1]
		# port = "/dev/ttyACM0"
		# ser = serial.Serial(port, 115200, timeout = 0)
		# ser_buffer = r''

		# while not rospy.is_shutdown():
		# 	data = ser.read(9999)
		# 	messages = []
		# 	if len(data) > 0:
		# 		ser_buffer += data
		# 		while True:
		# 			old_length = len(ser_buffer)
		# 			# ser_buffer, messages = 
		outbuffer = r'syncsync'
		outbuffer += str(first_byte)
		outbuffer += str(second_byte)
		outbuffer += '\n'
		print("bytes sent: " + str(first_byte) + ", " + str(second_byte))

		self.ser.write(outbuffer.encode("utf-8"))





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

