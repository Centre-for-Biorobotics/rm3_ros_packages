import rclpy

from rclpy.node import Node

from std_msgs.msg import String

class SerialInterface(Node):
	def __init__(self):
		super().__init__('serial_interface')
		self.publisher_ = self.create_publisher(String, 'topic', 10)
		timer_period = 0.5 # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)



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

