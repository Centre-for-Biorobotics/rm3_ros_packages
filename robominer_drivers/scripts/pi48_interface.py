#!/usr/bin/python3
"""
Serial interface for Pacific Inertial IMU (pi-48).
Based on TAU 'ros2_pi48_imu' package.

The sensor is supposed to stream at 250Hz, but from USB we get data at approx.62.6Hz, and each bunch contains 4 packets.
TODO: figure out if this is OK, if not figure out what to do about it.

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 23-03-2022
"""
import rclpy
from rclpy.node import Node

import binascii
import numpy as np

import serial
import serial.tools.list_ports

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class pi48Interface(Node):
    """Interfaces a pi48 IMU over serial port. Reads and decodes the incoming packets. 
    Populates and publishes an Imu type message.
    """

    def __init__(self):
        super().__init__('pi48_interface')

        # initialize message and publisher
        self.imu_msg = Imu()
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # get device parameters and find device
        self.declare_parameter('device_name')
        self.declare_parameter('device_hwid')
        self.get_logger().info(f'params ok')
        self.which_device = self.get_parameter('device_name').value
        self.device_hwid = self.get_parameter('device_hwid').value

        self.port = list(serial.tools.list_ports.grep(self.device_hwid))[1][0]
        self.get_logger().info(f'device name: {self.which_device}, device hwid: {self.device_hwid}, port: {self.port}')

        # start serial communication
        self.ser = serial.Serial(
            self.port, 230400, timeout = 0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)
        
        # setup timer for parsing and publishing the data
        self.receiving_timer_period = 0.004 # reading at 250Hz
        self.receiving_timer = self.create_timer(
            self.receiving_timer_period,
            self.readBuffer
        )


    def readBuffer(self):
        """
        Reads data from serial buffer. Calls to publish.
        @input: self
        @returns: /
        """
        
        packet = self.ser.read(999)
        
        if len(packet)>1:
            self.publishImu(packet)
            
            

    def publishImu(self, packet_bin):
        """
        Parses the IMU packet. Publishes the angular velocity and linear acceleration as imu message.
        @input: self
        @input: packet in binary format
        @returns: /
        """
        index = '4131'      # this string should be at the start of each packet.

        packet_string = binascii.b2a_hex(packet_bin)
        packet_string = str(packet_string.decode('utf-8'))
        self.get_logger().info(f'full packet: {packet_string}')

        # locate the index
        packet_header = packet_string.find(index)

        # isolate packet payload based on index location and length of packet
        packet_string = packet_string[packet_header+4: packet_header+150]
        self.get_logger().info(f'packet string: {packet_string}')

        ACC_X=int(packet_string[12:20],16)
        ACC_Y=int(packet_string[20:28],16)
        ACC_Z=int(packet_string[28:36],16)

        GYRO_X=int(packet_string[36:44],16)
        GYRO_Y=int(packet_string[44:52],16)
        GYRO_Z=int(packet_string[52:60],16)

        ACC=np.array([ACC_X,ACC_Y,ACC_Z],dtype=np.uint32)
        GYRO=np.array([GYRO_X,GYRO_Y,GYRO_Z],dtype=np.uint32)

        ACC.byteswap(True)
        GYRO.byteswap(True)

        ACC.dtype=np.int32
        GYRO.dtype=np.int32

        ACC_X=float(ACC[0])*2**(-17)
        ACC_Y=float(ACC[1])*2**(-17)
        ACC_Z=float(ACC[2])*2**(-17)

        GYRO_X=float(GYRO[0])*2**(-17) * np.pi / 180.0
        GYRO_Y=float(GYRO[1])*2**(-17) * np.pi / 180.0
        GYRO_Z=float(GYRO[2])*2**(-17) * np.pi / 180.0

        ACC_ROS = Vector3()
        ACC_ROS.x = ACC_X
        ACC_ROS.y = ACC_Y
        ACC_ROS.z = ACC_Z

        GYRO_ROS = Vector3()
        GYRO_ROS.x = GYRO_X
        GYRO_ROS.y = GYRO_Y
        GYRO_ROS.z = GYRO_Z

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = 'pi48'
        self.imu_msg.linear_acceleration = ACC_ROS
        self.imu_msg.angular_velocity = GYRO_ROS

        self.imu_pub.publish(self.imu_msg)



def main(args=None):
    rclpy.init(args=args)
    pi48_interface = pi48Interface()
    rclpy.spin(pi48_interface)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pi48_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()