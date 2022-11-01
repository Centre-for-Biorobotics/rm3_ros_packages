#!/usr/bin/python3
"""
Serial interface for Pacific Inertial IMU (pi-48).
Based on TAU 'ros2_pi48_imu' package.

The sensor is supposed to stream at 250Hz, but from USB we get data at approx.62.5Hz, and each bunch contains 4 packets.
TODO: figure out if this is OK, if not figure out what to do about it.

In RM3 the pi48 is oriented as:
x: right
y: front
z: up

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

        self.pi48_packet_length = 108 # characters

        # get device parameters and find device
        self.declare_parameter('device_name')
        self.declare_parameter('device_hwid')

        self.which_device = self.get_parameter('device_name').value
        self.device_hwid = self.get_parameter('device_hwid').value

        self.port = list(serial.tools.list_ports.grep(self.device_hwid))[1][0]
        # self.get_logger().info(f'device name: {self.which_device}, device hwid: {self.device_hwid}, port: {self.port}')

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
        Reads data from serial buffer. Isolates the packet payload. Calls to publish.
        @input: self
        @returns: /
        """

        packet_bin = self.ser.read(999)

        if len(packet_bin)>1:
            packet_header = '4131'
            packet_good = False
            payload_length = 0

            # convert entire packet to string:
            packet_string = binascii.b2a_hex(packet_bin)
            packet_string = str(packet_string.decode('utf-8'))

            # locate the index and slice the packet string after it
            header_index = packet_string.find(packet_header)
            buffer = packet_string[header_index + len(packet_header):]

            # use the issue of ~4 packets per buffer read to handle empty or incomplete packets
            while (not packet_good):
                # find the length of payload until the next header
                payload_length = buffer.find(packet_header)

                # if the payload has an appropriate length take it, if not move to the next packet in the buffer
                if (payload_length == self.pi48_packet_length):
                    packet_good = True
                    # self.get_logger().info(f'good packet: {buffer[:payload_length]}')
                    self.publishImu(buffer[:payload_length])
                    break
                else:
                    # self.get_logger().info(f'bad packet: {packet_string}')
                    buffer = buffer[payload_length + len(packet_header):]
                    packet_good = False

    def publishImu(self, payload):
        """
        Parses the IMU packet. Publishes the angular velocity and linear acceleration as imu message.
        @input: self
        @input: packet in binary format
        @returns: /
        """

        ACC_X=int(payload[12:20],16)
        ACC_Y=int(payload[20:28],16)
        ACC_Z=int(payload[28:36],16)

        GYRO_X=int(payload[36:44],16)
        GYRO_Y=int(payload[44:52],16)
        GYRO_Z=int(payload[52:60],16)

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
