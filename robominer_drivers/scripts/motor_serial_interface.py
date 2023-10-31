#!/usr/bin/python3
"""
Serial interface between Olimex sbc and arduino nano

Handles bidirectional communications between the two devices.
@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@creation date: 07-07-2020 (started)

"""

import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String, Float32
from robominer_msgs.msg import MotorModuleFeedback
from robominer_msgs.msg import MotorModuleCommand


import serial
import serial.tools.list_ports
import sys
import struct

RPM_goal = 0
rpm_est = 0


motors_mapping = {
    "front_right": "0",
    "rear_right": "1",
    "rear_left": "2",
    "front_left": "3"
}


class SerialInterface(Node):
    def __init__(self):
        super().__init__('serial_interface')

        # use parameters defined in launch file (directly or yaml)
        self.declare_parameter('motor_name', 'default_motor')
        self.declare_parameter('arduino_sn', 'default_sn')
        self.which_motor = self.get_parameter('motor_name').value
        self.which_arduino = self.get_parameter('arduino_sn').value

        self.RPM_goal = 0
        self.overcurrent = 0    # current safety threshold counter
        self.current_threshold = 5200.0  # mA
        self.RPM_reset_counter = 0
        self.RPM_current_zero = False
        self.RPM_wait = False

        # incoming packet info
        self.packet_header = 'SYNC'
        self.packet_trailer = 'CNYS'
        self.header_length = len(self.packet_header)

        # scan com ports to find the arduino that has the specified serial number and get its port
        self.port = list(serial.tools.list_ports.grep(self.which_arduino))[0][0]

        self.motor_ID = motors_mapping[self.which_motor]
        self.get_logger().info(
                'Port: %s, Motor name: %s, arduino sn: %s'
                % (str(self.port), self.which_motor, self.which_arduino))

        self.ser = serial.Serial(self.port, 115200, timeout = 0)
        self.publisher_motor_module = self.create_publisher(MotorModuleFeedback, 'motor_module', 10)
        self.motor_topic_name = '/motor' + str(self.motor_ID) + '/motor_rpm_setpoint'
        self.sub_motor_setpoint = self.create_subscription(MotorModuleCommand, self.motor_topic_name, self.motorCommandsCallback, 10)

        self.transmitting_timer_period = 0.2  # for sending rpm setpoint to arduino at 5Hz
        self.receiving_timer_period = 0.2  # arduino sends back rpm and current sensing every 200 milliseconds

        self.sending_timer = self.create_timer(
                self.transmitting_timer_period,
                self.sendToArduino)
        self.receiving_timer = self.create_timer(
                self.receiving_timer_period,
                self.readFromArduino)

    def readFromArduino(self):
        """
        Reads data from serial port and calls for message extraction
        """

        self.packet = self.ser.read(999)

        self.msg_start = -1
        self.msg_end = -1

        if len(self.packet) >3:
            # find indices of header and end character:
            self.msg_start = self.packet.index(self.packet_header.encode('UTF-8')) if self.packet_header.encode('UTF-8') in self.packet else None
            self.msg_end = self.packet.index(self.packet_trailer.encode('UTF-8')) if self.packet_trailer.encode('UTF-8') in self.packet else None

            # isolate data array (payload)
            self.data_packet = self.packet[ self.msg_start : self.msg_end ]

            if None not in (self.msg_start, self.msg_end):
                if self.msg_start < self.msg_end:
                    self.extractMessage(self.data_packet)

    def extractMessage(self, data_packet):
        """
        unpacks the data_packet and populates messages that contain motor module feedback info for publishing
        """
        self.motor_module_msg = MotorModuleFeedback() # custom message

        packet_length = len(data_packet) # determine data payload length (+ header length = 4 is included)

        # calculate XOR checksum of 'data' part of packet
        self.chk = self.calculateChecksum(data_packet[self.msg_start + self.header_length: self.msg_end])

        # unpack packet:
        motor_arduino_ID = struct.unpack('b', bytes(data_packet[0+self.header_length:1+self.header_length]))[0]

        data_array_length = struct.unpack('b', bytes(data_packet[1+self.header_length:2+self.header_length]))[0]

        rpm_est = struct.unpack('f', data_packet[2+self.header_length:6+self.header_length])[0]

        self.motor_module_msg.header.stamp = self.get_clock().now().to_msg()
        self.motor_module_msg.header.frame_id = self.which_motor
        self.motor_module_msg.motor_id = str(motor_arduino_ID)
        self.motor_module_msg.motor_rpm = rpm_est

        if packet_length == 11: 		# no current measurement
            checksum_byte = struct.unpack('b', bytes(data_packet[6+self.header_length:7+self.header_length]))[0]
        elif packet_length == 19: 		# with current and voltage measurement
            self.motor_module_msg.motor_current_ma = struct.unpack('f', data_packet[6+self.header_length:10+self.header_length])[0]
            self.motor_module_msg.motor_voltage_v = struct.unpack('f', data_packet[10+self.header_length:14+self.header_length])[0]
            checksum_byte = struct.unpack('b', bytes(data_packet[14+self.header_length:15+self.header_length]))[0]

        if self.chk != 0:
            self.get_logger().info(f'Checksum problem: {data_packet}')

        # register in counter if motor draws high current and give a warning, otherwise reset the counter
        if (self.motor_module_msg.motor_current_ma >= self.current_threshold): # current threshold at 6A
            self.overcurrent += 1
            self.get_logger().warn('High current: "%f" mA.' % (self.motor_module_msg.motor_current_ma))
        else:
            # sporadic high current readings will be ignored
            self.overcurrent = 0

        if self.overcurrent >= 5:
            # instead of destroy_node force RPM=0 for 9 seconds
            self.get_logger().warn(f'Forced waiting (cause: high current)')
            self.RPM_wait = True
            #self.RPM_current_zero = True
            #self.RPM_reset_counter += 1
        if self.RPM_wait:
            self.RPM_reset_counter += 1
            self.RPM_current_zero = True
        if self.RPM_reset_counter >= 9*5:
            self.get_logger().info(f'RPM not zero')
            self.RPM_current_zero = False
            self.RPM_wait = False
            self.RPM_reset_counter = 0

        self.publisher_motor_module.publish(self.motor_module_msg)

    def calculateChecksum(self, packet):
        """
        calculates XOR checksum of an array of bytes (packet) and returns the
        result.
        """
        checksum = 0
        for data in packet:
            checksum ^= data
        return checksum

    def motorCommandsCallback(self, msg):
        if str(msg.motor_id) == str(self.motor_ID):
            if self.RPM_current_zero == True:
                self.RPM_goal = 0
            else:
                self.RPM_goal = msg.motor_rpm_goal
        return

    def sendToArduino(self):
        """
        Packs data into array of bytes to form a packet.
        Writes packet to serial port.
        """

        outbuffer = 'sync'.encode('UTF-8')

        outbuffer += struct.pack('b', int(self.motor_ID))
        outbuffer += struct.pack('b', self.RPM_goal)

        checksum_out = self.calculateChecksum(outbuffer[4:6])
        outbuffer += struct.pack('B', checksum_out)

        outbuffer += '\n'.encode('UTF-8')

        self.ser.write(outbuffer)  # writes to serial port


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
