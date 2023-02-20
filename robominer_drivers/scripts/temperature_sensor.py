#!/usr/bin/python3
"""
Reads the MCP9808 temperature sensor
Current version reads it from an I2C device directly

@author: Jaan Rebane
@contact: jaan.rebane@ttu.ee
@creation date: 2023-02-16

"""

import rclpy
import smbus

from rclpy.node import Node

from sensor_msgs.msg import Temperature

# Todo: add bus number as a ROS parameter
bus = smbus.SMBus(1)

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.temperature = 0
        self.publisher_temperature = self.create_publisher(Temperature, '/temperature_sensor/temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Code from: https://github.com/DcubeTechVentures/MCP9808
        # MCP9808 address, 0x18(24)
        # Select configuration register, 0x01(1)
        #               0x0000(00)      Continuous conversion mode, Power-up default
        config = [0x00, 0x00]
        bus.write_i2c_block_data(0x18, 0x01, config)

    def timer_callback(self):
        msg = Temperature()
        try:
            data = bus.read_i2c_block_data(0x18, 0x05, 2)
            # Convert the data to 13-bits
            tempValue = ((data[0] & 0x1F) * 256) + data[1]
            if tempValue > 4095 :
                tempValue -= 8192
            tempValue = tempValue * 0.0625
            msg.temperature = tempValue
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_temperature.publish(msg)
            # self.get_logger().info('Publishing. Raw value: %s' % tempValue)

        except OSError:
            self.get_logger().error('Temperature data file not found!')
            # If we would like to destroy the node:
            # temperature_sensor.destroy_node()
            # rclpy.shutdown()
            # But we don't, we try again.

def main(args=None):
    rclpy.init(args=args)
    temperature_sensor = TemperatureSensor()
    temperature_sensor.get_logger().info('Started temperature sensor node.')

    rclpy.spin(temperature_sensor)

    temperature_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
