"""
Program to control the RM3 pathfinder.

@author: Tanel Kossas
@contact: tanel.kossas@gmail.com
"""

import rclpy
from rclpy.node import Node

import sys

from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32

from .control.algorithm import PathfinderAlgorithm

help_string = """
Configurable parameters:
Can be changed by writing the command, then a space and then the desired parameter. Current 
information appears when writing just the command (e.g. just 'algo' for getting the current
algorithm). The current information is the default settings, so if the robot has been running,
then it shows incorrect information for a setting until you give the command for that setting.

-   a/algo         Look at/change the current movement algorithm that is being used. The list of
                   available algorithms can be viewed when with the command 'a' or 'algo'.
                   Example: 'a left_wall' for changing algorithm to be left-wall-follower.
-   d/dest         Look/change the destination for the movement algorithm (X, Y). Only applicable 
                   to waypoint movement algorithms, i.e. A_STAR.
                   Example: 'd 2.2, 5' to change the destination to X:2.2, Y:5.
-   v/vel/speed    Look at/change the current velocity of the robot.
                   Example: 'speed 5'.
Helper methods:

-   h/help         Displays the help that you're currently looking at.
-   s/stop         Stop the robot - set the algorithm to NONE (shorthand for 'a none').
-   e/exit         Stop the robot and exit the application.
"""

class PathfinderControl(Node):

    def __init__(self):
        super().__init__('pathfinder_control')

        self.vel : float = 1.
        self.pub_vel = self.create_publisher(Float32, '/cmd_pathfinder_vel', 10)

        self.curr_algorithm : PathfinderAlgorithm = PathfinderAlgorithm.NONE
        self.pub_algo = self.create_publisher(String, '/cmd_pathfinder_algorithm', 10)

        self.destination : Point = None
        self.pub_dest = self.create_publisher(Point, '/cmd_pathfinder_destination', 10)

    def parse_command(self, inp : str) -> None:
        inp = inp.lower().strip()
        
        # sanitize multiple spaces/whitespaces into one
        inp = ' '.join(inp.split())

        if is_string_or_continues_with_space(inp, "a"):
            self.parse_algorithm(remove_prefix(inp, "a"))
        elif is_string_or_continues_with_space(inp, "algo"):
            self.parse_algorithm(remove_prefix(inp, "algo"))
        
        elif is_string_or_continues_with_space(inp, "d"):
            self.parse_destination(remove_prefix(inp, "d"))
        elif is_string_or_continues_with_space(inp, "dest"):
            self.parse_destination(remove_prefix(inp, "dest"))
        
        elif is_string_or_continues_with_space(inp, "v"):
            self.parse_speed(remove_prefix(inp, "v"))
        elif is_string_or_continues_with_space(inp, "vel"):
            self.parse_speed(remove_prefix(inp, "vel"))
        elif is_string_or_continues_with_space(inp, "speed"):
            self.parse_speed(remove_prefix(inp, "speed"))

        elif is_string_or_continues_with_space(inp, "h") or is_string_or_continues_with_space(inp, "help"):
            self.get_logger().info(help_string)

        elif is_string_or_continues_with_space(inp, "s") or inp.startswith("stop"):
            self.stop()
        
        elif inp == "e" or inp.startswith("exit"):
            sys.exit("Exited Progam")
        else:
            self.get_logger().error("Invalid!")

    def all_algorithms_string(self) -> str:
        return ", ".join(PathfinderAlgorithm._value2member_map_.keys())

    def parse_algorithm(self, inp: str) -> None:
        if inp == "":
            self.get_logger().info("Current algorithm: " + str(self.curr_algorithm.value) + "\nAvailable algorithms: " + self.all_algorithms_string())
            return

        try:
            self.curr_algorithm = PathfinderAlgorithm._value2member_map_[inp.upper()]
            self.pub_algo.publish(String(data=(self.curr_algorithm.value)))

            self.get_logger().info("Algorithm changed to: " + str(self.curr_algorithm.value))

        except Exception as e:
            self.get_logger().error("Invalid algorithm. Algorithm has to be one of the following (case-insensitive): " + self.all_algorithms_string())
    
    def parse_destination(self, inp: str) -> None:
        if inp == "":
            self.get_logger().info("Current destination: " + str(self.destination))
            return

        try:
            inp_split = inp.split(",")

            if len(inp_split) != 2:
                inp_split = inp.split(" ")

            if len(inp_split) != 2:
                raise ValueError

            x, y = inp_split
            
            destination = Point()
            destination.x, destination.y = float(x), float(y)

            self.pub_dest.publish(destination)
            self.destination = destination
            self.get_logger().info("Destination changed to: " + str(self.destination))

        except ValueError:
            self.get_logger().error("Invalid destination input. Please pass destination as 'X,Y', 'X Y' or 'X, Y'. Floats are also accepted, e.g.: '1.234, 4.3'")

    def parse_speed(self, inp: str) -> None:
        if inp == "":
            self.get_logger().info("Current velocity: " + str(self.vel))
            return

        try:
            v = float(inp)

            self.pub_vel.publish(Float32(data=v))
            self.vel = v
            self.get_logger().info("Velocity changed to: " + str(self.vel))
        except ValueError:
            self.get_logger().error("Invalid provided speed!")

    def stop(self):
        self.pub_algo.publish(String(data=PathfinderAlgorithm.NONE.value))
        self.get_logger().info("Removed movement algorithm")


def is_string_or_continues_with_space(s: str, check_str: str) -> bool:
    return s == check_str or s.startswith(check_str + " ")


def remove_prefix(text: str, prefix: str) -> str:
    return text[text.startswith(prefix) and len(prefix):].strip()


def main(args=None):
    rclpy.init(args=args)

    node = PathfinderControl()

    node.get_logger().info(help_string)

    try:
        while True:
            inp = input()
            node.parse_command(inp)
    except:
        node.stop()

if __name__ == '__main__':
    main()
