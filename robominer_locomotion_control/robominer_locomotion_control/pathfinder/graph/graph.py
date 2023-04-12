#!/usr/bin/python3

from typing import List

from .node_position import NodePosition
from .graph_node import GraphNode

import numpy as np

import math

from geometry_msgs.msg import Point

GRAPH_NODE_SIZE = 1

class Graph:
    nodes = {}

    def node_exists(self, position: NodePosition):
        return position in self.nodes.keys()

    def node_passable(self, position: NodePosition):
        return position not in self.nodes.keys() or self.nodes[position].passable

    def add_node(self, position: NodePosition, passable: bool=True):
        if position in self.nodes:
            raise RuntimeError('Node already existed!')

        self.nodes[position] = GraphNode(position, passable)

    def mark_node_unpassable(self, position: NodePosition):
        if position in self.nodes:
            if self.nodes[position].passable:
                self.nodes[position].passable = False
        else:
            self.add_node(position, False)

    def neighbors(self, position: NodePosition) -> List[NodePosition]:
        neighbors = []
        x, y = position.x, position.y

        potential_neighbors = [
            NodePosition(x + 1, y),
            NodePosition(x + 1, y),
            NodePosition(x - 1, y),
            NodePosition(x, y + 1),
            NodePosition(x, y - 1),
            NodePosition(x + 1, y + 1),
            NodePosition(x + 1, y - 1),
            NodePosition(x - 1, y + 1),
            NodePosition(x - 1, y - 1)
        ]

        for pos in potential_neighbors:
            if self.node_passable(pos):
                neighbors.append(pos)

        return neighbors
    
    def line_of_sight_zero_check(self, n1, n2, dx_max, dy_max):
        if dy_max == 0 and dx_max == 0:
            return {n1}
        
        if dx_max == 0:
            positions = {n1, n2}
            dy_abs = 0
            dy_max_abs = abs(dy_max)
            dy_sign = np.sign(dy_max)
            while dy_abs < dy_max_abs: 
                dy_abs += 1
                positions.add(NodePosition(n1.x, dy_sign * int(dy_abs) + n1.y))

            return positions
        
        if dy_max == 0:
            positions = {n1, n2}
            dx_abs = 0
            dx_max_abs = abs(dx_max)
            dx_sign = np.sign(dx_max)
            while dx_abs < dx_max_abs:
                dx_abs += 1
                positions.add(NodePosition(dx_sign * dx_abs + n1.x, n1.y))
            return positions
        
        return None
    
    def line_of_sight_nodes(self, n1: NodePosition, n2: NodePosition) -> List[NodePosition]:
        """
        Construct a line between two node positions and get all the 
        NodePositions in the grid that the line touches.

        Can be expanded to 3D by including z in the check as well.
        """
        # calculate 2D line path
        dx_max = n2.x - n1.x
        dy_max = n2.y - n1.y

        zero_check_result = self.line_of_sight_zero_check(n1, n2, dx_max, dy_max)
        if zero_check_result is not None:
            return zero_check_result
        
        positions = {n1, n2}  # Include start and endpoint in line of sight

        dx_max_abs = abs(dx_max)
        dy_max_abs = abs(dy_max)

        dx_abs = 0
        dy_abs = 0

        dx_sign = 1 if dx_max >= 0 else -1
        dy_sign = 1 if dy_max >= 0 else -1

        a_abs = abs(dy_max/dx_max)  # construct line y = ax

        while dx_abs < dx_max_abs and dy_abs < dy_max_abs:
            # check whether x or y axis will touch a point first
            if (self.add_or_ceil(dx_abs) - dx_abs) <= (self.add_or_ceil(a_abs * dx_abs) - a_abs * dx_abs):
                dx_abs = self.add_or_ceil(dx_abs)
                dy_abs = a_abs * dx_abs
                positions.add(NodePosition(dx_sign * dx_abs + n1.x, dy_sign * int(dy_abs) + n1.y))
            else:
                dy_abs = self.add_or_ceil(dy_abs)
                dx_abs = dy_abs / a_abs
                positions.add(NodePosition(dx_sign * int(dx_abs) + n1.x, dy_sign * dy_abs + n1.y))

        return positions


    def add_or_ceil(self, num) -> int:
        if num != math.ceil(num):
            return math.ceil(num)
        
        return num + 1


    def cost(self, pos1: NodePosition, pos2: NodePosition):
        return 1

    def get_surroundings(self, center : NodePosition, distance : int, path : List[NodePosition] = None, log_line_of_sight=False) -> str:
        if center is None:
            return ""

        x = center.x
        y = center.y

        txt = "\n"

        if log_line_of_sight and path and len(path) >= 1:
            los_nodes = self.line_of_sight_nodes(center, path[-1])
        for i in range(distance, -distance - 1, -1):
            for j in range(-distance, distance - 1):
                this_pos = NodePosition(x + i, y + j)
                if i == 0 and j == 0:
                    txt += 'P'
                    continue
                if self.node_exists(this_pos) and not self.nodes[this_pos].passable:
                    txt += '|'
                    continue
                if path is not None and this_pos in path:
                    txt += 'o'
                    continue
                if log_line_of_sight and path and this_pos in los_nodes:
                    txt += 'L'
                    continue
                if not self.node_exists(this_pos):
                    txt += '-'
                    continue

                txt += ' '
            txt += "\n"
        return txt

    @classmethod
    def translate_position_to_graph_pos(cls, position: Point) -> NodePosition:
        x_temp = int(round(position.x / GRAPH_NODE_SIZE, 0))
        y_temp = int(round(position.y / GRAPH_NODE_SIZE, 0))
        return NodePosition(x_temp, y_temp)

    @classmethod
    def translate_position_to_graph_pos_unrounded(cls, position: Point) -> Point:
        p = Point()
        p.x = position.x / GRAPH_NODE_SIZE
        p.y = position.y / GRAPH_NODE_SIZE
        return p
        
