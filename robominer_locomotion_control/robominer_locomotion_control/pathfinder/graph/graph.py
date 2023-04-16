#!/usr/bin/python3

from typing import List, Set

from .node_position import NodePosition
from .graph_node import GraphNode

import numpy as np

import math

from geometry_msgs.msg import Point

GRAPH_NODE_SIZE = 1
ROBOT_NODE_SIZE = 0.2  # How big should a robot be compared to a graph node when measuring line of sight

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
            #NodePosition(x - 1, y - 1),
            NodePosition(x - 1, y),
            #NodePosition(x - 1, y + 1),
            NodePosition(x, y - 1),
            NodePosition(x, y + 1),
            #NodePosition(x + 1, y - 1),
            NodePosition(x + 1, y)
            #NodePosition(x + 1, y + 1),
        ]

        for pos in potential_neighbors:
            if self.node_passable(pos):
                neighbors.append(pos)

        return neighbors
    
    def line_of_sight_nodes(self, n1: NodePosition, n2: NodePosition) -> Set[NodePosition]:
        """
        Finds all the nodes that fall on a rectangle drawn between n1 and n2 with a width of approximately c.

        Can get slow with big graphs and a c value of over 1. O(1) for c <= 1 and 2*O(n) for values higher than 1.
        """
        if n1 == n2:
            return {n1}, ""
        c = ROBOT_NODE_SIZE
        info = ""
        if abs(c) <= 1:
            info += "Line of sight between {} and {}".format(str(n1), str(n2)) + "\n\n"
            info += "-C" + "\n"
            los_nodes, nf = self._line_of_sight_nodes(n1, n2, -c)
            info += nf + "\n\n"
            info += str([str(los_node) for los_node in los_nodes]) + "\n"
            info += "+C" + "\n"
            los_nodes_plus, nf = self._line_of_sight_nodes(n1, n2, c)
            info += nf + "\n\n"
            info += str([str(los_node) for los_node in los_nodes_plus]) + "\n"
            los_nodes = los_nodes.union(los_nodes_plus)

            if abs(c) == 1:
                los_nodes_zero, nf = self._line_of_sight_nodes(n1, n2, 0)
                info += "ZERO" + "\n"
                info += nf + "\n"
                info += str([str(los_node) for los_node in los_nodes_zero]) + "\n"
                los_nodes = los_nodes.union(los_nodes_zero)

        if abs(c) > 1:
            los_nodes = set()
            c_interim = -c
            while c_interim <= c:
                los_nodes = los_nodes.union(self._line_of_sight_nodes(n1, n2, c_interim))
                c += 0.5

            # Make sure that the potential blindspot is also hit
            los_nodes.union(self._line_of_sight_nodes(n1, n2, c))
            
        return los_nodes, info
    
    def _line_of_sight_nodes(self, n1: NodePosition, n2: NodePosition, c: float) -> Set[NodePosition]:
        """
        Construct a line between two node positions and get all the 
        NodePositions in the grid that the line touches.

        Can be expanded to 3D by including z in the check as well.
        """
        # calculate 2D line path
        dx_max = n2.x - n1.x
        dy_max = n2.y - n1.y

        nf = ""
        
        zero_check_result = self._line_of_sight_zero_check(n1, n2, dx_max, dy_max)
        if zero_check_result is not None:
            return zero_check_result, nf
        
        positions = {n1, n2}  # Include start and endpoint in line of sight

        dx_max_abs = abs(dx_max)
        dy_max_abs = abs(dy_max)

        a = dy_max / dx_max
        a_abs = abs(a)  # construct line y = ax

        dx_sign = 1 if dx_max >= 0 else -1
        dy_sign = 1 if dy_max >= 0 else -1

        if dy_sign == 1 and c >= 0 or dy_sign == -1 and c < 0:
            dx = 0
            dy = c
        else:
            dx = -c / a
            dy = 0
        
        dx_abs, dy_abs = abs(dx), abs(dy)

        pot_node_x = int(n1.x + dx)
        pot_node_y = int(n1.y + dy)

        while dx_abs <= dx_max_abs and dy_abs <= dy_max_abs:
            nf += "dx_abs {:.2f}, dy_abs {:.2f}".format(dx_abs * dx_sign, dy_abs * dy_sign) + "\n"
            
            positions.add(NodePosition(math.floor(pot_node_x), math.floor(pot_node_y)))
            nf += "Added x={} y={}".format(str(pot_node_x), str(pot_node_y)) + "\n"

            # check whether x or y axis will touch a point first
            if (self._add_or_ceil(dy_abs) - c) * a_abs - dx_abs >= self._add_or_ceil(dx_abs) * a_abs + c - dy_abs:
                dx_abs = self._add_or_ceil(dx_abs)
                dy_abs = abs(a_abs * dx_abs + c)
                
                pot_node_x, pot_node_y = dx_sign * dx_abs + n1.x, dy_sign * int(dy_abs) + n1.y
            else:
                dy_abs = self._add_or_ceil(dy_abs)
                dx_abs = abs((dy_abs - c) / a_abs)
                pot_node_x, pot_node_y = dx_sign * int(dx_abs) + n1.x, dy_sign * dy_abs + n1.y

        if not (n1.x == -10 and n1.y == 5 and n2.x == -8 and n2.y == 3):
            nf = ""

        return positions, nf
    
    def _line_of_sight_zero_check(self, n1: NodePosition, n2:NodePosition, dx_max, dy_max):
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


    def _add_or_ceil(self, num) -> int:
        if int(num) != math.ceil(num):
            return math.ceil(num)
        
        return num + 1


    def _cost(self, pos1: NodePosition, pos2: NodePosition):
        return 1

    def get_surroundings(self, center : NodePosition, distance : int, path : List[NodePosition] = None, log_line_of_sight=False) -> str:
        if center is None:
            return ""

        x = center.x
        y = center.y

        txt = "\n"

        if log_line_of_sight and path and len(path) >= 1:
            los_nodes, nf = self.line_of_sight_nodes(center, path[0])
            txt += nf + "\n"
        for i in range(distance, -distance - 1, -1):
            for j in range(distance, - distance - 1, -1):
                this_pos = NodePosition(x + i, y + j)
                if i == 0 and j == 0 and log_line_of_sight and path and this_pos in los_nodes:
                    txt += 'P'
                    continue
                if i == 0 and j == 0:
                    txt += 'p'
                    continue
                if self.node_exists(this_pos) and not self.nodes[this_pos].passable and log_line_of_sight and path and this_pos in los_nodes:
                    txt += '/'
                    continue
                if log_line_of_sight and path and this_pos in los_nodes:
                    txt += 'L'
                    continue
                if self.node_exists(this_pos) and not self.nodes[this_pos].passable:
                    txt += '|'
                    continue
                if path is not None and this_pos in path:
                    txt += 'o'
                    continue
                if not self.node_exists(this_pos):
                    txt += '-'
                    continue

                txt += ' '
            txt += "\n"
        if log_line_of_sight and path and los_nodes:
            txt += str([str(los_node) for los_node in los_nodes]) + "\n"
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
        
