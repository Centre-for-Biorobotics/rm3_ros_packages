#!/usr/bin/python3

from typing import List, Set

from .node_position import NodePosition
from .graph_node import GraphNode

import numpy as np

import math

from geometry_msgs.msg import Point

MAX_MAP_SIZE = 20

class Graph:
    nodes = {}

    def __init__(self, node_size: float, robot_size: float):
        self.node_size: float = node_size
        self.robot_size: float = robot_size

    def node_exists(self, position: NodePosition) -> bool:
        return position in self.nodes.keys()

    def node_passable(self, position: NodePosition) -> bool:
        return position not in self.nodes.keys() or self.nodes[position].passable

    def add_node(self, position: NodePosition, passable: bool=True) -> None:
        if position in self.nodes:
            raise RuntimeError('Node already existed!')

        self.nodes[position] = GraphNode(position, passable)

    def mark_node_unpassable(self, position: NodePosition) -> None:
        if position in self.nodes:
            if self.nodes[position].passable:
                self.nodes[position].passable = False
        else:
            self.add_node(position, False)

    def node_outside_bounds(self, pos: NodePosition) -> bool:
        return pos.x * self.node_size > MAX_MAP_SIZE and pos.y * self.node_size > MAX_MAP_SIZE

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
            #NodePosition(x + 1, y + 1)
        ] 

        for pos in potential_neighbors:
            if self.node_passable(pos) and not self.node_outside_bounds(pos):
                neighbors.append(pos)

        return neighbors
    
    def line_of_sight_nodes(self, n1: NodePosition, n2: NodePosition) -> Set[NodePosition]:
        """
        Finds all the nodes that fall on a rectangle drawn between n1 and n2 with a width of approximately c.

        Can get slow with big graphs and a c value of over 1. O(1) for c <= 1 and 2*O(n) for values higher than 1.
        """
        if n1 == n2:
            return {n1}
        c = self.robot_size / 2
        
        if abs(c) <= 1.4:
            los_nodes = set(self.line_of_sight_nodes_c(n1, n2, -c))
            los_nodes_plus = set(self.line_of_sight_nodes_c(n1, n2, c))
            los_nodes = los_nodes.union(set(los_nodes_plus))
            if abs(c) >= 0.7:
                los_nodes_zero = set(self.line_of_sight_nodes_c(n1, n2, 0))
                los_nodes = los_nodes.union(set(los_nodes_zero))

        if abs(c) > 1.4:
            los_nodes = set()
            c_interim = -c
            while c_interim <= c:
                los_nodes = los_nodes.union(set(self.line_of_sight_nodes_c(n1, n2, c_interim)))
                c += 0.7

            # Make sure that the potential blindspot is also hit
            los_nodes.union(set(self.line_of_sight_nodes_c(n1, n2, c)))

        # remove origin
            los_nodes.discard(n1)
        return los_nodes
    
    def cost(a, b) -> float:
        return 1.0
    
    def line_of_sight_nodes_c(self, n1: NodePosition, n2: NodePosition, c: float) -> List[NodePosition]:
        """
        Construct a line between two node positions and get all the 
        NodePositions in the grid that the line touches.

        Can be expanded to 3D by including z in the check as well.
        """
        # calculate 2D line path
        y_diff = n2.y - n1.y
        x_diff = n2.x - n1.x
        dy_min, dy_max = min(0, y_diff), max(0, y_diff)
        dx_min, dx_max = min(0, x_diff), max(0, x_diff)
        
        zero_check_result = self._line_of_sight_zero_check(n1, n2, c, x_diff, y_diff, dx_min, dx_max, dy_min, dy_max)
        if zero_check_result is not None:
            return zero_check_result
        
        positions = []

        a = y_diff / x_diff

        if y_diff >= 0 and c >= 0 or y_diff < 0 and c < 0:
            dx = 0
            dy = c
        else:
            dx = -c / a
            dy = 0

        # Calculate all intersections for x
        dx = math.ceil(np.clip(dx, dx_min, dx_max)) if x_diff >= 0 else math.floor(np.clip(dx, dx_min, dx_max))
        dy = a * dx + c
        pot_node_x, pot_node_y = dx + n1.x, math.floor(dy) + n1.y
        # cross all edges
        while dx_min <= dx <= dx_max and dy_min <= dy <= dy_max:
            # after crossing an edge, add both sides of the X axis crossing
            positions.append(NodePosition(pot_node_x, pot_node_y))

            previous_x = n1.x + self.subtract_one_abs(dx, x_diff)
            if n1.x + dx_min <= previous_x <= n1.x + dx_max:
                positions.append(NodePosition(previous_x, pot_node_y))

            # If passing a node edge, then add all four around it
            if abs(round(dx) - dx) < 0.05 and abs(round(dy) - dy) < 0.05:
                previous_y = n1.y + self.subtract_one_abs(math.floor(dy), y_diff)

                if n1.y + dy_min <= previous_y <= n1.y + dy_max:
                    positions.append(NodePosition(pot_node_x, previous_y))

                if n1.x + dx_min <= previous_x <= n1.x + dx_max and n1.y + dy_min <= previous_y <= n1.y + dy_max:
                    positions.append(NodePosition(previous_x, previous_y))

            dx = self.add_one_abs(dx, x_diff)
            dy = a * dx + c

            pot_node_x, pot_node_y = dx + n1.x, math.floor(dy) + n1.y

        if y_diff >= 0 and c >= 0 or y_diff < 0 and c < 0:
            dx = 0
            dy = c
        else:
            dx = -c / a
            dy = 0

        # Calculate all intersections for y
        dy = math.ceil(np.clip(dy, dy_min, dy_max)) if y_diff >= 0 else math.floor(np.clip(dy, dy_min, dy_max))
        dx = (dy - c) / a
        pot_node_x, pot_node_y = math.floor(dx) + n1.x, dy + n1.y
        while dy_min <= dy <= dy_max and dx_min <= dx <= dx_max:
            # after crossing an edge, add both sides of the Y axis crossing
            positions.append(NodePosition(pot_node_x, pot_node_y))

            second_pot_node_y = n1.y + self.subtract_one_abs(dy, y_diff)
            if n1.y + dy_min <= second_pot_node_y <= n1.y + dy_max:
                positions.append(NodePosition(pot_node_x, second_pot_node_y))

            dy = self.add_one_abs(dy, y_diff)
            dx = (dy - c) / a

            pot_node_x, pot_node_y = math.floor(dx) + n1.x, dy + n1.y

        return positions
    
    def round_lower(self, x) -> int:
        if x >= 0.0:
            return math.floor(x)
        else:
            return math.floor(x - 0.5)
        
    def add_one_abs(self, x, signed) -> int:
        if signed >= 0.0:
            return x + 1
        else:
            return x - 1
    
    def subtract_one_abs(self, x, signed) -> int:
        if signed >= 0.0:
            return x - 1
        else:
            return x + 1

    def _line_of_sight_zero_check(self, n1: NodePosition, n2:NodePosition, c, 
                                  x_diff, y_diff, dx_min, dx_max, dy_min, dy_max) -> Set[NodePosition]:
        if x_diff != 0 and y_diff != 0:
            return None

        positions = [n1, n2]

        if x_diff == y_diff == 0:
            return positions
        
        if x_diff == 0:
            dx = math.ceil(np.clip(c, dx_min, dx_max))
            dy = 0
            pot_node_x, pot_node_y = dx + n1.x, n1.y
            while dy_min <= dy <= dy_max and dx_min <= dx <= dx_max:    
                positions.append(NodePosition(pot_node_x, pot_node_y))

                dy = self.add_one_abs(dy, y_diff)
                pot_node_x, pot_node_y = dx + n1.x, dy + n1.y
            return positions

        # y_diff == 0:
        dy = math.ceil(np.clip(c, dy_min, dy_max))
        dx = 0
        pot_node_x, pot_node_y = n1.x, n1.y + dy
        while dy_min <= dy <= dy_max and dx_min <= dx <= dx_max:    
            positions.append(NodePosition(pot_node_x, pot_node_y))

            dx = self.add_one_abs(dx, x_diff)
            pot_node_x, pot_node_y = dx + n1.x, dy + n1.y
        return positions

    def get_surroundings(self, center : NodePosition, distance : int, path : List[NodePosition] = None, log_line_of_sight=False) -> str:
        if center is None:
            return ""

        x = center.x
        y = center.y

        txt = "\n"

        if log_line_of_sight and path and len(path) >= 1:
            los_nodes = self.line_of_sight_nodes(center, path[0])
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
                if self.node_outside_bounds(this_pos):
                    txt += 'x'
                    continue
                if not self.node_exists(this_pos):
                    txt += '-'
                    continue

                txt += ' '
            txt += "\n"
        #if log_line_of_sight and path and los_nodes:
            #txt += str([str(los_node) for los_node in los_nodes]) + "\n"
        return txt

    def translate_position_to_graph_pos(self, position: Point) -> NodePosition:
        x_temp = round(position.x / self.node_size)
        y_temp = round(position.y / self.node_size)
        return NodePosition(x_temp, y_temp)

    def translate_position_to_graph_pos_unrounded(self, position: Point) -> Point:
        p = Point()
        p.x = position.x / self.node_size
        p.y = position.y / self.node_size
        return p
