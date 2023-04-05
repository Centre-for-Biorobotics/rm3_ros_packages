#!/usr/bin/python3

from typing import List

from .node_position import NodePosition
from .graph_node import GraphNode

from geometry_msgs.msg import Point

GRAPH_NODE_SIZE = 0.8

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

    def cost(self, pos1: NodePosition, pos2: NodePosition):
        return 1

    def get_surroundings(self, center : NodePosition, distance : int, path : List[NodePosition] = None) -> str:
        if center is None:
            return ""

        x = center.x
        y = center.y

        txt = "\n"
        for i in range(distance, -distance - 1, -1):
            for j in range(distance, -distance - 1, -1):
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
                if not self.node_exists(this_pos):
                    txt += '-'
                    continue

                node: GraphNode = self.nodes[this_pos]
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
        p.x = round(position.x / GRAPH_NODE_SIZE, 0)
        p.y = round(position.y / GRAPH_NODE_SIZE, 0)
        return p
        
