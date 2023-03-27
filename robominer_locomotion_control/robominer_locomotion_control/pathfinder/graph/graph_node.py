#!/usr/bin/python3

from .node_position import NodePosition

class GraphNode:
    position: NodePosition
    passable: bool = True

    def __init__(self, position: NodePosition, passable: bool=True):
        self.position = position

    def __str__(self):
        return str(self.position) + ": " + str(self.passable)

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other):
        return isinstance(other, GraphNode) and self.position == other.position and self.x == other.y