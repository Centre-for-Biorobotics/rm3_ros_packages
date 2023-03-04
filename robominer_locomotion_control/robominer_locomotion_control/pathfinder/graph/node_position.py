#!/usr/bin/python3

class NodePosition:
    x: int
    y: int

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return str(self.x) + ", " + str(self.y)

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other):
        return isinstance(other, NodePosition) and self.x == other.x and self.y == other.y