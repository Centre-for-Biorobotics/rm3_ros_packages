
from enum import Enum, auto

import numpy as np

class DirectionTwist(Enum):
    MOVE_LEFT       = auto(), np.array([  0,  50,  0], dtype=float)
    MOVE_RIGHT      = auto(), np.array([  0, -50,  0], dtype=float)
    MOVE_FORWARD    = auto(), np.array([ 50,   0,  0], dtype=float)
    MOVE_BACKWARD   = auto(), np.array([-50,   0,  0], dtype=float)
    TURN_LEFT       = auto(), np.array([  0,   0, -1], dtype=float)
    TURN_RIGHT      = auto(), np.array([  0,   0,  1], dtype=float)

    def __new__(cls, *args, **kwds):
        value = len(cls.__members__) + 1
        obj = object.__new__(cls)
        obj._value_ = value
        return obj
    def __init__(self, i, twist):
        self.i = i
        self.twist = twist