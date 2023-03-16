
from enum import Enum

class DirectionTwist(Enum):
    MOVE_LEFT       = [ 0,  1,  0]
    MOVE_RIGHT      = [ 0, -1,  0]
    MOVE_FORWARD    = [ 1,  0,  0]
    MOVE_BACKWARD   = [-1,  0,  0]
    TURN_LEFT       = [ 0,  0, -1]
    TURN_RIGHT      = [ 0,  0,  1]