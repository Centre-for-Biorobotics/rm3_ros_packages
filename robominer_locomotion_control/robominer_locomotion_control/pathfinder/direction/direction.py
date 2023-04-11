
from __future__ import annotations

from enum import Enum
import numpy as np

from .direction_twist import DirectionTwist

class Direction(Enum):
    LEFT = 'L'
    RIGHT = 'R'
    FORWARD = 'F'
    BACKWARD = 'B'

    def opposite(self) -> Direction:
        if self == Direction.LEFT:
            return Direction.RIGHT
        elif self == Direction.RIGHT:
            return Direction.LEFT
        elif self == Direction.FORWARD:
            return Direction.BACKWARD
        elif self == Direction.BACKWARD:
            return Direction.FORWARD

    
    def move_twist(self) -> np.ndarray:
        if self == Direction.LEFT:
            return DirectionTwist.MOVE_LEFT.twist
        elif self == Direction.RIGHT:
            return DirectionTwist.MOVE_RIGHT.twist
        elif self == Direction.FORWARD:
            return DirectionTwist.MOVE_FORWARD.twist
        elif self == Direction.BACKWARD:
            return DirectionTwist.MOVE_BACKWARD.twist

    def turn_twist(self) -> list:
        if self == Direction.LEFT:
            return DirectionTwist.TURN_LEFT.twist
        elif self == Direction.RIGHT:
            return DirectionTwist.TURN_RIGHT.twist
        else:
            return np.array([0, 0, 0])