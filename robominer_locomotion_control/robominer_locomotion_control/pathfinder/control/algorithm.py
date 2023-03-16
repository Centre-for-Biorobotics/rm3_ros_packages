from enum import Enum

class PathfinderAlgorithm(Enum):
    LEFT_WALL_FOLLOWER = 'LEFT_WALL'
    RIGHT_WALL_FOLLOWER = 'RIGHT_WALL'
    A_STAR = 'A_STAR'
    NONE = 'NONE'
