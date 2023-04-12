from enum import Enum

class PathfinderAlgorithm(Enum):
    LEFT_WALL_FOLLOWER = 'LEFT_WALL'
    RIGHT_WALL_FOLLOWER = 'RIGHT_WALL'
    THETA_STAR = 'THETA_STAR'
    A_STAR = 'A_STAR'
    NONE = 'NONE'
