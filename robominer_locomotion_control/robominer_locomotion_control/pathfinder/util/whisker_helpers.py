
from statistics import mean
from typing import List

from robominer_msgs.msg import Whisker


WHISKER_ROW_AMOUNT = 10
WHISKERS_PER_ROW_AMOUNT = 8


def create_whisker_matrix(whiskers: List[Whisker]):
    whisker_matrix = [[0 for _ in range(WHISKERS_PER_ROW_AMOUNT)] for _ in range(WHISKER_ROW_AMOUNT)]

    for whisker in whiskers:
        whisker_matrix[whisker.pos.row_num][whisker.pos.col_num] = whisker

    return whisker_matrix


def whisker_multiplier(col_num: int):
    """
    Convert 4-7 to 1..4 and 0-3 to -4..-1 for finding appropriate turning radius
    Returns: Value from -4..4 excluding 0
    """
    if col_num < 4:
        return col_num - 4
    
    return col_num - 3


def calc_whiskers_inclination_x(whiskers : List[Whisker]):
    """
    Calculate an inclination for a whisker array, taking into account the position of the whiskers.
    Positive if higher columns have higher values, negative if lower columns have higher values.
    """
    return sum([abs(w.x) * whisker_multiplier(w.pos.col_num) for w in whiskers])


def calc_whiskers_inclination_y(whiskers : List[Whisker]):
    """
    Calculate an inclination for a whisker array, taking into account the position of the whiskers.
    Positive if higher columns have higher values, negative if lower columns have higher values.
    """
    return sum([abs(w.y) * whisker_multiplier(w.pos.col_num) for w in whiskers])


def calc_whiskers_inclination_euclid(whiskers : List[Whisker]):
    """
    Calculate an inclination for a whisker array, taking into account the position of the whiskers.
    Positive if higher columns have higher values, negative if lower columns have higher values.
    Max possible value is 31.4, should be capped at ~5.
    """
    return sum([whisker_euclid_dist(w) * whisker_multiplier(w.pos.col_num) for w in whiskers])


def calc_whisker_pressure_max(whiskers: List[Whisker]):
    """
    Get the maximum pressure of the whisker with most pressure in the list.
    """
    return max([whisker_euclid_dist(w) for w in whiskers])


def calc_whisker_pressure_avg(whiskers: List[Whisker]):
    """
    Get the average pressure of all whiskers in the list.
    """
    return mean([whisker_euclid_dist(w) for w in whiskers])


def whisker_euclid_dist(whisker: Whisker):
    """
    Calculate the euclidean distance of the whisker's x and y displacement.
    """
    return (whisker.x**2 + whisker.y**2)**0.5
