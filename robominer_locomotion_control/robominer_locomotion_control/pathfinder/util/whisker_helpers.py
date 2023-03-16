
from statistics import mean
from typing import List

from robominer_msgs.msg import Whisker

import numpy as np


WHISKER_ROW_AMOUNT = 10
WHISKERS_PER_ROW_AMOUNT = 8


def whiskers_add_noise(whiskers: List[Whisker], min_total_value, max_total_value) -> None:
    random_matrix = np.random.normal(loc=0, scale=.05, size=(len(whiskers), 2))

    for i in range(len(whiskers)):
        whisker = whiskers[i]
        whisker.x = np.clip(whisker.x + random_matrix[i][0], min_total_value, max_total_value)
        whisker.y = np.clip(whisker.y + random_matrix[i][1], min_total_value, max_total_value)


def create_whisker_matrix(whiskers: List[Whisker]) -> List[List[Whisker]]:
    whisker_matrix = [[None for _ in range(WHISKERS_PER_ROW_AMOUNT)] for _ in range(WHISKER_ROW_AMOUNT)]

    for whisker in whiskers:
        whisker_matrix[whisker.pos.row_num][whisker.pos.col_num] = whisker

    # Set row to None that don't have a whisker.
    return [_list if any(_list) else None for _list in whisker_matrix]


def create_bias_matrix(whiskers: List[Whisker]) -> np.array:
    bias_matrix = np.zeros((WHISKER_ROW_AMOUNT, WHISKERS_PER_ROW_AMOUNT, 3), dtype=float)

    for whisker in whiskers:
        bias_matrix[whisker.pos.row_num][whisker.pos.col_num] = np.array([whisker.x, whisker.y, whisker.z])

    # Set row to None that don't have a whisker.
    return bias_matrix


def create_averaged_bias_matrix(offset_matrices: np.array) -> np.array:
    return np.sum(offset_matrices, axis=0) / len(offset_matrices)


def whiskers_create_simulated_bias_matrix() -> np.array:
    return np.random.normal(loc=0, scale=.24, size=(WHISKER_ROW_AMOUNT, WHISKERS_PER_ROW_AMOUNT, 2))


def whiskers_apply_simulated_bias(whiskers: List[Whisker], simulated_bias_matrix: np.array) -> None:
    for w in whiskers:
        w.x += simulated_bias_matrix[w.pos.row_num][w.pos.col_num][0]
        w.y += simulated_bias_matrix[w.pos.row_num][w.pos.col_num][1]


def create_adjusted_whisker_matrix(whiskers: List[Whisker], offset_whisker_matrix: np.array):
    whisker_matrix = [[None for _ in range(WHISKERS_PER_ROW_AMOUNT)] for _ in range(WHISKER_ROW_AMOUNT)]

    for whisker in whiskers:
        whisker.x -= offset_whisker_matrix[whisker.pos.row_num][whisker.pos.col_num][0]
        whisker.y -= offset_whisker_matrix[whisker.pos.row_num][whisker.pos.col_num][1]
        whisker.z -= offset_whisker_matrix[whisker.pos.row_num][whisker.pos.col_num][2]

        whisker_matrix[whisker.pos.row_num][whisker.pos.col_num] = whisker

    # Set row to None that don't have a whisker.
    return [_list if any(_list) else None for _list in whisker_matrix]


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
