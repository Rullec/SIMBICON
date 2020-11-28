import numpy as np


def get_first_lager_index(x, x_lst):
    if len(x_lst) == 0:
        return 0

    for i in range(len(x_lst)):
        tar_x = x_lst[i]
        if tar_x > x:
            return i
    return len(x_lst) - 1


def interploate_catmull_rom(x, x_lst, y_lst):
    assert len(x_lst) == len(y_lst)
    assert len(x_lst) >= 2
    x_min, x_max = x_lst[0], x_lst[-1]

    # 1. beyond the control segment, we return the endpoint value
    if x < x_min:
        return y_lst[0]
    elif x > x_max:
        return y_lst[-1]

    # 2. begin to find the lager index
    # larger_idx >=1, larger_idx <= size - 1
    # it's P2
    larger_idx = get_first_lager_index(x, x_lst)
    assert larger_idx >= 1
    assert larger_idx <= len(x_lst) - 1
    p0 = y_lst[max(larger_idx - 2, 0)]
    p1 = y_lst[larger_idx - 1]
    p2 = y_lst[larger_idx]
    p3 = y_lst[min(len(x_lst) - 1, larger_idx + 1)]

    t = (x - x_lst[larger_idx - 1]) / \
        (x_lst[larger_idx] - x_lst[larger_idx - 1])

    return 0.5 * (
        (2 * p1) +
        (-p0 + p2) * t +
        (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
        (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t
    )