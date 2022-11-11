import numpy as np


def vec2str(vec):
    return " ".join([str(e) for e in vec])


def flip_quat(quat):
    return (np.array(quat) * [-1, 1, 1, 1]).tolist()


def flip_tuple(tuple):
    return np.array(tuple[::-1]).tolist()


def mirror_y(pos_vec):
    return (np.array(pos_vec) * [1, -1, 1]).tolist()


def mirror_inertia_y(inertia_vec):
    return (np.array(inertia_vec) * [1, 1, 1, -1, 1, -1]).tolist()
