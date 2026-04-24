import math

import numpy as np


def model_matrix(theta: float, tx: float, ty: float) -> np.ndarray:
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    return np.array(
        [
            [cos_theta, -sin_theta, tx],
            [sin_theta, cos_theta, ty],
            [0.0, 0.0, 1.0],
        ]
    )


class ForwardKinematicsChain:
    """Planar chain using 2D homogeneous transform matrices."""

    def __init__(self, base: tuple[float, float], link_lengths: list[float]) -> None:
        self.base = base
        self.link_lengths = link_lengths

    def solve(self, angles: list[float]) -> list[np.ndarray]:
        if len(angles) != len(self.link_lengths):
            raise ValueError("Each link needs one angle.")

        transform = model_matrix(0.0, self.base[0], self.base[1])
        transforms: list[np.ndarray] = []

        for angle, link_length in zip(angles, self.link_lengths):
            link = model_matrix(angle, 0.0, 0.0) @ model_matrix(0.0, link_length, 0.0)
            transform = transform @ link
            transforms.append(transform)

        return transforms


def transform_position(transform: np.ndarray) -> tuple[float, float]:
    return float(transform[0, 2]), float(transform[1, 2])
