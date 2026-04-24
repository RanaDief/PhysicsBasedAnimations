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

    def points(self, angles: list[float]) -> list[tuple[float, float]]:
        transforms = self.solve(angles)
        return [self.base] + [transform_position(transform) for transform in transforms]


class CCDInverseKinematicsSolver:
    """Cyclic coordinate descent solver for a planar kinematic chain."""

    def __init__(self, chain: ForwardKinematicsChain, iterations: int = 10) -> None:
        self.chain = chain
        self.iterations = iterations

    def solve(
        self,
        angles: list[float],
        target: tuple[float, float],
    ) -> list[float]:
        if len(angles) != len(self.chain.link_lengths):
            raise ValueError("Each link needs one angle.")

        solved_angles = angles.copy()
        target_vector = np.array(target)

        for _ in range(self.iterations):
            for index in reversed(range(len(solved_angles))):
                points = self.chain.points(solved_angles)
                joint = np.array(points[index])
                end = np.array(points[-1])

                end_vector = end - joint
                target_direction = target_vector - joint

                if np.linalg.norm(end_vector) == 0.0 or np.linalg.norm(target_direction) == 0.0:
                    continue

                end_angle = math.atan2(end_vector[1], end_vector[0])
                target_angle = math.atan2(target_direction[1], target_direction[0])
                solved_angles[index] += target_angle - end_angle

        return solved_angles


def transform_position(transform: np.ndarray) -> tuple[float, float]:
    return float(transform[0, 2]), float(transform[1, 2])
