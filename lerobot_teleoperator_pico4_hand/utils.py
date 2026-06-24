#!/usr/bin/env python

import logging

import numpy as np


def get_logger(name: str) -> logging.Logger:
    """Return a package logger without depending on LeRobot private helpers."""

    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter("[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s")
    )
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    return logger


def normalize_quaternion(q: np.ndarray, input_format: str = "wxyz") -> np.ndarray:
    """Normalize a quaternion and return it in [qw, qx, qy, qz] format."""

    q = np.asarray(q, dtype=np.float32).reshape(-1)
    if len(q) != 4:
        raise ValueError(f"Quaternion must have 4 components, got {len(q)}")

    norm = np.linalg.norm(q)
    if norm < 1e-10:
        if input_format == "wxyz":
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        if input_format == "xyzw":
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        raise ValueError(f"Unknown input_format: {input_format!r}")

    if abs(norm - 1.0) > 1e-6:
        q = q / norm

    if input_format == "wxyz":
        return q.astype(np.float32)
    if input_format == "xyzw":
        return np.array([q[3], q[0], q[1], q[2]], dtype=np.float32)
    raise ValueError(f"Unknown input_format: {input_format!r}")


def slerp_quaternion(
    q1: np.ndarray, q2: np.ndarray, t: float, input_format: str = "wxyz"
) -> np.ndarray:
    """Spherical linear interpolation between two quaternions."""

    q1 = normalize_quaternion(q1, input_format=input_format)
    q2 = normalize_quaternion(q2, input_format=input_format)

    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))

    if dot > 0.9995:
        return normalize_quaternion(q1 + t * (q2 - q1), input_format="wxyz")

    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    w1 = np.sin((1.0 - t) * theta) / sin_theta
    w2 = np.sin(t * theta) / sin_theta
    return normalize_quaternion(w1 * q1 + w2 * q2, input_format="wxyz")


def quaternion_to_rotation_6d(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Convert [qw, qx, qy, qz] quaternion to the first two rotation columns."""

    r1 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r2 = 2.0 * (qx * qy + qz * qw)
    r3 = 2.0 * (qx * qz - qy * qw)

    r4 = 2.0 * (qx * qy - qz * qw)
    r5 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r6 = 2.0 * (qy * qz + qx * qw)

    return np.array([r1, r2, r3, r4, r5, r6], dtype=np.float32)
