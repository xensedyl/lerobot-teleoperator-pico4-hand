#!/usr/bin/env python

# Copyright 2025 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pico hand tracking teleoperator integrated with dex-retargeting.

This teleoperator is intentionally different from ``pico4``:

- ``pico4`` reads controller pose and outputs arm TCP actions.
- ``pico4_hand`` reads OpenXR hand-tracking data and outputs both arm TCP
  actions and dexterous-hand joint position actions.  There is no gripper:
  control is enabled whenever hand tracking is active.

Runtime data flow
-----------------
1. The Pico SDK returns 26 OpenXR hand joints in the headset/operator frame.
2. The palm pose at ``hand_state[0]`` replaces the old physical controller pose
   and drives the original TCP-control path:
     a. Filter out position jumps (VR tracking glitches).
     b. Check enable state — enabled whenever hand tracking is active.
     c. Apply window filter to the raw palm pose (in Pico/OpenXR frame).
     d. Transform from Pico/OpenXR coordinates to the robot TCP frame.
     e. On first enable / reference reset: record reference position and
        compute quaternion offset to align palm orientation with robot orientation.
     f. Compute relative position + absolute orientation target.
     g. Clamp output velocity (position and rotation rate limiters).
3. The joints are reordered to the 21-point MediaPipe/MANO convention used by
   dex-retargeting examples.
4. A wrist frame is estimated from the palm points, then points are rotated into
   MANO coordinates using dex-retargeting's OPERATOR2MANO matrix.
5. dex-retargeting optimizes robot qpos for the configured hand model.
6. qpos values for target_joint_names are mapped by name to LeRobot action keys.

Coordinate systems
------------------
- Pico/OpenXR: right-handed, X right, Y up, Z toward user.
  Origin is fixed at the headset position when the Unity app launches.
- Robot frame: right-handed, X forward (away from base), Y left, Z up.

For Revo2 this class has a built-in mapping.  For future hands, pass
``action_mapping`` or ``action_keys`` through ``Pico4HandConfig``.
"""

import time
import json
import os
from pathlib import Path
from queue import Queue
import subprocess
import sys
from typing import Any

import numpy as np

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import (
    get_logger,
    normalize_quaternion,
    quaternion_to_rotation_6d,
    slerp_quaternion,
)

from .config_pico4_hand import Pico4HandConfig

# OpenXR XR_EXT_hand_tracking 26 joints to MediaPipe-style 21 joints.
# OpenXR joint 0 is the palm (used for TCP control); joint 1 is the wrist
# (MediaPipe joint 0).  The non-thumb fingers keep metacarpal/proximal/
# intermediate/tip points, matching the convention used by the dex-retargeting
# Pico examples.
_OPENXR_TO_MEDIAPIPE = np.array(
    [
        1,
        2,
        3,
        4,
        5,
        7,
        8,
        9,
        10,
        12,
        13,
        14,
        15,
        17,
        18,
        19,
        20,
        22,
        23,
        24,
        25,
    ],
    dtype=int,
)

_REVO2_FEATURE_BY_JOINT_BASENAME = {
    "thumb_proximal": "th_prox",
    "thumb_metacarpal": "th_mcp",
    "index_proximal": "idx_prox",
    "middle_proximal": "mid_prox",
    "ring_proximal": "ring_prox",
    "pinky_proximal": "pky_prox",
}
_REVO2_ACTION_KEYS = {
    "left": [
        "l_th_prox.pos",
        "l_th_mcp.pos",
        "l_idx_prox.pos",
        "l_mid_prox.pos",
        "l_ring_prox.pos",
        "l_pky_prox.pos",
    ],
    "right": [
        "r_th_prox.pos",
        "r_th_mcp.pos",
        "r_idx_prox.pos",
        "r_mid_prox.pos",
        "r_ring_prox.pos",
        "r_pky_prox.pos",
    ],
}

_RETARGETING_WORKER_PATH = Path(__file__).resolve().parent / "pico4_hand_retargeting_worker.py"


class Pico4Hand(Teleoperator):
    """Read Pico hand tracking and return arm TCP plus robot-hand actions.

    Control scheme
    --------------
    - Hand tracking active: arm TCP control is enabled (no grip button needed).
    - Palm pose (hand_state[0]): drives 6-DoF arm TCP target.
    - Finger joints (hand_state[1:], via dex-retargeting): drive robot hand.

    Position control (relative accumulation):
    - When tracking becomes active, record palm position as reference.
    - target_pos = start_pos + (palm_pos - ref_pos) * pos_sensitivity

    Orientation control (absolute mapping with offset):
    - At enable time, compute offset = inv(palm_quat_robot) * robot_start_quat.
    - Each frame: target_quat = palm_quat_robot * offset.
    - This gives intuitive control: palm orientation maps directly to robot orientation.

    Output action format:
    - tcp.x, tcp.y, tcp.z: absolute TCP target position (meters) in robot frame.
    - tcp.r1–r6: absolute TCP target orientation (6D rotation) in robot frame.
    - <hand joint keys>: robot hand joint positions from dex-retargeting.
    """

    config_class = Pico4HandConfig
    name = "pico4_hand"

    def __init__(self, config: Pico4HandConfig):
        super().__init__(config)
        self.config = config
        self.logger = get_logger(f"Pico4HandTeleop/{config.id}")

        # The tracked human hand side and the target robot hand side must match
        # dex-retargeting configs.  If hand_type is omitted, infer from
        # use_left_controller / use_right_controller for CLI compatibility.
        self.hand_type = self._resolve_hand_type(config)

        # Pico SDK module handle.  Loaded lazily in connect() so importing this
        # teleoperator does not require the Pico runtime to be installed.
        self._xrt = None

        self._retargeting_worker: subprocess.Popen | None = None
        self._is_connected = False

        self._action_keys = self._build_initial_action_keys()

        # Last valid hand command, held when tracking drops for a few frames to
        # prevent sudden zero commands from snapping the hand open/closed.
        self._last_action = dict.fromkeys(self._action_keys, 0.0)

        self._last_tracking_loss_warn_time = 0.0
        self._last_retargeting_error_warn_time = 0.0

        # --- Arm TCP target state (in robot frame) ---
        self._target_pos: np.ndarray = np.zeros(3, dtype=np.float32)  # [x, y, z]
        self._target_quat: np.ndarray = np.array(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float32
        )  # [qw, qx, qy, qz]

        # Start pose captured at enable time (in robot frame).
        # Pos:  target_pos  = start_pos  + (palm_pos  - ref_pos)  * sensitivity
        # Ori:  target_quat = palm_quat_robot * quat_offset
        self._start_pos: np.ndarray = np.zeros(3, dtype=np.float32)  # [x, y, z]
        self._start_quat: np.ndarray = np.array(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float32
        )  # [qw, qx, qy, qz]

        # Reference position for relative position control.
        # Set to palm_pos (in robot frame) when control is first enabled.
        self._ref_pos: np.ndarray | None = None  # [x, y, z] in robot frame

        # Quaternion offset for absolute orientation mapping.
        # Calculated at enable: offset = inv(palm_quat_robot) * robot_start_quat
        # Applied each frame:  target_quat = palm_quat_robot * offset
        self._quat_offset: np.ndarray | None = None  # [qw, qx, qy, qz]

        # Window filter queues for raw palm pose data (before coordinate transform).
        # Simple moving-average for position; recursive SLERP for orientation.
        self._raw_pos_queue: Queue = Queue(
            self.config.filter_window_size
        )  # [x, y, z] Pico frame
        self._raw_quat_queue: Queue = Queue(
            self.config.filter_window_size
        )  # Raw quaternion [qx, qy, qz, qw] in Pico4 frame

        # Enable state.  Control is enabled whenever hand tracking is active —
        # no grip button required.
        self._enabled = False
        self._was_enabled = False  # Previous frame state for rising-edge detection
        self._orientation_control_active = (
            True  # Disabled if orientation offset is too large
        )

        # Position jump filter state.
        self._last_raw_pose: np.ndarray | None = None  # Last accepted raw pose
        self._jump_filter_count: int = (
            0  # Running count of filtered jumps (for debugging)
        )

        # Output rate-limiter state.
        self._last_action_time: float | None = (
            None  # Timestamp of last get_action() call
        )
        self._prev_target_pos: np.ndarray | None = (
            None  # Previous frame target position
        )
        self._prev_target_quat: np.ndarray | None = (
            None  # Previous frame target quaternion
        )

    @staticmethod
    def _resolve_hand_type(config: Pico4HandConfig) -> str:
        if config.hand_type is not None:
            return config.hand_type
        return "right" if config.use_right_controller else "left"

    def _build_initial_action_keys(self) -> list[str]:
        if self.config.action_keys is not None:
            return list(self.config.action_keys)
        if self.config.action_mapping:
            return list(dict.fromkeys(self.config.action_mapping.values()))
        if self.config.robot_name == "revo2":
            return list(_REVO2_ACTION_KEYS[self.hand_type])
        return []

    @property
    def is_connected(self) -> bool:
        """Check if the Pico4 VR headset is connected."""
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        """Pico4 doesn't require calibration."""
        return self._is_connected

    @property
    def action_features(self) -> dict[str, type]:
        """Return action features: arm TCP (9 values) + robot hand joints.

        - tcp.x, tcp.y, tcp.z: absolute TCP position (meters) in robot frame.
        - tcp.r1–r6: absolute TCP orientation (6D rotation) in robot frame.
        - <hand joint keys>: robot hand joint positions from dex-retargeting.
        """
        if self.config.hand_only:
            return dict.fromkeys(self._action_keys, float)

        features = {
            "tcp.x": float,
            "tcp.y": float,
            "tcp.z": float,
            "tcp.r1": float,
            "tcp.r2": float,
            "tcp.r3": float,
            "tcp.r4": float,
            "tcp.r5": float,
            "tcp.r6": float,
        }
        features.update(dict.fromkeys(self._action_keys, float))
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        """Pico4 doesn't support feedback."""
        return {}

    def connect(
        self,
        calibrate: bool = True,
        current_tcp_pose_quat: np.ndarray = np.zeros(7, dtype=np.float32),
    ) -> None:
        """Connect to the Pico SDK and start Pico hand tracking.

        Note: The Pico/OpenXR coordinate origin is fixed when the Unity app
        launches, not when xrt.init() is called and not when clicking connect
        in Unity.  The origin remains stable until Unity restarts.

        Args:
            calibrate: Unused; kept for Teleoperator interface compatibility.
            current_tcp_pose_quat: Current TCP pose [x, y, z, qw, qx, qy, qz]
                                   in robot frame (wxyz quaternion format).
        """
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.logger.info("Connecting to Pico4 VR headset...")
        try:
            import xensevr_pc_service_sdk as xrt
        except ImportError as e:
            raise ImportError(
                "xensevr_pc_service_sdk is required for Pico4 teleoperator. "
                "Please install it according to your Pico4 SDK documentation."
            ) from e

        self._start_retargeting_worker()

        self.logger.info(
            f"Connecting Pico SDK for {self.hand_type} hand tracking with "
            f"{self.config.robot_name}/{self.config.retargeting_type} retargeting..."
        )
        try:
            xrt.init()
            self._xrt = xrt
            self.logger.info("XenseVR SDK initialized successfully.")
            try:
                self._wait_for_hand_tracking()
            except DeviceNotConnectedError:
                self.logger.warn(
                    f"Pico {self.hand_type} hand tracking did not become active within "
                    f"{self.config.wait_for_tracking_timeout_s:.1f}s. "
                    "Continuing to wait for tracking in the teleop loop."
                )
        except RuntimeError as e:
            self._close_xrt()
            self._xrt = None
            self._close_retargeting_worker()
            raise RuntimeError(f"Failed to initialize XenseVR SDK: {e}") from e
        except DeviceNotConnectedError:
            self._close_xrt()
            self._xrt = None
            self._close_retargeting_worker()
            raise
        except Exception as e:
            self._close_xrt()
            self._xrt = None
            self._close_retargeting_worker()
            raise DeviceNotConnectedError(
                f"Failed to connect to Pico4 VR device: {e}. "
                f"Please ensure the Pico VR service is running and the device is connected."
            ) from e

        # Initialise target pose from the robot's current TCP pose.
        # Input format: [x, y, z, qw, qx, qy, qz] in robot frame.
        self._target_pos = current_tcp_pose_quat[:3].copy()
        self._target_quat = normalize_quaternion(
            current_tcp_pose_quat[3:7], input_format="wxyz"
        )
        self._start_pos = self._target_pos.copy()
        self._start_quat = self._target_quat.copy()
        self._ref_pos = None
        self._quat_offset = None
        self._enabled = False
        self._was_enabled = False
        self._orientation_control_active = True
        self._last_raw_pose = None
        self._prev_target_pos = self._target_pos.copy()
        self._prev_target_quat = self._target_quat.copy()
        self._last_action_time = None

        self._is_connected = True
        self.logger.info(f"{self} connected. action_keys={self._action_keys}")

    def _start_retargeting_worker(self) -> None:
        cmd = [
            sys.executable,
            str(_RETARGETING_WORKER_PATH),
            "--hand-type",
            self.hand_type,
            "--robot-name",
            self.config.robot_name,
            "--retargeting-type",
            self.config.retargeting_type,
            "--action-mapping-json",
            json.dumps(self.config.action_mapping),
        ]
        if self.config.retargeting_config_path is not None:
            cmd.extend(
                [
                    "--retargeting-config-path",
                    str(Path(self.config.retargeting_config_path).expanduser()),
                ]
            )
        if self.config.robot_urdf_dir is not None:
            cmd.extend(
                [
                    "--robot-urdf-dir",
                    str(Path(self.config.robot_urdf_dir).expanduser()),
                ]
            )

        env = dict(os.environ)
        # dex-retargeting's cmeel pinocchio/coal stack is sensitive to ROS and
        # other injected Python/C++ paths.  Run the worker in a clean env so it
        # resolves its own packaged binaries instead of mixed ROS/conda ones.
        env.pop("PYTHONPATH", None)
        env.pop("LD_LIBRARY_PATH", None)
        worker = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=env,
        )

        ready_line = worker.stdout.readline() if worker.stdout is not None else ""
        if not ready_line:
            stderr = worker.stderr.read() if worker.stderr is not None else ""
            worker.poll()
            raise RuntimeError(f"dex-retargeting worker failed to start: {stderr}")

        ready = json.loads(ready_line)
        if ready.get("status") != "ready":
            raise RuntimeError(f"Unexpected dex-retargeting worker response: {ready}")

        self._retargeting_worker = worker
        if self.config.action_keys is None:
            self._action_keys = list(ready.get("action_keys", []))
        self._last_action = {key: self._last_action.get(key, 0.0) for key in self._action_keys}
        self.logger.info(
            f"Loaded dex-retargeting worker: config={ready.get('config_path')}, "
            f"urdf_dir={ready.get('robot_urdf_dir')}, action_keys={self._action_keys}"
        )

    def calibrate(self) -> None:
        """No calibration needed for Pico4."""
        pass

    def configure(self) -> None:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

    def reset_to_pose(self, pose_7d: np.ndarray) -> None:
        """Reset the arm TCP target pose while keeping hand retargeting alive.

        Clears the reference pose and orientation offset so the next tracking
        frame will establish a fresh reference.

        Args:
            pose_7d: 7D EEF pose [x, y, z, qw, qx, qy, qz] in robot frame.
        """
        self._target_pos = np.array(pose_7d[:3], dtype=np.float32).copy()
        self._target_quat = normalize_quaternion(pose_7d[3:7], input_format="wxyz")

        # Clear reference/offset — recalculated on the next active tracking frame.
        self._ref_pos = None
        self._quat_offset = None

        # Reset enable state so the next frame is treated as a fresh enable.
        self._was_enabled = False
        self._enabled = False
        self._orientation_control_active = True

        # Clear filter queues to avoid stale data affecting the new control epoch.
        while not self._raw_pos_queue.empty():
            self._raw_pos_queue.get()
        while not self._raw_quat_queue.empty():
            self._raw_quat_queue.get()

        # Reset jump filter and rate-limiter baselines.
        self._last_raw_pose = None
        self._prev_target_pos = self._target_pos.copy()
        self._prev_target_quat = self._target_quat.copy()
        self._last_action_time = None

        self.logger.info(
            f"Reset target pose to: pos={pose_7d[:3]}, quat={pose_7d[3:7]}"
        )

    @staticmethod
    def _quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions q1 * q2.  Both in [qw, qx, qy, qz] format."""
        qw1, qx1, qy1, qz1 = q1
        qw2, qx2, qy2, qz2 = q2
        return np.array(
            [
                qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2,
                qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2,
                qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2,
                qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2,
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _quaternion_inverse(q: np.ndarray) -> np.ndarray:
        """Compute quaternion inverse.  Input and output in [qw, qx, qy, qz] format."""
        qw, qx, qy, qz = q
        norm_sq = qw * qw + qx * qx + qy * qy + qz * qz
        if norm_sq < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # identity
        return np.array([qw, -qx, -qy, -qz], dtype=np.float32) / norm_sq

    @staticmethod
    def _slerp_quaternion(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between two quaternions."""
        return slerp_quaternion(q1, q2, t, input_format="wxyz")

    def _filter_raw_pose(
        self, palm_pose_raw: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Apply window filter to the raw palm pose (Step 1 of TCP pipeline).

        For position: simple moving average (arithmetic mean).
        For orientation: recursive SLERP through the window.
        When filter_window_size=1 no filtering is applied (pass-through).

        Args:
            palm_pose_raw: Raw palm pose from SDK [x, y, z, qx, qy, qz, qw]
                           in Pico/OpenXR frame (SDK uses xyzw quaternion order).

        Returns:
            (filtered_pos, filtered_quat) still in Pico/OpenXR frame:
            - filtered_pos:  np.ndarray [x, y, z] in metres.
            - filtered_quat: np.ndarray [qw, qx, qy, qz] (converted to wxyz).
        """
        # Extract position; convert quaternion from SDK xyzw to internal wxyz.
        pos = palm_pose_raw[:3].copy()  # [x, y, z] in Pico frame
        quat = np.array(
            [
                palm_pose_raw[6],  # qw
                palm_pose_raw[3],  # qx
                palm_pose_raw[4],  # qy
                palm_pose_raw[5],  # qz
            ],
            dtype=np.float32,
        )
        quat = normalize_quaternion(quat, input_format="wxyz")

        # window_size=1 → skip filtering, return raw data directly.
        if self.config.filter_window_size <= 1:
            return pos, quat

        # Moving-average filter for position.
        if self._raw_pos_queue.full():
            self._raw_pos_queue.get()
        self._raw_pos_queue.put(pos)
        filtered_pos = np.mean(np.array(list(self._raw_pos_queue.queue)), axis=0)

        # SLERP-based filter for quaternion.
        # Split window into two halves, SLERP each half to its midpoint,
        # then SLERP the two midpoints for the final result.
        if self._raw_quat_queue.full():
            self._raw_quat_queue.get()
        self._raw_quat_queue.put(quat)

        quat_list = list(self._raw_quat_queue.queue)
        if len(quat_list) == 1:
            filtered_quat = quat_list[0]
        elif len(quat_list) == 2:
            filtered_quat = self._slerp_quaternion(quat_list[0], quat_list[1], 0.5)
        else:
            mid = len(quat_list) // 2
            left_half = quat_list[: mid + 1]
            right_half = quat_list[mid:]
            left_mid = (
                left_half[0]
                if len(left_half) == 1
                else self._slerp_quaternion(left_half[0], left_half[-1], 0.5)
            )
            right_mid = (
                right_half[0]
                if len(right_half) == 1
                else self._slerp_quaternion(right_half[0], right_half[-1], 0.5)
            )
            filtered_quat = self._slerp_quaternion(left_mid, right_mid, 0.5)

        return filtered_pos, normalize_quaternion(filtered_quat, input_format="wxyz")

    def _transform_pico_to_robot_coordinate(
        self, pos: np.ndarray, quat: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Transform pose from Pico/OpenXR frame to the robot TCP frame (Step 2).

        Pico/OpenXR: right-handed, X right, Y up, Z toward user.
        Robot frame: right-handed, X forward (away from base), Y left, Z up.

        Assuming the operator stands in front of and faces the robot:

          Pico/OpenXR                  Robot
              Y (up)                   Z (up)   X (forward)
              |                          |     /
              +--- X (right)   Y (left)--+
             /
            Z (toward user)

        Axis mapping:
            Pico X (right)        → Robot Y (left):    negate
            Pico Y (up)           → Robot Z (up):      same
            Pico Z (toward user)  → Robot X (forward): negate (opposite direction)

        Args:
            pos:  [x, y, z] in Pico/OpenXR frame.
            quat: [qw, qx, qy, qz] in Pico/OpenXR frame (from _filter_raw_pose).

        Returns:
            (transformed_pos, transformed_quat) in robot frame [qw, qx, qy, qz].
        """
        transformed_pos = np.array(
            [
                -pos[2],  # Pico Z (toward user) → Robot X (forward), negated
                -pos[0],  # Pico X (right)       → Robot Y (left),    negated
                pos[1],  # Pico Y (up)           → Robot Z (up),      same
            ],
            dtype=np.float32,
        )

        # Rotation equivalent of the position transform matrix:
        #   [ 0  0 -1]
        #   [-1  0  0]
        #   [ 0  1  0]
        # Quaternion: [cos(60°), sin(60°)·axis] = [0.5, 0.5, -0.5, -0.5]
        # Applied as: q_robot = q_R * q_pico * q_R⁻¹
        q_frame_transform = np.array([0.5, 0.5, -0.5, -0.5], dtype=np.float32)
        q_transform_inv = self._quaternion_inverse(q_frame_transform)
        q_temp = self._quaternion_multiply(q_frame_transform, quat)
        transformed_quat = self._quaternion_multiply(q_temp, q_transform_inv)
        return transformed_pos, normalize_quaternion(
            transformed_quat, input_format="wxyz"
        )

    def _wait_for_hand_tracking(self) -> None:
        deadline = time.time() + self.config.wait_for_tracking_timeout_s
        while True:
            active, hand_state = self._read_raw_hand_state()
            if self._hand_tracking_is_usable(active, hand_state):
                skeleton_usable = self._hand_state_has_position_data(hand_state)
                self.logger.info(
                    f"Pico {self.hand_type} hand tracking is active "
                    f"(skeleton_usable={skeleton_usable})."
                )
                return
            if time.time() >= deadline:
                raise DeviceNotConnectedError(
                    f"Pico {self.hand_type} hand tracking did not become active within "
                    f"{self.config.wait_for_tracking_timeout_s:.1f}s."
                )
            time.sleep(0.1)

    def _read_hand_state_for_type(self, hand_type: str) -> tuple[bool, np.ndarray]:
        if self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected")

        if hand_type == "right":
            active = bool(self._xrt.get_right_hand_is_active())
            hand_state = self._xrt.get_right_hand_tracking_state()
        else:
            active = bool(self._xrt.get_left_hand_is_active())
            hand_state = self._xrt.get_left_hand_tracking_state()
        return active, np.asarray(hand_state, dtype=float)

    def _read_raw_hand_state(self) -> tuple[bool, np.ndarray]:
        return self._read_hand_state_for_type(self.hand_type)

    @staticmethod
    def _hand_state_has_pose_data(hand_state: np.ndarray) -> bool:
        if hand_state.shape != (26, 7):
            return False
        positions = hand_state[:, :3]
        quats = hand_state[:, 3:7]
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quats)):
            return False
        if np.linalg.norm(positions[1]) < 1e-6:
            return False
        quat_norm = np.linalg.norm(quats[1])
        return bool(0.5 <= quat_norm <= 1.5)

    @staticmethod
    def _hand_state_has_position_data(hand_state: np.ndarray) -> bool:
        if hand_state.shape != (26, 7):
            return False
        positions = hand_state[:, :3]
        if not np.all(np.isfinite(positions)):
            return False
        if not np.any(np.abs(positions) > 1e-6):
            return False

        return bool(Pico4Hand._hand_skeleton_metrics(hand_state)["valid"])

    @staticmethod
    def _hand_skeleton_metrics(hand_state: np.ndarray) -> dict[str, Any]:
        metrics: dict[str, Any] = {
            "valid": False,
            "wrist_index_m": 0.0,
            "wrist_middle_m": 0.0,
            "index_middle_m": 0.0,
            "rank": 0,
        }
        if hand_state.shape != (26, 7):
            return metrics
        positions = hand_state[:, :3]
        if not np.all(np.isfinite(positions)):
            return metrics
        # Mirror the wrist-frame degeneracy check used by the retargeting worker.
        keypoints = positions[_OPENXR_TO_MEDIAPIPE] - positions[_OPENXR_TO_MEDIAPIPE[0]]
        metrics["wrist_index_m"] = float(np.linalg.norm(keypoints[5] - keypoints[0]))
        metrics["wrist_middle_m"] = float(np.linalg.norm(keypoints[9] - keypoints[0]))
        metrics["index_middle_m"] = float(np.linalg.norm(keypoints[9] - keypoints[5]))
        palm_points = keypoints[[0, 5, 9], :]
        x_vector = palm_points[0] - palm_points[2]
        centered = palm_points - np.mean(palm_points, axis=0, keepdims=True)
        try:
            _, _, vh = np.linalg.svd(centered)
        except np.linalg.LinAlgError:
            return metrics
        metrics["rank"] = int(np.linalg.matrix_rank(centered, tol=1e-6))
        normal = vh[2, :]
        x = x_vector - np.sum(x_vector * normal) * normal
        metrics["valid"] = bool(np.linalg.norm(x) >= 1e-6)
        return metrics

    def _hand_tracking_is_usable(self, active: bool, hand_state: np.ndarray) -> bool:
        has_position_data = self._hand_state_has_pose_data(hand_state)
        if self.config.allow_low_quality_tracking:
            return has_position_data
        return bool(active) and has_position_data

    def _handle_tracking_loss(
        self, active: bool | None = None, hand_state: np.ndarray | None = None
    ) -> dict[str, float]:
        now = time.time()
        if (
            now - self._last_tracking_loss_warn_time
            > self.config.tracking_warn_interval_s
        ):
            if active is None or hand_state is None:
                self.logger.warn(f"Pico {self.hand_type} hand tracking lost.")
            else:
                has_pose_data = self._hand_state_has_pose_data(hand_state)
                has_skeleton_data = self._hand_state_has_position_data(hand_state)
                metrics = self._hand_skeleton_metrics(hand_state)
                other_hand = "right" if self.hand_type == "left" else "left"
                other_hand_status = ""
                try:
                    other_active, other_state = self._read_hand_state_for_type(other_hand)
                    other_metrics = self._hand_skeleton_metrics(other_state)
                    other_hand_status = (
                        f", other_{other_hand}: active={bool(other_active)}, "
                        f"has_pose={self._hand_state_has_pose_data(other_state)}, "
                        f"has_skeleton={self._hand_state_has_position_data(other_state)}, "
                        f"wrist_index={other_metrics['wrist_index_m']:.4f}m, "
                        f"wrist_middle={other_metrics['wrist_middle_m']:.4f}m, "
                        f"rank={other_metrics['rank']}"
                    )
                except Exception:
                    other_hand_status = ""
                max_abs_pos = (
                    float(np.max(np.abs(hand_state[:, :3])))
                    if hand_state.ndim == 2 and hand_state.shape[1] >= 3
                    else 0.0
                )
                self.logger.warn(
                    f"Pico {self.hand_type} hand tracking lost "
                    f"(active={bool(active)}, has_pose_data={has_pose_data}, "
                    f"has_skeleton_data={has_skeleton_data}, shape={hand_state.shape}, "
                    f"max_abs_pos={max_abs_pos:.6f}, "
                    f"wrist_index={metrics['wrist_index_m']:.4f}m, "
                    f"wrist_middle={metrics['wrist_middle_m']:.4f}m, "
                    f"index_middle={metrics['index_middle_m']:.4f}m, "
                    f"rank={metrics['rank']}{other_hand_status})."
                )
            self._last_tracking_loss_warn_time = now
        if self.config.hold_last_action_on_tracking_loss:
            return dict(self._last_action)
        raise RuntimeError(f"Pico {self.hand_type} hand tracking lost.")

    def _tcp_action_from_palm_pose(
        self, palm_pose_raw: np.ndarray, hand_active: bool
    ) -> dict[str, float]:
        """Run the arm TCP control path using the tracked palm pose as input.

        Data processing pipeline:
        1. Filter position jumps caused by VR tracking glitches.
        2. Determine enable state (active whenever hand tracking is active).
        3. Apply window filter to the raw palm pose (in Pico/OpenXR frame).
        4. Transform from Pico/OpenXR to robot coordinate frame.
        5. Handle enable transitions: record reference pose and compute
           quaternion offset on the rising edge.
        6. If enabled: update target position (relative) and orientation (absolute).
        7. Apply output rate limiters for position and rotation.

        Args:
            palm_pose_raw: Palm pose [x, y, z, qx, qy, qz, qw] in Pico/OpenXR frame
                           (hand_state[0], SDK xyzw quaternion order).
            hand_active:   True when hand tracking is active this frame.

        Returns:
            Dict with tcp.x/y/z and tcp.r1–r6 in robot frame.
        """
        controller_pose_raw = np.array(palm_pose_raw, dtype=np.float32)

        # Step 1: Filter out position jumps (VR tracking glitches).
        if self._last_raw_pose is not None and self.config.position_jump_threshold > 0:
            pos_delta = np.linalg.norm(
                controller_pose_raw[:3] - self._last_raw_pose[:3]
            )
            if pos_delta > self.config.position_jump_threshold:
                self._jump_filter_count += 1
                self.logger.warn(
                    f"[JUMP] Palm position jump #{self._jump_filter_count}: "
                    f"delta={pos_delta:.4f}m > threshold={self.config.position_jump_threshold}m, "
                    f"raw_pos={controller_pose_raw[:3]}, last_pos={self._last_raw_pose[:3]}. "
                    f"Clamping to last frame. Auto-recovering next frame."
                )
                controller_pose_raw[:3] = self._last_raw_pose[:3]
                # Reset baseline so the next frame establishes a fresh reference
                # instead of permanently clamping translation.
                self._last_raw_pose = None
            else:
                self._last_raw_pose = controller_pose_raw.copy()
        else:
            self._last_raw_pose = controller_pose_raw.copy()

        # Step 2: Enable state — active whenever hand tracking is active.
        prev_enabled = self._enabled
        self._enabled = hand_active
        if self._enabled != prev_enabled:
            self.logger.debug(
                f"[ENABLE] State changed: {prev_enabled} -> {self._enabled}, "
                f"hand_active={hand_active}"
            )

        # Step 3: Apply window filter to raw palm pose (in Pico/OpenXR frame).
        raw_pos_pico = controller_pose_raw[:3].copy()
        filtered_pos_pico, filtered_quat_pico = self._filter_raw_pose(
            controller_pose_raw
        )
        filter_pos_delta = np.linalg.norm(filtered_pos_pico - raw_pos_pico)
        if filter_pos_delta > 0.01:
            self.logger.debug(
                f"[FILTER] Large palm filter delta: {filter_pos_delta:.4f}m, "
                f"raw={raw_pos_pico}, filtered={filtered_pos_pico}"
            )

        # Step 4: Transform from Pico/OpenXR to robot coordinate frame.
        filtered_pos_robot, filtered_quat_robot = (
            self._transform_pico_to_robot_coordinate(
                filtered_pos_pico, filtered_quat_pico
            )
        )

        # Step 5: Handle rising edge (just enabled) and reference/offset setup.
        just_enabled = self._enabled and not self._was_enabled
        if just_enabled:
            # Reset jump-filter baseline so the new enable epoch starts fresh.
            self._last_raw_pose = None

        if just_enabled or self._ref_pos is None:
            self.logger.debug(
                f"[REF_RESET] {'just_enabled' if just_enabled else 'ref_pos is None'}: "
                f"ref_pos={filtered_pos_robot}, start_pos={self._target_pos}, "
                f"filtered_quat={filtered_quat_robot}"
            )
            self._ref_pos = filtered_pos_robot.copy()
            self._start_pos = self._target_pos.copy()
            self._start_quat = self._target_quat.copy()

            # Orientation offset: offset = inv(palm_quat_robot) * robot_start_quat
            # Applied each frame: target_quat = palm_quat_robot * offset
            ref_quat_inv = self._quaternion_inverse(filtered_quat_robot)
            self._quat_offset = self._quaternion_multiply(
                ref_quat_inv, self._target_quat
            )
            self._quat_offset = normalize_quaternion(
                self._quat_offset, input_format="wxyz"
            )

            # Choose the shorter-path quaternion (|angle| <= 90°).
            offset_angle_rad = 2.0 * np.arccos(
                np.clip(abs(self._quat_offset[0]), 0.0, 1.0)
            )
            offset_angle_deg = np.degrees(offset_angle_rad)
            if offset_angle_deg > 90.0:
                self._quat_offset = normalize_quaternion(
                    -self._quat_offset, input_format="wxyz"
                )
                offset_angle_rad = 2.0 * np.arccos(
                    np.clip(self._quat_offset[0], 0.0, 1.0)
                )
                offset_angle_deg = np.degrees(offset_angle_rad)

            self.logger.debug(
                f"Orientation offset: "
                f"palm_quat={filtered_quat_robot}, robot_quat={self._target_quat}, "
                f"offset={self._quat_offset}, angle={offset_angle_deg:.1f}°"
            )

            if offset_angle_deg > self.config.orientation_offset_warning_deg:
                self.logger.warn(
                    f"Orientation offset too large: {offset_angle_deg:.1f}° > "
                    f"{self.config.orientation_offset_warning_deg}°. "
                    f"Orientation control DISABLED."
                )
                self._orientation_control_active = False
            else:
                self._orientation_control_active = True
                self.logger.debug(f"Orientation offset OK: {offset_angle_deg:.1f}°")

        self._was_enabled = self._enabled

        # Step 6: Update target pose when enabled.
        if self._enabled:
            # Position: relative accumulation from reference point.
            rel_pos = filtered_pos_robot - self._ref_pos
            self._target_pos = self._start_pos + rel_pos * self.config.pos_sensitivity

            # Orientation: absolute mapping with offset.
            # target_quat = palm_quat_robot * offset
            if self._orientation_control_active and self._quat_offset is not None:
                full_target_quat = self._quaternion_multiply(
                    filtered_quat_robot, self._quat_offset
                )
                full_target_quat = normalize_quaternion(
                    full_target_quat, input_format="wxyz"
                )
                if self.config.ori_sensitivity < 1.0:
                    # SLERP: ori_sensitivity=0.5 → half-speed tracking.
                    self._target_quat = self._slerp_quaternion(
                        self._start_quat, full_target_quat, self.config.ori_sensitivity
                    )
                else:
                    self._target_quat = full_target_quat
            # If orientation control is disabled, target_quat stays at its value
            # from when tracking was last enabled.
        # When disabled, target pose holds its last value (no update).

        # Step 7: Output rate limiters — clamp target_pos / target_quat velocity.
        now = time.time()
        if self._prev_target_pos is not None and self._last_action_time is not None:
            dt = now - self._last_action_time
            if dt > 0:
                # Position rate limit.
                if self.config.max_pos_velocity > 0:
                    max_delta = self.config.max_pos_velocity * dt
                    delta_pos = self._target_pos - self._prev_target_pos
                    delta_norm = np.linalg.norm(delta_pos)
                    if delta_norm > max_delta:
                        self.logger.warn(
                            f"[RATE_LIMIT] Position velocity {delta_norm / dt:.2f} m/s "
                            f"exceeds limit {self.config.max_pos_velocity} m/s, clamping."
                        )
                        self._target_pos = self._prev_target_pos + delta_pos * (
                            max_delta / delta_norm
                        )

                # Rotation rate limit.
                if (
                    self.config.max_rot_velocity > 0
                    and self._prev_target_quat is not None
                ):
                    max_angle = self.config.max_rot_velocity * dt
                    # Angle between two quaternions: θ = 2·arccos(|q1·q2|)
                    dot = np.clip(
                        abs(np.dot(self._target_quat, self._prev_target_quat)), 0.0, 1.0
                    )
                    angle = 2.0 * np.arccos(dot)
                    if angle > max_angle:
                        self.logger.warn(
                            f"[RATE_LIMIT] Rotation velocity {np.degrees(angle / dt):.1f} deg/s "
                            f"exceeds limit {np.degrees(self.config.max_rot_velocity):.1f} deg/s, clamping."
                        )
                        self._target_quat = self._slerp_quaternion(
                            self._prev_target_quat, self._target_quat, max_angle / angle
                        )

        self._prev_target_pos = self._target_pos.copy()
        self._prev_target_quat = self._target_quat.copy()
        self._last_action_time = now

        r6d = quaternion_to_rotation_6d(
            self._target_quat[0],
            self._target_quat[1],
            self._target_quat[2],
            self._target_quat[3],
        )
        self.logger.debug(
            f"[ACTION] pos=[{self._target_pos[0]:.4f}, {self._target_pos[1]:.4f}, {self._target_pos[2]:.4f}], "
            f"quat=[{self._target_quat[0]:.3f}, {self._target_quat[1]:.3f}, "
            f"{self._target_quat[2]:.3f}, {self._target_quat[3]:.3f}], "
            f"enabled={self._enabled}, ori_active={self._orientation_control_active}"
        )
        return {
            "tcp.x": float(self._target_pos[0]),
            "tcp.y": float(self._target_pos[1]),
            "tcp.z": float(self._target_pos[2]),
            "tcp.r1": float(r6d[0]),
            "tcp.r2": float(r6d[1]),
            "tcp.r3": float(r6d[2]),
            "tcp.r4": float(r6d[3]),
            "tcp.r5": float(r6d[4]),
            "tcp.r6": float(r6d[5]),
        }

    def get_action(self) -> dict[str, Any]:
        """Get the current arm TCP target and robot hand joint positions.

        Data processing pipeline:
        1. Read hand tracking state from the Pico SDK.
        2. If tracking is lost, hold the last hand action and return the
           current TCP target unchanged.
        3. Run the arm TCP control path using the palm pose (hand_state[0]).
        4. Convert hand joints to MANO convention and run dex-retargeting.
        5. Map retargeted qpos to LeRobot action keys and return the combined action.

        Returns:
            Dict with tcp.x/y/z, tcp.r1–r6, and all robot hand joint keys.
        """
        if not self._is_connected or self._xrt is None or self._retargeting_worker is None:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # Step 1: Read hand tracking state.
        active, hand_state = self._read_raw_hand_state()
        tracking_usable = self._hand_tracking_is_usable(active, hand_state)
        skeleton_usable = self._hand_state_has_position_data(hand_state)

        # Step 2: Handle tracking loss — hold last hand action, keep TCP target.
        if not tracking_usable:
            hand_action = self._handle_tracking_loss(active, hand_state)
            return {**self._current_tcp_action(), **hand_action}

        # Step 3: Arm TCP control from palm pose (hand_state[0]) unless this is
        # a standalone hand teleoperator with no arm target.
        if self.config.hand_only:
            tcp_action = {}
        else:
            tcp_action = self._tcp_action_from_palm_pose(hand_state[1], tracking_usable)

        # Step 4: Run dex-retargeting in the worker process.
        if skeleton_usable:
            hand_action = self._hand_action_from_worker(hand_state)
        else:
            hand_action = self._handle_tracking_loss(active, hand_state)

        return {**tcp_action, **hand_action}

    def _hand_action_from_worker(self, hand_state: np.ndarray) -> dict[str, float]:
        worker = self._retargeting_worker
        if worker is None or worker.stdin is None or worker.stdout is None:
            raise RuntimeError("dex-retargeting worker is not available.")

        worker.stdin.write(json.dumps({"hand_state": hand_state.tolist()}) + "\n")
        worker.stdin.flush()
        response_line = worker.stdout.readline()
        if not response_line:
            stderr = worker.stderr.read() if worker.stderr is not None else ""
            if "KeyboardInterrupt" in stderr:
                raise KeyboardInterrupt
            raise RuntimeError(f"dex-retargeting worker stopped: {stderr}")

        response = json.loads(response_line)
        if response.get("status") != "ok":
            error = str(response.get("error", response)).strip()
            now = time.time()
            if now - self._last_retargeting_error_warn_time > self.config.tracking_warn_interval_s:
                first_line = error.splitlines()[-1] if error else response
                self.logger.warn(
                    "Pico hand retargeting failed; holding last hand command: "
                    f"{first_line}"
                )
                self._last_retargeting_error_warn_time = now
            if self.config.hold_last_action_on_tracking_loss:
                return dict(self._last_action)
            raise RuntimeError(error)

        hand_action = {key: float(value) for key, value in response["action"].items()}
        if not all(np.isfinite(value) for value in hand_action.values()):
            self.logger.warn(
                f"Non-finite retargeted hand action, holding last command: {hand_action}"
            )
            return dict(self._last_action)

        self._last_action = hand_action
        return hand_action

    def _current_tcp_action(self) -> dict[str, float]:
        """Return the current TCP target as an action dict (no SDK read)."""
        r6d = quaternion_to_rotation_6d(
            self._target_quat[0],
            self._target_quat[1],
            self._target_quat[2],
            self._target_quat[3],
        )
        return {
            "tcp.x": float(self._target_pos[0]),
            "tcp.y": float(self._target_pos[1]),
            "tcp.z": float(self._target_pos[2]),
            "tcp.r1": float(r6d[0]),
            "tcp.r2": float(r6d[1]),
            "tcp.r3": float(r6d[2]),
            "tcp.r4": float(r6d[3]),
            "tcp.r5": float(r6d[4]),
            "tcp.r6": float(r6d[5]),
        }

    def get_target_pose_array(self) -> np.ndarray:
        """Return the current TCP target as a 7D array [x, y, z, qw, qx, qy, qz]
        in robot frame, for direct use with the robot SDK."""
        return np.array(
            [
                self._target_pos[0],
                self._target_pos[1],
                self._target_pos[2],
                self._target_quat[0],  # qw
                self._target_quat[1],  # qx
                self._target_quat[2],  # qy
                self._target_quat[3],  # qz
            ],
            dtype=np.float32,
        )

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError(
            "Feedback is not implemented for Pico4 hand teleoperator."
        )

    def _close_xrt(self) -> None:
        if self._xrt is not None and hasattr(self._xrt, "close"):
            self._xrt.close()

    def disconnect(self) -> None:
        """Disconnect from the Pico SDK and release dex-retargeting resources."""
        if not self._is_connected and self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected")
        self.logger.info("Closing XenseVR SDK...")
        try:
            self._close_xrt()
        finally:
            self._xrt = None
            self._close_retargeting_worker()
            self._is_connected = False
            self.logger.info(f"{self} disconnected.")

    def _close_retargeting_worker(self) -> None:
        worker = self._retargeting_worker
        self._retargeting_worker = None
        if worker is None:
            return

        try:
            if worker.stdin is not None and worker.poll() is None:
                worker.stdin.write(json.dumps({"command": "close"}) + "\n")
                worker.stdin.flush()
        except Exception:
            pass

        try:
            worker.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            worker.terminate()
            try:
                worker.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                worker.kill()

    def __del__(self):
        if getattr(self, "_is_connected", False):
            try:
                self.disconnect()
            except Exception:
                self._is_connected = False
                self._xrt = None
                self._close_retargeting_worker()
