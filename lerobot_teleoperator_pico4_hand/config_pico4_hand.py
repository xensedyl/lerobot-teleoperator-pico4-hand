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

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("pico4_hand")
@dataclass
class Pico4HandConfig(TeleoperatorConfig):
    """Pico hand-tracking teleoperator using dex-retargeting.

    Outputs both arm TCP actions (driven by the tracked palm pose) and
    dexterous-hand joint position actions (from dex-retargeting).
    There is no gripper: control is enabled whenever hand tracking is active.
    """

    id: str = "pico4_hand"

    # Hand-tracking source.  If hand_type is None, the legacy controller flags
    # below are used to infer left/right for backwards-compatible CLI configs.
    hand_type: str | None = None
    use_left_controller: bool = True
    use_right_controller: bool = False

    # dex-retargeting target.  The default is Revo2, but custom hands can be
    # added by supplying a config path plus an action mapping.
    robot_name: str = "revo2"
    retargeting_type: str = "vector"
    retargeting_config_path: Path | None = None
    robot_urdf_dir: Path | None = None

    # Maps dex-retargeting target_joint_names to LeRobot action keys.  Revo2
    # has a built-in mapping; other hands should provide either this or action_keys.
    action_mapping: dict[str, str] = field(default_factory=dict)
    action_keys: list[str] | None = None

    # Hand-only mode is used for standalone dexterous hands such as Revo2Hand.
    # In this mode get_action() returns only retargeted hand joint keys.
    hand_only: bool = False

    hold_last_action_on_tracking_loss: bool = True
    wait_for_tracking_timeout_s: float = 8.0
    tracking_warn_interval_s: float = 1.0
    allow_low_quality_tracking: bool = True

    # Arm TCP control.  The pose source is the tracked palm pose (hand_state[0]).
    # pos/ori_sensitivity scale the relative movement / orientation response.
    # filter_window_size=1 disables filtering (pass-through mode).
    pos_sensitivity: float = 1.5
    ori_sensitivity: float = 1.0
    filter_window_size: int = 1
    # Disable orientation control if the palm-to-robot angular offset exceeds this.
    orientation_offset_warning_deg: float = 180.0
    # Discard frames where the palm jumped more than this distance (meters).
    # Set to 0 to disable.
    position_jump_threshold: float = 0.1
    # Clamp output velocity to prevent unsafe robot motion.
    max_pos_velocity: float = 1.0   # m/s
    max_rot_velocity: float = 6.28  # rad/s

    def __post_init__(self):
        robot_name_aliases = {
            "revo2_hand": "revo2",
            "schunk_svh_hand": "svh",
            "panda_gripper": "panda",
        }
        self.robot_name = robot_name_aliases.get(self.robot_name.lower(), self.robot_name.lower())
        self.retargeting_type = self.retargeting_type.lower()
        if self.hand_type is not None:
            self.hand_type = self.hand_type.lower()
        if self.hand_type is not None and self.hand_type not in {"left", "right"}:
            raise ValueError(
                f"hand_type must be 'left', 'right', or None, got {self.hand_type!r}"
            )
        if (
            self.hand_type is None
            and self.use_left_controller == self.use_right_controller
        ):
            raise ValueError(
                "Set hand_type, or set exactly one of use_left_controller/use_right_controller."
            )
        if self.retargeting_type not in {"vector", "position", "dexpilot"}:
            raise ValueError(
                "retargeting_type must be 'vector', 'position', or 'dexpilot', "
                f"got {self.retargeting_type!r}"
            )
        if self.wait_for_tracking_timeout_s < 0:
            raise ValueError("wait_for_tracking_timeout_s must be non-negative")
        if self.tracking_warn_interval_s <= 0:
            raise ValueError("tracking_warn_interval_s must be positive")
        if self.filter_window_size < 1:
            raise ValueError("filter_window_size must be >= 1")
