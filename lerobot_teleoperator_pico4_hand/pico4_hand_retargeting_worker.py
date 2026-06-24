#!/usr/bin/env python

import argparse
import json
import traceback
from pathlib import Path

import numpy as np

# pinocchio's Python bindings in this environment require the collision
# wrapper module to be initialized first.
try:
    import coal  # noqa: F401
except Exception:
    try:
        import hppfcl  # noqa: F401
    except Exception:
        pass


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


def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
    if keypoint_3d_array.shape != (21, 3):
        raise ValueError(
            f"Expected hand keypoints shape (21, 3), got {keypoint_3d_array.shape}."
        )

    points = keypoint_3d_array[[0, 5, 9], :]
    x_vector = points[0] - points[2]

    centered = points - np.mean(points, axis=0, keepdims=True)
    _, _, vh = np.linalg.svd(centered)
    normal = vh[2, :]

    x = x_vector - np.sum(x_vector * normal) * normal
    x_norm = np.linalg.norm(x)
    if x_norm < 1e-8:
        raise ValueError("Degenerate Pico hand keypoints: cannot estimate wrist frame.")
    x = x / x_norm
    z = np.cross(x, normal)

    if np.sum(z * (centered[1] - centered[2])) < 0:
        normal *= -1
        z *= -1

    return np.stack([x, normal, z], axis=1)


def hand_state_to_mano_joint_positions(hand_state: np.ndarray, hand_type: str) -> np.ndarray:
    from dex_retargeting.constants import OPERATOR2MANO_LEFT, OPERATOR2MANO_RIGHT

    if hand_state.shape != (26, 7):
        raise ValueError(f"Expected Pico hand_state shape (26, 7), got {hand_state.shape}.")

    keypoints = hand_state[_OPENXR_TO_MEDIAPIPE, :3]
    keypoints = keypoints - keypoints[0:1, :]
    wrist_rot = estimate_frame_from_hand_points(keypoints)
    operator2mano = OPERATOR2MANO_RIGHT if hand_type == "right" else OPERATOR2MANO_LEFT
    return keypoints @ wrist_rot @ operator2mano


def compute_ref_value(retargeting, joint_pos: np.ndarray) -> np.ndarray:
    retargeting_type = retargeting.optimizer.retargeting_type
    indices = retargeting.optimizer.target_link_human_indices
    if retargeting_type == "POSITION":
        return joint_pos[indices, :]
    origin_indices = indices[0, :]
    task_indices = indices[1, :]
    return joint_pos[task_indices, :] - joint_pos[origin_indices, :]


def normalize_robot_name(robot_name: str) -> str:
    aliases = {
        "revo2_hand": "revo2",
        "schunk_svh_hand": "svh",
        "panda_gripper": "panda",
    }
    return aliases.get(robot_name, robot_name)


def infer_robot_urdf_dir(config_path: Path) -> Path:
    for parent in config_path.resolve().parents:
        candidate = parent / "assets" / "robots" / "hands"
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        "Could not infer dex-retargeting robot URDF directory. Set teleop.robot_urdf_dir explicitly."
    )


def revo2_action_key_from_joint_name(joint_name: str, hand_type: str) -> str:
    prefix = "l" if hand_type == "left" else "r"
    side_prefix = f"{hand_type}_"
    basename = joint_name.removesuffix("_joint")
    if basename.startswith(side_prefix):
        basename = basename[len(side_prefix) :]
    try:
        return f"{prefix}_{_REVO2_FEATURE_BY_JOINT_BASENAME[basename]}.pos"
    except KeyError as e:
        raise ValueError(f"Unsupported Revo2 retargeting joint name: {joint_name}") from e


def build_action_mapping(
    ctrl_joint_names: list[str],
    hand_type: str,
    robot_name: str,
    action_mapping: dict[str, str],
) -> tuple[dict[str, str], list[str]]:
    if action_mapping:
        resolved = dict(action_mapping)
    elif robot_name == "revo2":
        resolved = {
            joint_name: revo2_action_key_from_joint_name(joint_name, hand_type)
            for joint_name in ctrl_joint_names
        }
    else:
        resolved = {joint_name: f"{joint_name}.pos" for joint_name in ctrl_joint_names}

    action_keys = [
        resolved[joint_name]
        for joint_name in ctrl_joint_names
        if joint_name in resolved
    ]
    return resolved, action_keys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hand-type", required=True, choices=("left", "right"))
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--retargeting-type", required=True, choices=("vector", "position", "dexpilot"))
    parser.add_argument("--retargeting-config-path", default=None)
    parser.add_argument("--robot-urdf-dir", default=None)
    parser.add_argument("--action-mapping-json", default="{}")
    args = parser.parse_args()

    from dex_retargeting.constants import HandType, RetargetingType, RobotName, get_default_config_path
    from dex_retargeting.retargeting_config import RetargetingConfig

    robot_name = normalize_robot_name(args.robot_name.lower())
    retargeting_type = args.retargeting_type.lower()
    hand_type = args.hand_type

    if args.retargeting_config_path is not None:
        config_path = Path(args.retargeting_config_path).expanduser()
    else:
        config_path = get_default_config_path(
            RobotName[robot_name],
            RetargetingType[retargeting_type],
            HandType[hand_type],
        )

    if config_path is None or not config_path.exists():
        raise FileNotFoundError(f"dex-retargeting config not found: {config_path}")

    robot_urdf_dir = (
        Path(args.robot_urdf_dir).expanduser()
        if args.robot_urdf_dir is not None
        else infer_robot_urdf_dir(config_path)
    )
    RetargetingConfig.set_default_urdf_dir(str(robot_urdf_dir))
    retargeting = RetargetingConfig.load_from_file(config_path).build()

    joint_names = list(retargeting.joint_names)
    ctrl_joint_names = list(retargeting.optimizer.target_joint_names)
    ctrl_indices = [joint_names.index(name) for name in ctrl_joint_names]
    action_mapping, action_keys = build_action_mapping(
        ctrl_joint_names=ctrl_joint_names,
        hand_type=hand_type,
        robot_name=robot_name,
        action_mapping=json.loads(args.action_mapping_json),
    )

    print(
        json.dumps(
            {
                "status": "ready",
                "config_path": str(config_path),
                "robot_urdf_dir": str(robot_urdf_dir),
                "ctrl_joint_names": ctrl_joint_names,
                "action_keys": action_keys,
            }
        ),
        flush=True,
    )

    try:
        while True:
            line = input()
            request = json.loads(line)
            if request.get("command") == "close":
                break

            try:
                hand_state = np.asarray(request["hand_state"], dtype=float)
                joint_pos = hand_state_to_mano_joint_positions(hand_state, hand_type)
                ref_value = compute_ref_value(retargeting, joint_pos)
                qpos = np.asarray(retargeting.retarget(ref_value), dtype=float)
                ctrl_positions = np.asarray(qpos[ctrl_indices], dtype=float)

                action = {}
                for joint_name, value in zip(ctrl_joint_names, ctrl_positions, strict=True):
                    action_key = action_mapping.get(joint_name)
                    if action_key is not None:
                        action[action_key] = float(value)

                print(json.dumps({"status": "ok", "action": action}), flush=True)
            except Exception:
                print(
                    json.dumps(
                        {
                            "status": "error",
                            "error": traceback.format_exc(),
                        }
                    ),
                    flush=True,
                )
    except (EOFError, KeyboardInterrupt):
        pass
    except Exception:
        print(
            json.dumps(
                {
                    "status": "error",
                    "error": traceback.format_exc(),
                }
            ),
            flush=True,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
