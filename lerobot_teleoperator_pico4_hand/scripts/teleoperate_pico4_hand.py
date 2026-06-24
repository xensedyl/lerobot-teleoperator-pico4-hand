import logging
import time
import traceback
from dataclasses import asdict, dataclass
from pprint import pformat
from typing import Any

import numpy as np
import rerun as rr

from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from lerobot_teleoperator_pico4_hand import Pico4Hand, Pico4HandConfig


@dataclass
class Pico4HandTeleoperateConfig:
    teleop: TeleoperatorConfig
    robot: RobotConfig
    fps: int = 30
    teleop_time_s: float | None = None
    display_data: bool = False
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    dryrun: bool = False


def _register_third_party_devices() -> None:
    try:
        from lerobot.utils.import_utils import register_third_party_plugins

        register_third_party_plugins()
    except ImportError:
        from lerobot.utils.import_utils import register_third_party_devices

        register_third_party_devices()


def _current_tcp_pose_7d(robot: Robot) -> np.ndarray:
    pose = robot.get_current_tcp_pose_quat()
    return np.asarray(pose[:7], dtype=np.float32)


def _initial_tcp_pose() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)


def _filter_robot_action(robot: Robot, action: RobotAction) -> RobotAction:
    action_features = getattr(robot, "action_features", {})
    if not action_features:
        return dict(action)
    return {key: value for key, value in action.items() if key in action_features}


def _connect_devices(robot: Robot, teleop: Teleoperator) -> None:
    if getattr(teleop, "name", None) != "pico4_hand":
        raise ValueError("lerobot-teleoperate-pico4-hand requires --teleop.type=pico4_hand.")

    robot_type = getattr(robot, "robot_type", None) or getattr(robot, "name", None) or robot.config.type

    if robot_type == "revo2_hand":
        robot.connect()
        if hasattr(teleop.config, "hand_only"):
            teleop.config.hand_only = True
        if getattr(teleop.config, "action_keys", None) is None and hasattr(robot.config, "action_keys"):
            teleop.config.action_keys = list(robot.config.action_keys)
        teleop.connect(current_tcp_pose_quat=_initial_tcp_pose())
        logging.info("Connected Revo2 hand and Pico4 hand tracking.")
        return

    # Arm + dexterous hand.  Start Pico first with a dummy pose so the headset
    # is ready while the arm moves to its start pose, then reset the TCP target
    # to the actual robot pose.
    teleop.connect(current_tcp_pose_quat=_initial_tcp_pose())
    logging.info("Connected Pico4 hand tracking.")
    try:
        robot.connect(go_to_start=True)
    except TypeError:
        robot.connect()

    if hasattr(robot, "get_current_tcp_pose_quat") and hasattr(teleop, "reset_to_pose"):
        start_pose = _current_tcp_pose_7d(robot)
        logging.info("Start EEF pose: %s", start_pose)
        teleop.reset_to_pose(start_pose)


def _sync_on_tracking_enable(robot: Robot, teleop: Teleoperator, was_enabled: bool) -> bool:
    enabled = bool(getattr(teleop, "_enabled", False))
    if enabled and not was_enabled and hasattr(robot, "get_current_tcp_pose_quat") and hasattr(teleop, "reset_to_pose"):
        try:
            teleop.reset_to_pose(_current_tcp_pose_7d(robot))
            logging.info("Pico4 hand teleop synced to robot TCP pose on tracking enable.")
        except Exception:
            logging.error("Failed to sync Pico4 hand teleop:\n%s", traceback.format_exc())
    return enabled


def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    display_data: bool = False,
    duration: float | None = None,
    display_compressed_images: bool = False,
    dryrun: bool = False,
) -> None:
    display_len = max(len(key) for key in robot.action_features) if robot.action_features else 1
    start = time.perf_counter()
    was_enabled = False

    while True:
        loop_start = time.perf_counter()

        obs = robot.get_observation()
        raw_action = teleop.get_action()
        was_enabled = _sync_on_tracking_enable(robot, teleop, was_enabled)

        teleop_action = teleop_action_processor((raw_action, obs))
        if getattr(teleop, "name", None) == "pico4_hand" and not was_enabled:
            teleop_action = {key: value for key, value in teleop_action.items() if not key.startswith("tcp.")}

        robot_action = robot_action_processor((_filter_robot_action(robot, teleop_action), obs))
        robot_action = _filter_robot_action(robot, robot_action)
        sent_action = robot_action if dryrun else robot.send_action(robot_action)

        if display_data:
            obs_transition = robot_observation_processor(obs)
            log_rerun_data(
                observation=obs_transition,
                action=sent_action,
                compress_images=display_compressed_images,
            )

            print("\n" + "-" * (display_len + 10))
            print(f"{'NAME':<{display_len}} | {'NORM':>7}")
            for motor, value in sent_action.items():
                print(f"{motor:<{display_len}} | {float(value):>7.2f}")
            move_cursor_up(len(sent_action) + 3)
        else:
            enabled_str = "ENABLED" if was_enabled else "DISABLED"
            tcp = [raw_action.get("tcp.x", 0.0), raw_action.get("tcp.y", 0.0), raw_action.get("tcp.z", 0.0)]
            print(
                f"\r\033[Ktime: {(time.perf_counter() - loop_start) * 1e3:.2f}ms | "
                f"{enabled_str} | tcp=[{tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f}]",
                end="",
                flush=True,
            )

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(1 / fps - dt_s, 0.0))

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate_pico4_hand(cfg: Pico4HandTeleoperateConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.teleop.type != "pico4_hand":
        raise ValueError("lerobot-teleoperate-pico4-hand requires --teleop.type=pico4_hand.")

    if cfg.display_data:
        init_rerun(session_name="pico4_hand_teleoperation", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    if not isinstance(cfg.teleop, Pico4HandConfig):
        raise ValueError("lerobot-teleoperate-pico4-hand requires Pico4HandConfig.")

    teleop = Pico4Hand(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    try:
        _connect_devices(robot, teleop)
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            display_compressed_images=display_compressed_images,
            dryrun=cfg.dryrun,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        try:
            if teleop.is_connected:
                teleop.disconnect()
        finally:
            if robot.is_connected:
                robot.disconnect()


def main() -> None:
    _register_third_party_devices()
    teleoperate_pico4_hand()
