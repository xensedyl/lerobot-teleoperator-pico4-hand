import logging
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat

import rerun as rr

from lerobot.cameras import CameraConfig  # noqa: F401
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
try:
    from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
except ModuleNotFoundError as exc:
    if exc.name not in {"lerobot.datasets.pipeline_features", "lerobot.processor"}:
        raise
    aggregate_pipeline_dataset_features = None
    create_initial_features = None

from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from lerobot_teleoperator_pico4_hand import Pico4Hand, Pico4HandConfig
from lerobot_teleoperator_pico4_hand._lerobot_compat import (
    PROCESSOR_API_AVAILABLE,
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)

from .teleoperate_pico4_hand import (
    _connect_devices,
    _filter_robot_action,
    _register_third_party_devices,
    _sync_on_tracking_enable,
)


@dataclass
class Pico4HandDatasetRecordConfig:
    repo_id: str
    single_task: str
    root: str | Path | None = None
    fps: int = 30
    episode_time_s: int | float = 60
    reset_time_s: int | float = 60
    num_episodes: int = 50
    video: bool = True
    push_to_hub: bool = True
    private: bool = False
    tags: list[str] | None = None
    num_image_writer_processes: int = 0
    num_image_writer_threads_per_camera: int = 4
    video_encoding_batch_size: int = 1
    vcodec: str = "libsvtav1"
    rename_map: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.single_task is None:
            raise ValueError("You need to provide --dataset.single_task.")


@dataclass
class Pico4HandRecordConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    dataset: Pico4HandDatasetRecordConfig
    display_data: bool = False
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    resume: bool = False


def _disconnect_recording_devices(robot: Robot, teleop: Teleoperator | None) -> None:
    if robot.is_connected:
        robot.disconnect()
    if teleop is not None and teleop.is_connected:
        teleop.disconnect()


def _build_dataset_features(
    robot: Robot,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    use_videos: bool,
) -> dict[str, dict]:
    if PROCESSOR_API_AVAILABLE and aggregate_pipeline_dataset_features and create_initial_features:
        return combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=teleop_action_processor,
                initial_features=create_initial_features(action=robot.action_features),
                use_videos=use_videos,
            ),
            aggregate_pipeline_dataset_features(
                pipeline=robot_observation_processor,
                initial_features=create_initial_features(observation=robot.observation_features),
                use_videos=use_videos,
            ),
        )

    return combine_feature_dicts(
        hw_to_dataset_features(robot.action_features, ACTION, use_videos),
        hw_to_dataset_features(robot.observation_features, OBS_STR, use_videos),
    )


@safe_stop_image_writer
def record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    dataset: LeRobotDataset | None,
    teleop: Teleoperator,
    control_time_s: int | float,
    single_task: str,
    display_data: bool = False,
    display_compressed_images: bool = False,
) -> None:
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    timestamp = 0.0
    start_episode_t = time.perf_counter()
    was_enabled = False

    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        obs = robot.get_observation()
        obs_processed = robot_observation_processor(obs)
        observation_frame = (
            build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)
            if dataset is not None
            else {}
        )

        raw_action = teleop.get_action()
        was_enabled = _sync_on_tracking_enable(robot, teleop, was_enabled)

        action_values = teleop_action_processor((raw_action, obs_processed))
        if getattr(teleop, "name", None) == "pico4_hand" and not was_enabled:
            action_values = {key: value for key, value in action_values.items() if not key.startswith("tcp.")}

        robot_action = robot_action_processor((_filter_robot_action(robot, action_values), obs_processed))
        robot_action = _filter_robot_action(robot, robot_action)
        sent_action = robot.send_action(robot_action)

        if dataset is not None:
            action_frame = build_dataset_frame(dataset.features, sent_action, prefix=ACTION)
            dataset.add_frame({**observation_frame, **action_frame, "task": single_task})

        if display_data:
            log_rerun_data(
                observation=obs_processed,
                action=sent_action,
                compress_images=display_compressed_images,
            )

        dt_s = time.perf_counter() - start_loop_t
        precise_sleep(max(1 / fps - dt_s, 0.0))
        timestamp = time.perf_counter() - start_episode_t


@parser.wrap()
def record_pico4_hand(cfg: Pico4HandRecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.teleop.type != "pico4_hand":
        raise ValueError("lerobot-record-pico4-hand requires --teleop.type=pico4_hand.")

    if cfg.display_data:
        init_rerun(session_name="pico4_hand_recording", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    robot = make_robot_from_config(cfg.robot)
    if not isinstance(cfg.teleop, Pico4HandConfig):
        raise ValueError("lerobot-record-pico4-hand requires Pico4HandConfig.")

    teleop = Pico4Hand(cfg.teleop)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    dataset_features = _build_dataset_features(
        robot=robot,
        teleop_action_processor=teleop_action_processor,
        robot_observation_processor=robot_observation_processor,
        use_videos=cfg.dataset.video,
    )

    dataset = None
    listener = None
    try:
        if cfg.resume:
            dataset = LeRobotDataset(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
            )
            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer(
                    num_processes=cfg.dataset.num_image_writer_processes,
                    num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                )
            sanity_check_dataset_robot_compatibility(
                dataset, robot, cfg.dataset.fps, dataset_features
            )
        else:
            sanity_check_dataset_name(cfg.dataset.repo_id, None)
            dataset = LeRobotDataset.create(
                cfg.dataset.repo_id,
                cfg.dataset.fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
            )

        _connect_devices(robot, teleop)
        listener, events = init_keyboard_listener()

        with VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=cfg.dataset.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    dataset=dataset,
                    control_time_s=cfg.dataset.episode_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    display_compressed_images=display_compressed_images,
                )

                if not events["stop_recording"] and recorded_episodes < cfg.dataset.num_episodes - 1:
                    log_say("Reset the environment", cfg.play_sounds)
                    record_loop(
                        robot=robot,
                        events=events,
                        fps=cfg.dataset.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop,
                        dataset=None,
                        control_time_s=cfg.dataset.reset_time_s,
                        single_task=cfg.dataset.single_task,
                        display_data=cfg.display_data,
                        display_compressed_images=display_compressed_images,
                    )

                if events["rerecord_episode"]:
                    log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                if recorded_episodes >= cfg.dataset.num_episodes - 1 or events["stop_recording"]:
                    _disconnect_recording_devices(robot, teleop)

                dataset.save_episode()
                recorded_episodes += 1
    finally:
        log_say("Stop recording", cfg.play_sounds, blocking=True)
        _disconnect_recording_devices(robot, teleop)
        if dataset:
            dataset.finalize()
        if not is_headless() and listener:
            listener.stop()
        if cfg.display_data:
            rr.rerun_shutdown()
        if dataset and cfg.dataset.push_to_hub:
            dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
        log_say("Exiting", cfg.play_sounds)

    return dataset


def main() -> None:
    _register_third_party_devices()
    record_pico4_hand()
