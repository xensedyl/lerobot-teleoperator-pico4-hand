# LeRobot Pico4 Hand Teleoperator

[中文版说明](./README.zh-CN.md)

Standalone LeRobot teleoperator plugin for Pico4 hand tracking with
`dex-retargeting`.

The package registers:

- `--teleop.type=pico4_hand`

The teleoperator reads Pico/OpenXR 26-joint hand tracking data, uses the palm
pose for Cartesian TCP control, and uses `dex-retargeting` to map human finger
keypoints to robot-hand joint actions.

For Revo2, the default action keys are:

- `l_th_prox.pos`, `l_th_mcp.pos`, `l_idx_prox.pos`, `l_mid_prox.pos`, `l_ring_prox.pos`, `l_pky_prox.pos`
- or the corresponding `r_*` keys when `--teleop.hand_type=right`

When `--teleop.hand_only=true`, only hand joint actions are emitted. Otherwise
the teleoperator also emits:

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6` using 6D rotation representation

## Dependencies

This package depends on:

```text
dex_retargeting @ git+ssh://git@github.com/xensedyl/dex-retargeting.git
```

```bash
git clone git@github.com:xensedyl/dex-retargeting.git
cd dex-retargeting
pip install -e .
```

Install in the active LeRobot environment:

```bash
pip install -e ./lerobot-teleoperator-pico4-hand
```


The Pico4 SDK Python module `xensevr_pc_service_sdk` must already be installed
in the same environment.

## Teleoperation

Franka + Revo2 hand:

```bash
lerobot-teleoperate-pico4-hand \
  --robot.type=franka_research3_dexhand \
  --robot.fci_ip=192.168.99.111 \
  --robot.control_mode=cartesian_impedance \
  --robot.use_gripper=false \
  --robot.dexhand_hand_type=left \
  --robot.dexhand_auto_detect_quick=true \
  --teleop.type=pico4_hand \
  --teleop.hand_type=left \
  --teleop.robot_name=revo2 \
  --teleop.retargeting_type=vector \
  --fps=30 \
  --display_data=false
```

Revo2 hand only:

```bash
lerobot-teleoperate-pico4-hand \
  --robot.type=revo2_hand \
  --robot.hand_type=left \
  --robot.auto_detect_quick=true \
  --teleop.type=pico4_hand \
  --teleop.hand_type=left \
  --teleop.robot_name=revo2 \
  --teleop.retargeting_type=vector \
  --teleop.hand_only=true \
  --fps=30 \
  --display_data=false
```

## Recording

```bash
lerobot-record-pico4-hand \
  --robot.type=franka_research3_dexhand \
  --robot.fci_ip=192.168.99.111 \
  --robot.control_mode=cartesian_impedance \
  --robot.use_gripper=false \
  --robot.dexhand_hand_type=left \
  --robot.dexhand_auto_detect_quick=true \
  --teleop.type=pico4_hand \
  --teleop.hand_type=left \
  --teleop.robot_name=revo2 \
  --teleop.retargeting_type=vector \
  --dataset.repo_id=${HF_USER}/franka-revo2-pico4-hand-demo \
  --dataset.single_task="Teleoperate Franka Revo2 with Pico4 hand tracking" \
  --dataset.num_episodes=1 \
  --dataset.fps=30 \
  --dataset.episode_time_s=600 \
  --dataset.reset_time_s=120 \
  --resume=false \
  --dataset.push_to_hub=false \
  --display_data=false
```

## Notes

This package provides independent commands and the LeRobot plugin registration.
It does not vendor the Pico4 SDK. Install `xensevr_pc_service_sdk` separately.

The retargeting worker runs in a subprocess with `PYTHONPATH` and
`LD_LIBRARY_PATH` cleared to avoid mixing ROS/conda binary dependencies with
`dex-retargeting`'s `pin`/`coal` stack.
