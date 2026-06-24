# LeRobot Pico4 Hand 遥操作插件

[English README](./README.md)

这是一个独立的 LeRobot Pico4 手追踪遥操作插件，安装后注册：

- `--teleop.type=pico4_hand`

插件读取 Pico/OpenXR 的 26 个手部关节点：

- 掌心位姿用于机械臂 TCP 笛卡尔控制
- 手指关节点通过 `dex-retargeting` 映射到灵巧手关节动作

Revo2 默认动作 key：

- 左手：`l_th_prox.pos`, `l_th_mcp.pos`, `l_idx_prox.pos`, `l_mid_prox.pos`, `l_ring_prox.pos`, `l_pky_prox.pos`
- 右手：对应的 `r_*` key

如果设置 `--teleop.hand_only=true`，只输出灵巧手动作；否则还会输出：

- `tcp.x`, `tcp.y`, `tcp.z`
- `tcp.r1` ... `tcp.r6`

## 安装

本包依赖你的 dex-retargeting 仓库：

```text
dex_retargeting @ git+ssh://git@github.com/xensedyl/dex-retargeting.git
```

```bash
git clone git@github.com:xensedyl/dex-retargeting.git
cd dex-retargeting
pip install -e .
```

在 LeRobot 使用的 Python 环境中安装：

```bash
pip install -e ./lerobot-teleoperator-pico4-hand
```

Pico4 SDK Python 模块 `xensevr_pc_service_sdk` 需要提前安装在同一个环境里。

## 遥操作

Franka + Revo2 灵巧手：

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

只控制 Revo2 灵巧手：

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

## 采集数据

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

## 说明

这个包提供独立命令 `lerobot-teleoperate-pico4-hand` 和
`lerobot-record-pico4-hand`，同时提供 LeRobot 第三方插件注册。

本包不内置 Pico4 SDK。`xensevr_pc_service_sdk` 需要单独安装。

retargeting worker 会在子进程里运行，并清理 `PYTHONPATH` /
`LD_LIBRARY_PATH`，避免 ROS/conda 的二进制依赖污染 `dex-retargeting`
的 `pin` / `coal` 运行环境。
