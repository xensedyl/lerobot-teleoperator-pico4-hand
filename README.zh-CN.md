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

## Franka 实时权限配置

如果 Pico4 和 dex-retargeting 已经正常连接，但 Franka 初始化失败，日志中出现：

```text
Failed to connect: libfranka: unable to set realtime scheduling: Operation not permitted
franky._franky.RealtimeException: libfranka: unable to set realtime scheduling: Operation not permitted
```

这个问题通常不是 Pico4 SDK、Franka IP、FCI 网络或 dexhand 配置导致的，
而是当前 Linux 用户没有实时调度权限，导致 `libfranka` 无法设置 realtime
scheduling。Franka 实时控制需要较高的调度优先级，普通用户默认不能直接使用
`SCHED_FIFO` 等实时调度策略。

下面假设当前运行用户为 `xense`。如果用户名不同，请替换成实际用户名。

创建 realtime 用户组：

```bash
sudo groupadd -f realtime
```

将当前用户加入 realtime 用户组：

```bash
sudo usermod -aG realtime xense
```

创建 realtime limits 配置：

```bash
sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<'EOF'
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority -20
@realtime hard priority -20
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF
```

配置含义：

| 配置项 | 作用 |
| --- | --- |
| `rtprio 99` | 允许 realtime 用户组使用最高 99 的实时线程优先级 |
| `priority -20` | 允许进程使用更高的 nice 优先级 |
| `memlock unlimited` | 允许锁定内存，避免实时控制过程中发生内存换页 |

检查 PAM 是否启用了 `pam_limits.so`：

```bash
grep -R "pam_limits.so" /etc/pam.d/common-session /etc/pam.d/common-session-noninteractive
```

正常情况下应看到类似输出：

```text
/etc/pam.d/common-session:session required pam_limits.so
/etc/pam.d/common-session-noninteractive:session required pam_limits.so
```

如果没有输出，则手动追加：

```bash
echo "session required pam_limits.so" | sudo tee -a /etc/pam.d/common-session
echo "session required pam_limits.so" | sudo tee -a /etc/pam.d/common-session-noninteractive
```

完成后建议直接重启系统：

```bash
sudo reboot
```

只重新打开 terminal 不一定生效，因为用户组和 limits 配置需要在新的登录会话中
重新加载。

重启后检查用户组：

```bash
id
```

确认输出中包含 `realtime`。

检查实时优先级权限：

```bash
ulimit -r
```

期望输出：

```text
99
```

检查内存锁定权限：

```bash
ulimit -l
```

期望输出：

```text
unlimited
```

测试 `SCHED_FIFO` 权限：

```bash
python - <<'PY'
import os

try:
    os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
    print("SCHED_FIFO OK")
except Exception as e:
    print("SCHED_FIFO FAILED:", repr(e))
PY
```

如果输出 `SCHED_FIFO OK`，说明当前用户已经具备实时调度权限。

如果重启后仍然报同样错误，可以为当前 conda 环境中的 Python 添加 capability。
请按实际环境路径替换下面的 Python 路径：

```bash
sudo setcap cap_sys_nice,cap_ipc_lock=eip $(readlink -f /home/xense/miniforge3/envs/hand/bin/python)
getcap $(readlink -f /home/xense/miniforge3/envs/hand/bin/python)
```

可以临时使用 `sudo` 验证是否是权限问题：

```bash
sudo -E /home/xense/miniforge3/envs/hand/bin/lerobot-teleoperate-pico4-hand ...
```

但不建议长期用 `sudo` 运行遥操作程序，因为 conda 环境、缓存文件、日志文件、
校准文件和配置文件可能被写成 root owner，影响后续普通用户使用。

如果日志中已经出现：

```text
Pico left hand tracking is active
pico4_hand Pico4Hand connected
```

随后在 Franka 初始化阶段出现：

```text
libfranka: unable to set realtime scheduling: Operation not permitted
```

则应优先排查 Linux realtime 权限，而不是 Pico4、retargeting、FCI IP 或 dexhand
配置。

## USB 串口权限配置

如果 Revo2 或基于 FTDI 的 USB 串口设备无法打开，可以为 FTDI FT2232 设备
`0403:6010` 配置 udev 权限。这是推荐的长期方案。

```bash
sudo tee /etc/udev/rules.d/99-ftdi-ft2232.rules >/dev/null <<'EOF'
# FTDI FT2232C/D/H Dual UART/FIFO (0403:6010)
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", MODE:="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE:="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

如果权限没有立即更新，重新插拔 USB 设备。

临时测试时，也可以直接修改当前 `/dev/ttyUSB*` 节点权限：

```bash
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB*
```

`chmod` 方法不是持久配置。设备重新插拔、系统重启，或者设备变成新的
`/dev/ttyUSB*` 路径后，都可能需要重新执行。

## 说明

这个包提供独立命令 `lerobot-teleoperate-pico4-hand` 和
`lerobot-record-pico4-hand`，同时提供 LeRobot 第三方插件注册。

本包不内置 Pico4 SDK。`xensevr_pc_service_sdk` 需要单独安装。

retargeting worker 会在子进程里运行，并清理 `PYTHONPATH` /
`LD_LIBRARY_PATH`，避免 ROS/conda 的二进制依赖污染 `dex-retargeting`
的 `pin` / `coal` 运行环境。
