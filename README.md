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

To encode frames into video on the fly while recording (lower disk usage and no separate encoding step afterwards), enable streaming encoding:

  --dataset.streaming_encoding=true \
  --dataset.vcodec=auto \

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
  --dataset.streaming_encoding=true \
  --dataset.vcodec=auto \
  --resume=false \
  --dataset.push_to_hub=false \
  --display_data=false
```

## Franka Realtime Permissions

If Pico4 hand tracking connects successfully but Franka initialization fails,
the issue is usually Linux realtime scheduling permissions, not Pico4,
retargeting, the FCI IP, or dexhand configuration.

Typical log:

```text
Failed to connect: libfranka: unable to set realtime scheduling: Operation not permitted
franky._franky.RealtimeException: libfranka: unable to set realtime scheduling: Operation not permitted
```

Grant realtime permissions to the Linux user that runs teleoperation. Replace
`xense` with your actual username if needed.

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime xense
```

Create `/etc/security/limits.d/99-realtime.conf`:

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

Make sure PAM loads limits:

```bash
grep -R "pam_limits.so" /etc/pam.d/common-session /etc/pam.d/common-session-noninteractive
```

If there is no output, add it:

```bash
echo "session required pam_limits.so" | sudo tee -a /etc/pam.d/common-session
echo "session required pam_limits.so" | sudo tee -a /etc/pam.d/common-session-noninteractive
```

Reboot the computer so the group membership and limits are loaded:

```bash
sudo reboot
```

After reboot, verify:

```bash
id
ulimit -r
ulimit -l
```

Expected results:

- `id` contains `realtime`
- `ulimit -r` prints `99`
- `ulimit -l` prints `unlimited`

You can also test `SCHED_FIFO` directly:

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

If realtime limits are still not applied after reboot, add capabilities to the
Python executable in the active conda environment:

```bash
sudo setcap cap_sys_nice,cap_ipc_lock=eip $(readlink -f /home/xense/miniforge3/envs/hand/bin/python)
getcap $(readlink -f /home/xense/miniforge3/envs/hand/bin/python)
```

Using `sudo -E` to run teleoperation can confirm that the failure is permission
related, but it is not recommended for normal use because it can create root
owned cache, log, calibration, or config files inside the user workspace.

## USB Serial Permissions

If Revo2 or an FTDI-based USB serial device cannot be opened, configure udev
permissions for the FTDI FT2232 device (`0403:6010`). This is the recommended
persistent fix.

```bash
sudo tee /etc/udev/rules.d/99-ftdi-ft2232.rules >/dev/null <<'EOF'
# FTDI FT2232C/D/H Dual UART/FIFO (0403:6010)
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", MODE:="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE:="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the USB device after reloading the rules if the permissions do
not update immediately.

For a temporary test, you can directly relax permissions on the current
`ttyUSB` nodes:

```bash
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB*
```

The `chmod` method is not persistent. It must be repeated after unplugging the
device, rebooting, or if the device is assigned a new `/dev/ttyUSB*` path.

## Notes

This package provides independent commands and the LeRobot plugin registration.
It does not vendor the Pico4 SDK. Install `xensevr_pc_service_sdk` separately.

The retargeting worker runs in a subprocess with `PYTHONPATH` and
`LD_LIBRARY_PATH` cleared to avoid mixing ROS/conda binary dependencies with
`dex-retargeting`'s `pin`/`coal` stack.
