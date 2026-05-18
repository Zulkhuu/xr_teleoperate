# xr_teleoperate 当前配置与运行流程

本文档记录的是这台 Host 和这台 Unitree G1 当前可用的配置。旧版文档里关于 `pixi`、Dex3、DFX Inspire bridge、`teleimager-server --rs` 的说明已经不适用于当前状态。

## 1. 当前硬件和网络

**硬件**

- 机器人：Unitree G1 EDU，`G1_29`，lock waist
- 灵巧手：Inspire FTP / RH56DFTP，使用 `--ee=inspire_ftp`
- 头部相机：Intel RealSense D435i，目前通过 OpenCV camera path 给 teleimager 使用
- XR：Meta Quest 3S，当前使用 controller 输入，不使用手部 tracking

**网络**

| 设备 | IP / 接口 | 用途 |
| --- | --- | --- |
| Host PC | `eno1 = 192.168.123.222` | 运行 teleop 主程序，接 DDS 和手部 ModbusTCP |
| G1 PC2 / image server | `192.168.123.164` | 运行 teleimager，提供 RGB 图像 |
| G1 robot controller | `192.168.123.161` | Unitree motion / mode service |
| Inspire FTP 左手 | `192.168.123.210:6000` | ModbusTCP |
| Inspire FTP 右手 | `192.168.123.211:6000` | ModbusTCP |
| Quest 3S | 与 Host 同一 WiFi | 打开 Vuer 页面 |

机器人 SSH：

```bash
ssh unitree@192.168.123.164
# 密码：123
# 如果进入后提示 ros: foxy(1) noetic(2)，选 1
```

## 2. Host 环境

当前使用 `uv` 和仓库里的 `.venv`，不是 `pixi`。

常用验证：

```bash
cd /home/zc1525/xr_teleoperate
uv run python -c "import pinocchio, casadi, pymodbus, teleimager, televuer, unitree_sdk2py; print('env ok')"
uv pip list | grep -E 'inspire-sdkpy|teleimager|televuer|unitree-sdk2py|pymodbus'
```

当前关键包应该包括：

- `teleimager` editable: `/home/zc1525/xr_teleoperate/teleop/teleimager`
- `televuer` editable: `/home/zc1525/xr_teleoperate/teleop/televuer`
- `inspire-sdkpy` editable: `/home/zc1525/inspire_hand_ws/inspire_hand_sdk`
- `unitree-sdk2py`
- `pymodbus`

如果 DDS 初始化失败，先确认 Host 机器人网口名：

```bash
ip addr show eno1
```

正常应该能看到 `192.168.123.222/24`。启动 teleop 时使用：

```bash
--network-interface=eno1
```

## 3. 启动 RGB 图像服务

PC2 上当前稳定路径是 OpenCV 模式，不用 `--rs`。之前 `--rs` 会因为 `librealsense2.so.2.50` 缺失失败。

在 Host 上执行：

```bash
ssh unitree@192.168.123.164
echo 123 | sudo -S modprobe uvcvideo
pkill -f teleimager-server || true
nohup ~/.local/bin/teleimager-server > /tmp/teleimager.log 2>&1 &
tail -f /tmp/teleimager.log
```

正常日志应出现类似：

```text
[OpenCVCamera: head_camera] initialized with 480x640 @ 30 FPS
head_camera is ready
```

如果 RGB 掉了，先在 PC2 上重置 RealSense USB，再重启 teleimager：

```bash
ssh unitree@192.168.123.164
echo 123 | sudo -S sh -c 'echo 0 > /sys/bus/usb/devices/2-3/authorized; sleep 2; echo 1 > /sys/bus/usb/devices/2-3/authorized'
pkill -f teleimager-server || true
nohup ~/.local/bin/teleimager-server > /tmp/teleimager.log 2>&1 &
tail -f /tmp/teleimager.log
```

Host 侧快速验证图像：

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python - <<'PY'
import time
from teleimager.image_client import ImageClient
c = ImageClient(host="192.168.123.164", request_bgr=True)
print(c.get_cam_config()["head_camera"])
time.sleep(0.2)
img = c.get_head_frame()
print(None if img is None or img.bgr is None else img.bgr.shape)
c.close()
PY
```

正常输出应包含 `(480, 640, 3)`。

## 4. Teleop 启动命令

### 4.1 推荐：上半身 teleop，下半身保持 Unitree 内置平衡

这个模式适合做桌面操作：机器人下半身保持 Regular/Motion 的站立平衡，不用 Quest 控制移动。

先让机器人处于可站立的 Regular/AI mode，不要在 Damping mode。然后在 Host 上运行：

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion \
  --upper-body-only
```

说明：

- `--motion`：使用 Unitree arm SDK 通道控制上肢，避免进入全身 Debug lowcmd 模式。
- `--upper-body-only`：不发送下半身速度命令，让下半身继续由 Unitree 内置运动/平衡模块保持站立。
- 退出后程序会尝试切回 AI/remote mode，让遥控器恢复可用。

### 4.2 Quest controller 控制移动 + 上半身 teleop

如果你想用 Quest controller 控制机器人移动，去掉 `--upper-body-only`：

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion
```

移动映射：

- 左摇杆前后：`vx`
- 左摇杆左右：`vy`
- 右摇杆左右：`yaw`
- 最大速度比例：`0.3`
- 双侧 thumbstick 同时按下：发送 `Damp()`

注意：这个移动不是用 Unitree 物理遥控器控制，而是由 Quest controller 通过 `LocoClientWrapper.Move(vx, vy, vyaw)` 发送给机器人。

## 5. Quest 3S 连接顺序

推荐顺序：

1. 先启动 `teleop_hand_and_arm.py`
2. Quest 浏览器打开 Vuer 页面：

```text
https://vuer.ai?grid=False
```

或者使用终端里打印出的 `Visit: ...` 地址。

3. 等页面显示 WebSocket connected
4. 点 **Virtual Reality**，允许 VR 权限
5. 人和 controller 面向机器人正前方方向，尽量和机器人初始姿态一致
6. 回到 Host 终端按 `r` 开始
7. 按 `q` 退出

为什么要先进 VR 再按 `r`：

- 程序在按 `r` 后会采样 controller 当前 pose，并把它 anchor 到机器人当前 wrist pose。
- 如果 VR session 的世界坐标朝向不对，手会往操作者真实站位方向扭。
- 当前代码默认使用相对 controller wrist pose；如要退回旧的绝对 wrist pose，可加 `--absolute-wrist-pose`，但不推荐。

## 6. Inspire FTP 手开关逻辑

当前手不是 Dex3，也不是 DFX bridge，而是 Inspire FTP，直接走 ModbusTCP：

- 左手：`192.168.123.210:6000`
- 右手：`192.168.123.211:6000`
- register `1486`：angle command
- register `1522`：speed
- register `1004`：clear error

二值动作：

- controller trigger 松开：open
- controller trigger 按下：close
- 左 trigger 控左手
- 右 trigger 控右手

当前目标值：

```text
open  = [1, 1, 1, 1, 1, 1]      -> [1000, 1000, 1000, 1000, 1000, 1000]
close = [0, 0, 0, 0, 0, 1]      -> [0, 0, 0, 0, 0, 1000]
```

这里是位置 state / angle command，不是加速度。碰到物体后，手会继续尝试到 close 目标；目前没有基于触觉/力反馈的自动减速或自动 hold 逻辑。

手部保护顺序：

- 上电后：手应保持 close，保护手指
- 程序等待 `r` 时：默认不启动手 controller，不会提前 open
- 按 `r` 后：先把双臂抬到安全姿态，再启动手 controller 并 open
- 按 `q` 后：先 hold 当前姿态，再回安全姿态，再 close 手，再慢慢放回启动姿态，最后 release arm SDK

手动把双手 close：

```bash
cd /home/zc1525/xr_teleoperate
uv run python teleop/utils/inspire_ftp_close_hands.py
```

如果手没反应，按顺序查：

```bash
ping 192.168.123.210
ping 192.168.123.211
nc -vz 192.168.123.210 6000
nc -vz 192.168.123.211 6000
cd /home/zc1525/xr_teleoperate
uv run python teleop/utils/inspire_ftp_close_hands.py
```

## 7. 退出保护逻辑

当前 `q` 退出顺序已经改成防止手臂突然掉下：

1. hold 当前 teleop 手臂姿态，默认 `0.8s`
2. 低速回到安全姿态，默认速度 `--arm-safety-velocity=0.8`
3. hold 安全姿态，默认 `1.0s`
4. close 双手
5. `--motion` 下，用 SDK 慢慢放回程序启动时记录的站立手臂姿态，默认 `--exit-lower-velocity=0.35`
6. hold 最终姿态，默认 `1.0s`
7. 用 `--exit-release-duration=12.0` 慢慢 release arm SDK
8. 切回 AI/remote mode

如果放下仍然太快，启动时加：

```bash
--exit-lower-velocity=0.2 --exit-release-duration=18
```

如果只想调试，不想退出时放回启动姿态：

```bash
--disable-exit-lower
```

## 8. 录制数据

启动时加 `--record` 和任务信息：

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion \
  --upper-body-only \
  --record \
  --task-dir=/home/zc1525/xr_teleoperate/records \
  --task-name=g1_inspire_teleop_test \
  --task-goal="teleoperate G1 with Inspire FTP hand" \
  --task-desc="Quest controller teleoperation with RGB observation" \
  --task-steps="start teleop; manipulate object; save episode"
```

录制流程：

- 按 `r` 开始 teleop
- 按 `s` 开始录制
- 再按 `s` 保存当前 episode
- 按 `q` 退出

数据默认保存到：

```text
/home/zc1525/xr_teleoperate/records/<task-name>/episode_0000/
```

当前记录内容：

- `colors/color_0`：头部 RGB，30 Hz，`640x480`
- `states.left_arm/right_arm.qpos`：左右臂当前关节状态
- `actions.left_arm/right_arm.qpos`：IK 输出目标关节
- `states.left_ee/right_ee.qpos`：手状态数组
- `actions.left_ee/right_ee.qpos`：手 open/close action
- full locomotion 模式下：`actions.body.qpos = [vx, vy, vyaw]`
- full locomotion 模式下：`states.body.qpos` 为 35 维全身 motor qpos

当前不记录：

- depth
- wrist camera
- torque
- qvel
- raw Quest pose
- tactile

录制频率：

```text
--frequency 默认 30.0 Hz
1 step = 1 / 30 = 0.033333 s
```

## 9. 常见问题

### 9.1 启动时报 CycloneDDS interface error

确认网卡名：

```bash
ip addr
```

`eno` 是错的，当前 Host 应用：

```bash
--network-interface=eno1
```

如果看到：

```text
/tmp/cdds.LOG: cannot open for writing
```

一般不是主因；真正的问题通常是 DDS interface 选错或权限/环境问题。

### 9.2 RGB 进 VR 后黑屏

先确认 PC2 teleimager 正在输出 OpenCV head camera：

```bash
ssh unitree@192.168.123.164
tail -50 /tmp/teleimager.log
```

再用第 3 节 Host 侧 Python 验证图像是否能拿到 `(480, 640, 3)`。

### 9.3 Controller 一按 r 手臂往人站的位置扭

原因通常是 Quest/OpenXR 世界坐标朝向和机器人坐标朝向没对齐。进入 **Virtual Reality** 的时候，人和 controller 要面向机器人正前方，然后再按 `r`。当前代码会把 controller 起始 pose anchor 到机器人当前 wrist pose，已经比旧的 absolute pose 稳定。

### 9.4 手没动作

当前是 FTP，不需要 PC2 上跑 DFX bridge。先查网络和端口：

```bash
ping 192.168.123.210
ping 192.168.123.211
nc -vz 192.168.123.210 6000
nc -vz 192.168.123.211 6000
```

再手动 close 测试：

```bash
cd /home/zc1525/xr_teleoperate
uv run python teleop/utils/inspire_ftp_close_hands.py
```

### 9.5 遥控器退出后没恢复

按 `q` 后程序会 release arm SDK 并调用 `Exit_Debug_Mode()` 切回 AI/remote mode。等待日志出现：

```text
release arm sdk mode OK
Switch to AI/remote mode: Success
```

如果仍不能用遥控器，手动用 Unitree app 或遥控器切回 Regular/AI mode。

## 10. 当前推荐启动模板

日常上半身 teleop：

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion \
  --upper-body-only \
  --exit-lower-velocity=0.2 \
  --exit-release-duration=18
```

需要 Quest controller 移动时：

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion \
  --exit-lower-velocity=0.2 \
  --exit-release-duration=18
```
