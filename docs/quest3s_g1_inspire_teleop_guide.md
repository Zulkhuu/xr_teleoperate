# Meta Quest 3S Teleoperation Guide for G1 + Inspire Hand

中文 | [English](#english)

本文档针对本仓库当前 `main` 分支整理，目标组合是：

- XR 设备：Meta Quest 3S
- 机器人：Unitree G1，通常使用 29 DoF 配置，即 `--arm=G1_29`
- 灵巧手：Inspire 因时灵巧手，按实际型号选择 `--ee=inspire_dfx` 或 `--ee=inspire_ftp`

仓库入口是 `teleop/teleop_hand_and_arm.py`。该脚本负责从 Quest 浏览器/Vuer 读取手部或手柄数据，通过 IK 控制 G1 双臂，并把手部关键点重定向到 Inspire 手。

注意：仓库 README 的部分旧示例仍写 `--xr-mode=hand`，当前代码实际参数是 `--input-mode hand`。

## 1. 先判断你的 Inspire 手型号

本仓库支持两条 Inspire 路径：

| 手型 | 启动参数 | 代码控制器 | DDS 话题/依赖 | 适用情况 |
| --- | --- | --- | --- | --- |
| Inspire DFX / RH56DFX | `--ee=inspire_dfx` | `Inspire_Controller_DFX` | `rt/inspire/cmd`, `rt/inspire/state`; 需要在 PC2 上运行 `DFX_inspire_service` | Unitree 官方 DFX 手服务方案 |
| Inspire FTP / RH56DFTP | `--ee=inspire_ftp` | `Inspire_Controller_FTP` | `rt/inspire_hand/ctrl/l`, `rt/inspire_hand/ctrl/r`, `rt/inspire_hand/state/l`, `rt/inspire_hand/state/r`; 需要 `inspire_sdkpy` | FTP 手，仓库已在主程序中支持 |

如果你不确定是哪种手，先看采购/装机资料，或在 PC2 上确认现有手服务和 DDS 话题。DFX 通常会用 `DFX_inspire_service` 里的 `sudo ./inspire_g1`。

## 2. 网络拓扑和 IP

推荐使用仓库默认网络：

| 设备 | 作用 | 常用 IP / 地址 |
| --- | --- | --- |
| Host PC | 运行 `xr_teleoperate/teleop/teleop_hand_and_arm.py`，接入 DDS 和 Vuer 服务 | 例如 `192.168.123.2` |
| G1 PC2 | 运行图像服务 `teleimager`，以及 Inspire DFX 手服务 | 默认常见 `192.168.123.164` |
| Quest 3S | 浏览器进入 Vuer 页面，提供手部/手柄输入和显示机器人视角 | 与 Host/PC2 在同一局域网，或用 ADB reverse |

建议 Host PC 有线连接 G1/路由器，Quest 3S 连接同一 Wi-Fi。路由器至少建议 Wi-Fi 6。Host IP 和 PC2 IP 要能互相 `ping` 通。

检查 Host 网卡名和 IP：

```bash
ip addr
ping 192.168.123.164
```

如果你的 DDS 通信走指定网卡，启动时加：

```bash
--network-interface <your_interface_name>
```

例如 `--network-interface enp3s0`。

## 3. Host PC 安装

在 Host PC 上：

```bash
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv

git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd xr_teleoperate
git submodule update --init --depth 1

cd teleop/teleimager
pip install -e . --no-deps

cd ../televuer
pip install -e .

cd ../robot_control/dex-retargeting
pip install -e .

cd ../../..
pip install -r requirements.txt
```

安装 Unitree Python SDK：

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

生成 Quest/Pico 可用的自签名证书，并让 Vuer 和 teleimager 共用：

```bash
cd ~/xr_teleoperate/teleop/televuer
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem

mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/

sudo ufw allow 8012
```

如果系统防火墙开启，还要允许 WebRTC 图像端口，默认头部相机是 `60001`：

```bash
sudo ufw allow 60001
```

如还使用腕部相机，默认还有 `60002`、`60003`。

## 4. G1 PC2 图像服务

PC2 上需要启动 `teleimager`，负责把头部相机图像通过 ZMQ/WebRTC 发给 Host 和 Quest。

SSH 到 PC2：

```bash
ssh unitree@192.168.123.164
```

安装 teleimager 服务端：

```bash
conda create -n teleimager python=3.10 -y
conda activate teleimager

sudo apt install -y libusb-1.0-0-dev libturbojpeg-dev
git clone https://github.com/unitreerobotics/teleimager.git
cd teleimager
pip install -e ".[server]"
bash setup_uvc.sh
```

把 Host 生成的证书拷到 PC2：

```bash
# 在 Host PC 执行
scp ~/xr_teleoperate/teleop/televuer/key.pem \
    ~/xr_teleoperate/teleop/televuer/cert.pem \
    unitree@192.168.123.164:~/teleimager/
```

PC2 上配置证书路径：

```bash
cd ~/teleimager
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/
```

PC2 上发现相机：

```bash
teleimager-server --cf
# 如果使用 RealSense:
teleimager-server --cf --rs
```

按发现结果修改 `~/teleimager/cam_config_server.yaml`。头部相机默认关键配置类似：

```yaml
head_camera:
  enable_zmq: true
  zmq_port: 55555
  enable_webrtc: true
  webrtc_port: 60001
  type: uvc
  image_shape: [480, 1280]
  binocular: true
  fps: 30
  video_id: 0
```

启动图像服务：

```bash
cd ~/teleimager
teleimager-server
# 如果使用 RealSense:
teleimager-server --rs
```

在 Host PC 上可测试图像订阅：

```bash
conda activate tv
teleimager-client --host 192.168.123.164
```

在 Quest 3S 浏览器也可以打开下面地址测试 WebRTC。首次访问会出现证书警告，选择继续访问，然后点页面里的 `start`：

```text
https://192.168.123.164:60001
```

## 5. G1 PC2 Inspire 手服务

### 5.1 DFX 手

如果你使用 `--ee=inspire_dfx`，PC2 需要先跑 DFX 手服务：

```bash
cd ~
git clone https://github.com/unitreerobotics/DFX_inspire_service.git
sudo apt install libboost-all-dev libspdlog-dev

cd DFX_inspire_service
mkdir -p build
cd build
cmake ..
make -j6
```

终端 1，启动 G1 DFX 手服务：

```bash
sudo ./inspire_g1
```

终端 2，测试手是否能连续开合：

```bash
./hand_example
```

确认两只手能连续开合后，关闭 `hand_example`，保留终端 1 的 `inspire_g1` 服务运行。

常见问题：

- 如果编译时报 CycloneDDS 符号缺失，先按 `DFX_inspire_service` README 安装 C++ 版 `unitree_sdk2`。
- 如果提示打不开 `/dev/ttyUSB*`，DFX G1 服务的串口名可能是源码里写死的，需要按实际设备修改。

### 5.2 FTP 手

如果你使用 `--ee=inspire_ftp`，主程序会导入：

```python
from inspire_sdkpy import inspire_dds
import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
```

因此运行 `teleop_hand_and_arm.py` 的 Python 环境必须能 import `inspire_sdkpy`，并且 PC2/机器人侧要有 FTP 手对应的 DDS 服务发布：

- 左手控制：`rt/inspire_hand/ctrl/l`
- 右手控制：`rt/inspire_hand/ctrl/r`
- 左手状态：`rt/inspire_hand/state/l`
- 右手状态：`rt/inspire_hand/state/r`

FTP 的安装方式取决于你的 Unitree/Inspire 交付包。安装后先单独验证手的状态和角度控制，再启动本仓库主程序。

## 6. Quest 3S 设置

仓库 Wiki 记录的 Quest 3S 已验证环境为：

- system version: `v78.0 1057840091500610`
- browser version: `38.2.0.13.53.728503412`
- chromium version: `134.0.6998.196`

操作方式：

1. 确保 Quest 3S 与 Host PC/PC2 在同一网络。
2. 打开 Quest Browser。
3. 如果图像服务开启了 WebRTC，先访问 `https://192.168.123.164:60001`，手动信任证书并点 `start` 验证图像。
4. 打开 Vuer 页面：

```text
https://192.168.123.2:8012/?ws=wss://192.168.123.2:8012
```

其中 `192.168.123.2` 要替换成你的 Host PC IP。

5. 进入页面后点击 `Virtual Reality`，允许浏览器请求的 XR/手部跟踪权限。

如果 Quest 浏览器无法直接连到 Host 的 `8012`，可以用 ADB reverse。先用 USB 连接 Quest，开启开发者模式和 USB 调试授权，然后在 ADB 目录执行：

```bash
sudo ./adb devices
sudo ./adb -s <quest_device_id> reverse tcp:8012 tcp:8012
sudo ./adb -s <quest_device_id> reverse --list
```

使用 ADB reverse 后，Quest 里可尝试访问：

```text
https://127.0.0.1:8012/?ws=wss://127.0.0.1:8012
```

如果要无线 ADB reverse：

```bash
sudo ./adb shell ifconfig wlan0
sudo ./adb tcpip 5566
sudo ./adb connect <quest_ip>:5566
sudo ./adb -s <quest_ip>:5566 reverse tcp:8012 tcp:8012
```

## 7. 启动 G1 + Inspire 遥操作

确认以下服务已经运行：

- PC2: `teleimager-server`
- PC2: DFX 手则运行 `sudo ./inspire_g1`；FTP 手则确认 FTP DDS 服务正常
- Host: 已进入 `tv` 环境，并能 ping PC2
- Quest 3S: 已连接同一网络，浏览器可打开 Vuer 页面

### 7.1 不让机器人行走，只遥操双臂和手

这是更保守的首选方式。脚本会尝试进入 Debug Mode，并锁住非手臂关节。

DFX：

```bash
conda activate tv
cd ~/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --input-mode hand \
  --display-mode immersive \
  --arm G1_29 \
  --ee inspire_dfx \
  --img-server-ip 192.168.123.164 \
  --network-interface <your_interface_name>
```

FTP：

```bash
conda activate tv
cd ~/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --input-mode hand \
  --display-mode immersive \
  --arm G1_29 \
  --ee inspire_ftp \
  --img-server-ip 192.168.123.164 \
  --network-interface <your_interface_name>
```

如果 DDS 默认网卡正确，可以去掉 `--network-interface`。

### 7.2 边行走边遥操上肢

只有确认风险可控后再用 `--motion`。G1 先用 R3 遥控器进入常规运控模式。新版按键说明通常是：

```text
L2 + B   -> Damping mode
L2 + UP  -> Locked Standing
R1 + X   -> 1 DoF waist regular mode
```

然后启动：

```bash
python teleop_hand_and_arm.py \
  --input-mode controller \
  --display-mode immersive \
  --arm G1_29 \
  --ee inspire_dfx \
  --img-server-ip 192.168.123.164 \
  --motion \
  --network-interface <your_interface_name>
```

`--motion` 下手柄逻辑：

- 右手柄 A：退出遥操作
- 左 trigger：按下关闭左 Inspire 手，松开打开左 Inspire 手
- 右 trigger：按下关闭右 Inspire 手，松开打开右 Inspire 手
- 左右摇杆同时按下：软急停，切阻尼
- 左摇杆：前后左右移动，代码限速到 0.3
- 右摇杆：转向，代码限速到 0.3

手部精细控制建议仍用 `--input-mode hand`。如果只用手势模式且想让机器人走路，可继续用 R3 控制下肢。

## 8. 开始、录制和退出

启动主程序并进入 Quest VR 页面后：

1. 先把双臂姿态摆到接近机器人初始姿态，避免按 `r` 后突然大幅摆动。
2. 在 Host 终端按 `r`，机器人开始跟随。
3. 如果启动时加了 `--record`，遥操作中按 `s` 开始录制，再按 `s` 停止并保存。
4. 退出前尽量把双臂移回接近初始姿态。
5. 在 Host 终端按 `q` 退出。程序会调用 `ctrl_dual_arm_go_home()` 让双臂回到安全姿态。

录制示例：

```bash
python teleop_hand_and_arm.py \
  --input-mode hand \
  --arm G1_29 \
  --ee inspire_dfx \
  --img-server-ip 192.168.123.164 \
  --record \
  --task-dir ./utils/data \
  --task-name g1_inspire_pick_test \
  --task-goal "pick up the target object" \
  --task-desc "G1 with Inspire hand teleoperation using Quest 3S" \
  --task-steps "step1: approach; step2: grasp; step3: lift"
```

默认数据目录在 `teleop/utils/data/`。

## 9. 快速排查

| 现象 | 检查项 |
| --- | --- |
| Quest 打不开 `https://<host>:8012` | Host IP 是否正确；Host 和 Quest 是否同网段；`sudo ufw allow 8012`；必要时用 ADB reverse |
| Vuer 页面开了但没有机器人视角 | PC2 的 `teleimager-server` 是否运行；`cam_config_server.yaml` 相机 ID 是否正确；Quest 是否先信任 `https://<pc2>:60001` 证书 |
| Host 订阅不到图像 | `teleimager-client --host 192.168.123.164`；检查 55555/60001 端口和 PC2 防火墙 |
| 主程序一直等 DDS | Host 网卡选错时加 `--network-interface <name>`；Host 和 PC2 是否互通；`unitree_sdk2_python` 是否安装 |
| DFX 手不动 | PC2 是否运行 `sudo ./inspire_g1`；`./hand_example` 是否能单独控制；检查 `rt/inspire/cmd` 和 `rt/inspire/state` |
| FTP 手导入失败 | Host Python 环境是否安装 `inspire_sdkpy`；FTP 手 DDS 服务是否在发布 `rt/inspire_hand/*` |
| 按 `r` 后手臂动作过大 | 启动前操作者手臂没有对齐机器人初始姿态；先不用 `--motion`，降低频率或重新开始 |
| 退出时担心冲击 | 退出前手动把双臂移动到接近初始姿态，再按 `q` |

## 10. 参考链接

- 本仓库 README：`README.md`, `README_zh-CN.md`
- 图像服务：`teleop/teleimager/README.md`
- XR/Vuer 服务：`teleop/televuer/README.md`
- Quest 3S 与 ADB reverse：<https://github.com/unitreerobotics/xr_teleoperate/wiki/XR_Device>
- G1 motion mode 注意事项：<https://github.com/unitreerobotics/xr_teleoperate/wiki/Motion>
- DFX 手服务：<https://github.com/unitreerobotics/DFX_inspire_service>

---

## English

[中文](#meta-quest-3s-teleoperation-guide-for-g1--inspire-hand) | English

This guide is written for the current `main` branch of this repository and the following hardware:

- XR device: Meta Quest 3S
- Robot: Unitree G1, normally the 29 DoF model, `--arm=G1_29`
- Dexterous hand: Inspire hand, selected as either `--ee=inspire_dfx` or `--ee=inspire_ftp`

The main entry point is `teleop/teleop_hand_and_arm.py`. It receives hand/controller data from Quest Browser through Vuer, solves IK for the G1 arms, and retargets Quest hand tracking to the Inspire hand.

Note: some older README examples use `--xr-mode=hand`; the current code uses `--input-mode hand`.

## 1. Identify Your Inspire Hand Type

The repository supports two Inspire paths:

| Hand | Launch argument | Controller in code | DDS topics / dependency | Use case |
| --- | --- | --- | --- | --- |
| Inspire DFX / RH56DFX | `--ee=inspire_dfx` | `Inspire_Controller_DFX` | `rt/inspire/cmd`, `rt/inspire/state`; requires `DFX_inspire_service` running on PC2 | Unitree's DFX service path |
| Inspire FTP / RH56DFTP | `--ee=inspire_ftp` | `Inspire_Controller_FTP` | `rt/inspire_hand/ctrl/l`, `rt/inspire_hand/ctrl/r`, `rt/inspire_hand/state/l`, `rt/inspire_hand/state/r`; requires `inspire_sdkpy` | FTP hand path, already wired in the main program |

If you are not sure which one you have, check the purchase/integration documents or inspect the hand service already installed on PC2. DFX commonly runs through `DFX_inspire_service` with `sudo ./inspire_g1`.

## 2. Network Layout and IP Addresses

Recommended default layout:

| Device | Role | Typical IP / address |
| --- | --- | --- |
| Host PC | Runs `xr_teleoperate/teleop/teleop_hand_and_arm.py`, DDS client, and Vuer server | e.g. `192.168.123.2` |
| G1 PC2 | Runs `teleimager` camera service and, for DFX, Inspire hand service | commonly `192.168.123.164` |
| Quest 3S | Opens the Vuer page in browser, provides hand/controller input and robot view | same LAN as Host/PC2, or use ADB reverse |

Prefer a wired connection between Host PC and the G1/router, and connect Quest 3S to the same Wi-Fi. Wi-Fi 6 or better is recommended. Host and PC2 must be able to ping each other.

Check the Host network interface and IP:

```bash
ip addr
ping 192.168.123.164
```

If DDS must use a specific interface, add:

```bash
--network-interface <your_interface_name>
```

Example: `--network-interface enp3s0`.

## 3. Host PC Setup

On the Host PC:

```bash
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv

git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd xr_teleoperate
git submodule update --init --depth 1

cd teleop/teleimager
pip install -e . --no-deps

cd ../televuer
pip install -e .

cd ../robot_control/dex-retargeting
pip install -e .

cd ../../..
pip install -r requirements.txt
```

Install the Unitree Python SDK:

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

Generate a self-signed certificate for Quest/Pico and share it between Vuer and teleimager:

```bash
cd ~/xr_teleoperate/teleop/televuer
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem

mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/

sudo ufw allow 8012
```

If your firewall is enabled, also allow the WebRTC image ports. The default head-camera port is `60001`:

```bash
sudo ufw allow 60001
```

If wrist cameras are used, the defaults also include `60002` and `60003`.

## 4. G1 PC2 Image Service

PC2 should run `teleimager`, which streams the head camera through ZMQ/WebRTC to the Host and Quest.

SSH into PC2:

```bash
ssh unitree@192.168.123.164
```

Install the teleimager server:

```bash
conda create -n teleimager python=3.10 -y
conda activate teleimager

sudo apt install -y libusb-1.0-0-dev libturbojpeg-dev
git clone https://github.com/unitreerobotics/teleimager.git
cd teleimager
pip install -e ".[server]"
bash setup_uvc.sh
```

Copy the Host certificates to PC2:

```bash
# Run on Host PC
scp ~/xr_teleoperate/teleop/televuer/key.pem \
    ~/xr_teleoperate/teleop/televuer/cert.pem \
    unitree@192.168.123.164:~/teleimager/
```

Configure the certificate path on PC2:

```bash
cd ~/teleimager
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/
```

Discover cameras on PC2:

```bash
teleimager-server --cf
# If using RealSense:
teleimager-server --cf --rs
```

Edit `~/teleimager/cam_config_server.yaml` according to the discovery result. A typical head-camera setup is:

```yaml
head_camera:
  enable_zmq: true
  zmq_port: 55555
  enable_webrtc: true
  webrtc_port: 60001
  type: uvc
  image_shape: [480, 1280]
  binocular: true
  fps: 30
  video_id: 0
```

Start the image server:

```bash
cd ~/teleimager
teleimager-server
# If using RealSense:
teleimager-server --rs
```

Test image subscription on the Host PC:

```bash
conda activate tv
teleimager-client --host 192.168.123.164
```

You can also test WebRTC from Quest Browser. The first visit will show a certificate warning; continue to the page and click `start`:

```text
https://192.168.123.164:60001
```

## 5. G1 PC2 Inspire Hand Service

### 5.1 DFX Hand

If you use `--ee=inspire_dfx`, start the DFX hand service on PC2 first:

```bash
cd ~
git clone https://github.com/unitreerobotics/DFX_inspire_service.git
sudo apt install libboost-all-dev libspdlog-dev

cd DFX_inspire_service
mkdir -p build
cd build
cmake ..
make -j6
```

Terminal 1, start the G1 DFX service:

```bash
sudo ./inspire_g1
```

Terminal 2, verify that both hands can open and close repeatedly:

```bash
./hand_example
```

After the test succeeds, stop `hand_example` and keep `sudo ./inspire_g1` running.

Common issues:

- If build fails with missing CycloneDDS symbols, install the C++ `unitree_sdk2` as described in the `DFX_inspire_service` README.
- If `/dev/ttyUSB*` cannot be opened, the G1 DFX serial name may be hard-coded in the source and must match your robot.

### 5.2 FTP Hand

If you use `--ee=inspire_ftp`, the main program imports:

```python
from inspire_sdkpy import inspire_dds
import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
```

So the Python environment running `teleop_hand_and_arm.py` must be able to import `inspire_sdkpy`, and the robot/PC2 side must provide the FTP hand DDS service:

- Left command: `rt/inspire_hand/ctrl/l`
- Right command: `rt/inspire_hand/ctrl/r`
- Left state: `rt/inspire_hand/state/l`
- Right state: `rt/inspire_hand/state/r`

The FTP installation method depends on the Unitree/Inspire package delivered with your robot. Verify standalone hand state and angle control before starting this repository's main program.

## 6. Quest 3S Setup

The repository Wiki records the following verified Quest 3S environment:

- system version: `v78.0 1057840091500610`
- browser version: `38.2.0.13.53.728503412`
- chromium version: `134.0.6998.196`

Procedure:

1. Make sure Quest 3S, Host PC, and PC2 are on the same network.
2. Open Quest Browser.
3. If WebRTC is enabled for the image service, first open `https://192.168.123.164:60001`, trust the certificate, and click `start`.
4. Open the Vuer page:

```text
https://192.168.123.2:8012/?ws=wss://192.168.123.2:8012
```

Replace `192.168.123.2` with the Host PC IP.

5. Click `Virtual Reality` and allow the requested XR/hand-tracking permissions.

If Quest Browser cannot directly reach Host port `8012`, use ADB reverse. Connect Quest over USB, enable developer mode and USB debugging authorization, then run from the ADB directory:

```bash
sudo ./adb devices
sudo ./adb -s <quest_device_id> reverse tcp:8012 tcp:8012
sudo ./adb -s <quest_device_id> reverse --list
```

After ADB reverse, try this URL in Quest:

```text
https://127.0.0.1:8012/?ws=wss://127.0.0.1:8012
```

For wireless ADB reverse:

```bash
sudo ./adb shell ifconfig wlan0
sudo ./adb tcpip 5566
sudo ./adb connect <quest_ip>:5566
sudo ./adb -s <quest_ip>:5566 reverse tcp:8012 tcp:8012
```

## 7. Launch G1 + Inspire Teleoperation

Confirm these services are already running:

- PC2: `teleimager-server`
- PC2: for DFX, `sudo ./inspire_g1`; for FTP, the FTP DDS hand service
- Host: `tv` environment active and PC2 reachable
- Quest 3S: same network, Vuer page can be opened

### 7.1 Arms and hands only, no walking

This is the conservative first run. The script attempts to enter Debug Mode and locks non-arm joints.

DFX:

```bash
conda activate tv
cd ~/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --input-mode hand \
  --display-mode immersive \
  --arm G1_29 \
  --ee inspire_dfx \
  --img-server-ip 192.168.123.164 \
  --network-interface <your_interface_name>
```

FTP:

```bash
conda activate tv
cd ~/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --input-mode hand \
  --display-mode immersive \
  --arm G1_29 \
  --ee inspire_ftp \
  --img-server-ip 192.168.123.164 \
  --network-interface <your_interface_name>
```

Remove `--network-interface` if DDS already selects the correct interface.

### 7.2 Walking plus upper-body teleoperation

Use `--motion` only after you are confident about the setup. Put G1 into regular motion mode with the R3 remote first. Newer documentation commonly uses:

```text
L2 + B   -> Damping mode
L2 + UP  -> Locked Standing
R1 + X   -> 1 DoF waist regular mode
```

Then launch:

```bash
python teleop_hand_and_arm.py \
  --input-mode controller \
  --display-mode immersive \
  --arm G1_29 \
  --ee inspire_dfx \
  --img-server-ip 192.168.123.164 \
  --motion \
  --network-interface <your_interface_name>
```

Controller behavior in `--motion` mode:

- Right controller A: exit teleoperation
- Both thumbsticks pressed: soft emergency stop, switch to damping
- Left thumbstick: forward/back/side motion, velocity limited to 0.3 in code
- Right thumbstick: yaw motion, velocity limited to 0.3 in code

For fine hand control, `--input-mode hand` is still recommended. If you use hand tracking and need locomotion, continue using the R3 remote for the lower body.

## 8. Start, Record, and Exit

After the main program is running and Quest has entered the VR page:

1. Align your arms close to the robot initial arm pose before pressing `r`.
2. Press `r` in the Host terminal to start tracking.
3. If launched with `--record`, press `s` to start recording and press `s` again to stop and save.
4. Before exit, move both arms close to the initial pose.
5. Press `q` in the Host terminal. The program calls `ctrl_dual_arm_go_home()` before exiting.

Recording example:

```bash
python teleop_hand_and_arm.py \
  --input-mode hand \
  --arm G1_29 \
  --ee inspire_dfx \
  --img-server-ip 192.168.123.164 \
  --record \
  --task-dir ./utils/data \
  --task-name g1_inspire_pick_test \
  --task-goal "pick up the target object" \
  --task-desc "G1 with Inspire hand teleoperation using Quest 3S" \
  --task-steps "step1: approach; step2: grasp; step3: lift"
```

The default recording directory is `teleop/utils/data/`.

## 9. Quick Troubleshooting

| Symptom | What to check |
| --- | --- |
| Quest cannot open `https://<host>:8012` | Host IP; same subnet; `sudo ufw allow 8012`; use ADB reverse if needed |
| Vuer opens but robot view is missing | PC2 `teleimager-server`; correct camera IDs in `cam_config_server.yaml`; trust `https://<pc2>:60001` from Quest |
| Host cannot receive images | `teleimager-client --host 192.168.123.164`; ports 55555/60001; PC2 firewall |
| Main program keeps waiting for DDS | Add `--network-interface <name>`; check Host-PC2 connectivity; install `unitree_sdk2_python` |
| DFX hand does not move | Is `sudo ./inspire_g1` running on PC2; does `./hand_example` work; check `rt/inspire/cmd` and `rt/inspire/state` |
| FTP hand import fails | Is `inspire_sdkpy` installed in the Host Python environment; is the FTP DDS hand service publishing `rt/inspire_hand/*` |
| Arm moves too much after `r` | Operator arms were not aligned with the robot initial pose; first test without `--motion`; reduce frequency or restart |
| Exit may cause impact | Move both arms close to the initial pose before pressing `q` |

## 10. References

- This repository README: `README.md`, `README_zh-CN.md`
- Image service: `teleop/teleimager/README.md`
- XR/Vuer service: `teleop/televuer/README.md`
- Quest 3S and ADB reverse: <https://github.com/unitreerobotics/xr_teleoperate/wiki/XR_Device>
- G1 motion mode notes: <https://github.com/unitreerobotics/xr_teleoperate/wiki/Motion>
- DFX hand service: <https://github.com/unitreerobotics/DFX_inspire_service>
