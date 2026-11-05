# G1 真机部署与验证计划

更新日期：2026-11-05

本文整理当前双 WEMT 接收器到 Unitree G1 左臂的完整部署流程。当前实机方案使用 G1
官方运控（FSM 500）维持站立和平衡，本项目只通过 `rt/arm_sdk` 控制左臂；实机阶段不启动
本项目的全身 MPC。

本文是一份现场执行清单。更完整的首次上机背景和标定动作说明参见
[first_real_robot_run_zh.md](first_real_robot_run_zh.md) 和
[dual_receiver_workflow_zh.md](dual_receiver_workflow_zh.md)。

## 1. 系统结构与终端分工

### 1.1 仿真链路

```text
两个 WEMT 接收器
  -> dual_em_tracker_node
  -> dual_retargeting_node (simulation_only=true)
  -> /arm_joint_target_sim
  -> centroidal MPC / MuJoCo
```

### 1.2 实机链路

```text
两个 WEMT 接收器
  -> dual_em_tracker_node
  -> dual_retargeting_node (simulation_only=false)
  -> /arm_joint_target
  -> arm_target_udp_sender
  -> UDP 127.0.0.1:15000
  -> g1_arm_sdk_bridge.py
  -> Unitree DDS rt/arm_sdk
  -> G1
```

### 1.3 两类终端

- **Docker 终端**，提示符通常为 `root@kw:/wb_humanoid_mpc_ws#`：运行 ROS 2、WEMT、
  标定、重定向、UDP sender 和 MuJoCo。
- **宿主机终端**，提示符通常为 `kunwang@kw:~$`：运行 Unitree SDK 状态检查和
  `g1_arm_sdk_bridge.py`。

`RETARGET-SIM` 与 `RETARGET-REAL` 不能同时运行。实机阶段也不要保留 MuJoCo/MPC。

### 1.4 一页式启动顺序

| 顺序 | 环境/终端 | 动作 | 进入下一步的门槛 |
| --- | --- | --- | --- |
| 1 | 宿主机 | 配置 G1 有线网和 WEMT 热点 | ping G1 成功，两只接收器身份明确 |
| 2 | Docker | 构建并 source ROS 工作区 | `remote_control` 构建成功 |
| 3 | Docker `EM` | 启动 `dual_em_tracker_node` | 两路接近 50 Hz，无持续数据错误 |
| 4 | Docker `CALIBRATION` | 完成本次安装标定 | 保存 YAML，`dry_run_only: false` |
| 5 | Docker | 重定向 dry-run | 数值方向正确，无跳变/持续丢帧 |
| 6 | Docker | MuJoCo + `RETARGET-SIM` | 动作、碰撞保护和回中正确 |
| 7 | 宿主机 | LowState/FSM/固定接管检查 | 通信正常，权重正常回零 |
| 8 | 宿主机/手柄 | 进入官方 FSM 500 | 机器人自主平衡稳定，读回 500 |
| 9 | Docker `RETARGET-REAL` | 发布 `/arm_joint_target` | 约 50 Hz，无保护性丢帧 |
| 10 | Docker `UDP` | 启动 UDP sender | 五关节角连续输出 |
| 11 | 宿主机 | BRIDGE dry-run | `DRY-RUN RESULT: PASS` |
| 12 | 宿主机 | 带 `--execute`，输入 `ALIGN` | 完成绝对对齐后才开始人体动作 |

## 2. 环境准备

### 2.1 人员与现场

每次上机前确认：

- 吊架可靠，实时跟随时双脚着地、吊架只防跌；
- 一人操作电脑，一人负责手柄和现场实体急停；
- 左右手臂、躯干和腿周围无人、无桌椅、无线缆；
- 实体急停可用，现场人员知道正常停止和紧急停止的区别；
- 左臂接收器和发射器固定牢固，充电口和开关不被绑带压住；
- 手表、手机、金属扣和大型金属物远离电磁设备；
- 当前一轮实验中不再摘下或移动三个电磁设备。

设备一旦被摘下充电、重新绑扎或在支架中移动，本轮标定立即失效。旧 YAML、旧 NPZ
只能分析，不能用于新安装的实机控制。

### 2.2 宿主机软件环境

Unitree 脚本使用：

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc
```

ROS 2 使用 Docker 中的 ROS 2 Jazzy。检查容器：

```bash
docker ps -a --filter name=wb-mpc-dev
```

容器已运行时打开新终端：

```bash
docker exec -it wb-mpc-dev bash
```

容器存在但已停止时：

```bash
docker start wb-mpc-dev
docker exec -it wb-mpc-dev bash
```

容器尚未创建时：

```bash
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc/docker
./launch_wb_mpc.bash
```

## 3. 网络配置

### 3.1 G1 有线网络

当前机器人网卡为 `enx6c1ff701509c`，机器人地址为 `192.168.123.161`：

```bash
sudo ip link set enx6c1ff701509c up
sudo ip addr replace 192.168.123.222/24 dev enx6c1ff701509c
ip -br addr show enx6c1ff701509c
ping -I enx6c1ff701509c -c 4 192.168.123.161
```

通过条件：接口为 `UP,LOWER_UP`，四次 ping 全部返回。若网卡名变化，先运行：

```bash
ip -br link
```

不要在 CycloneDDS 或 Unitree 命令中继续使用一个已经不存在的旧接口名。

### 3.2 WEMT Wi-Fi 热点

热点接口通常为 `wlp0s20f3`，宿主机地址应为 `10.42.0.1/24`：

```bash
nmcli device status
ip -br addr show wlp0s20f3
ip neigh show dev wlp0s20f3
```

目前常用映射为：

- 大臂：`10.42.0.143`；
- 前臂：`10.42.0.144`。

IP 是 DHCP 结果，不能仅凭旧记录认定设备身份。更换接收器或 DHCP 地址变化时，逐只开机，
用 MAC、运动响应和邻居表确认哪一只是大臂、哪一只是前臂，然后在启动参数中显式填写。
IP 不会因为软件停止而自动“交换回来”。

## 4. 编译与终端初始化

代码修改后，在 Docker 中构建：

```bash
cd /wb_humanoid_mpc_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select remote_control --symlink-install
source install/setup.bash
```

只有代码发生变化时需要重新构建。每个新 Docker 终端仍需要运行：

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
```

构建成功不等于接收器在线，也不等于标定仍然有效；这三项必须分别检查。

## 5. 电磁设备、标定和仿真

### 5.1 启动双接收器

Docker 终端 `EM`：

```bash
ros2 run remote_control dual_em_tracker_node --ros-args \
  -p upper_arm_ip:=10.42.0.143 \
  -p forearm_ip:=10.42.0.144
```

必须看到：

```text
First synchronized pair published
```

另开终端检查两路频率：

```bash
ros2 topic hz /em/upper_arm_pose
ros2 topic hz /em/forearm_pose
```

两路都应稳定接近 50 Hz。不要同时运行 `em_tracker_node` 和
`dual_em_tracker_node`，它们会争用设备 TCP 连接。

### 5.2 当前安装重新标定

Docker 终端 `CALIBRATION`：

```bash
ros2 run remote_control dual_calibration_node
```

如果设备本轮被摘下过，在复用提示中输入 `n`。按程序完成：

1. 静态质量检查；
2. 接收器身份检查；
3. 前臂旋前/旋后轴检查；
4. `DOWN`、`FORWARD_30`、`FORWARD_60`、`FORWARD`、`SIDEWAYS`、
   `STRAIGHT_DOWN`、`STRAIGHT_FORWARD` 参考姿态。

必须看到标定保存信息。随后只读检查实机资格：

```bash
rg -n "dry_run_only|reference_max_error_deg|upper_reference_max_error_deg|forearm_reference_max_error_deg" \
  /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control/config/wmet_dual_calibration.yaml
```

- `dry_run_only: false`：可以继续仿真，通过后再进入实机；
- `dry_run_only: true`：只能 dry-run/MuJoCo，禁止实机发布；
- 设备移动、持续位置模型误差或姿态明显错误：重新安装并重新标定，不能放大阈值绕过。

### 5.3 重定向 dry-run

停止标定节点，保留 EM：

```bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=false
```

依次做安全的小动作，检查五个角度方向、连续性和静止漂移。完成后按 `Ctrl+C`，不要让该节点
与后面的 SIM 或 REAL 节点并存。

### 5.4 MuJoCo 检查

Docker 终端 `MUJOCO`：

```bash
ros2 launch g1_centroidal_mpc mujoco_sim.launch.py
```

Docker 终端 `RETARGET-SIM`：

```bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=true \
  -p simulation_only:=true \
  -p enable_torso_collision_filter:=true \
  -p torso_collision_margin_m:=0.025
```

先检查 DOWN、屈伸肘、前举、侧举、转掌以及返回动作。确认没有明显方向错误、跳变、躯干
穿透或长时间数据中断。完成后先停止 `RETARGET-SIM`，再停止 MuJoCo。

## 6. G1 开机与安全检查

### 6.1 只读检查

配置有线网络后，在宿主机运行：

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc

python3 humanoid_nmpc/remote_control/scripts/g1_lowstate_monitor.py \
  enx6c1ff701509c --duration 10 --print-period 1

python3 humanoid_nmpc/remote_control/scripts/g1_fsm_monitor.py \
  enx6c1ff701509c --duration 5
```

主要 FSM：

| `fsm_id` | 含义 | 是否允许实时跟随 |
| --- | --- | --- |
| 0 | 零力矩 | 否 |
| 1 | 阻尼 | 否 |
| 4 | 锁定站立、无自主平衡 | 仅固定动作/接管诊断 |
| 500 | 官方常规运控、自主平衡 | 是 |

模式切换后必须重新运行监视器，以读回的 `fsm_id` 为准，不能仅凭姿态或按键感觉判断。
手柄操作沿用实验室已经验证的流程：`L2+B` 进入/退出阻尼路径，`L2+↑` 进入 FSM 4，
`R1+X` 从 FSM 4 进入 FSM 500；若现场固件行为不同，应停止并由熟悉该机器人的人员确认，
不要尝试随机按键组合。

### 6.2 FSM 4 下的零动作接管检查

吊架可靠承重，机器人处于 FSM 4。先做只读固定测试：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_fixed_arm_test.py \
  enx6c1ff701509c \
  --joint left_elbow \
  --delta-deg 0
```

需要进一步验证 Arm SDK 接管时，先从低权重零动作诊断开始：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_fixed_arm_test.py \
  enx6c1ff701509c \
  --takeover-diagnostic \
  --diagnostic-max-weight 0.10 \
  --execute
```

输入 `DIAGNOSE`。通过后才按 `0.20 → 0.25` 逐级增加，不要在出现非目标关节速度时放宽
`0.30 rad/s` 保护阈值。已验证过当天的接管链路后，不必每轮重复所有权重。

### 6.3 进入 FSM 500

固定测试完全退出、Arm SDK 权重回零后：

1. 双脚着地，吊架只保留防跌余量；
2. 按已验证的手柄流程进入官方常规运控；
3. 等待机器人自主平衡稳定；
4. 再次运行 `g1_fsm_monitor.py`，必须稳定读到 `fsm_id=500`；
5. 明显踏步、倾斜或脚底接触不稳时不进入实机跟随。

## 7. 实机启动顺序

进入本节前，MuJoCo 和 `RETARGET-SIM` 必须已经停止。

### 7.1 Docker 终端 RETARGET-REAL

```bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=true \
  -p simulation_only:=false \
  -p enable_torso_collision_filter:=true \
  -p torso_collision_margin_m:=0.025
```

检查目标频率：

```bash
ros2 topic hz /arm_joint_target
```

预期接近 50 Hz。`dry_run_only: true` 的标定会被这里拒绝。

### 7.2 Docker 终端 UDP

```bash
ros2 run remote_control arm_target_udp_sender
```

应每秒打印一次 `sent pitch=... roll=... yaw=... elbow=... wrist=...`。如果人体动作在这里
没有反映出来，问题仍在 EM/重定向侧，不要启动实机 bridge。

### 7.3 宿主机 BRIDGE dry-run

绝对姿态跟随使用 `--absolute-alignment`。先保持一个安全、稳定、接近机器人当前姿态的
人体起始姿势，然后执行不带 `--execute` 的检查。示例五关节检查：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --absolute-alignment \
  --active-joints pitch roll yaw elbow wrist \
  --max-excursion-pitch-deg 60 \
  --max-excursion-roll-deg 60 \
  --max-excursion-yaw-deg 25 \
  --max-excursion-elbow-deg 60 \
  --max-excursion-wrist-deg 30 \
  --target-timeout 0.75
```

必须看到 `DRY-RUN RESULT: PASS`。每个 `max-excursion` 都以**接管前机器人实际角度**为中心，
不是人体目标的绝对上下限。dry-run 失败时应调整安全起始姿势或减小任务，不要盲目放大范围。

### 7.4 执行绝对对齐和跟随

执行命令后需要手动输入 `ALIGN`。在下面两行出现前，人体必须保持起始姿势不动：

```text
ABSOLUTE ALIGNMENT COMPLETE
arm_sdk enabled; tracking ROS targets.
```

只有看到这两行以后才开始测试动作。执行阶段 bridge 会检查：

- UDP 目标新鲜度；
- IMU 倾斜；
- 身体和未启用关节速度；
- 活动关节速度与 URDF 限位；
- 逐关节接管初始包络；
- 5°硬跟踪误差和提前 1°启动的 `LAG-GUARD`；
- 正常结束后的平滑权重释放。

## 8. 当前测试计划

同一时刻只能运行一个 `g1_arm_sdk_bridge.py`。每次结束必须看到：

```text
Releasing arm_sdk control...
arm_sdk weight returned to zero.
Saved ... real-robot tracking samples to ...csv
```

### 8.1 Wrist 单关节 W1：±15°

人体大臂保持不动，肘弯曲 60–90°，前臂朝前，初始 `wrist` 尽量为 `0±5°`。测试的是
前臂旋前/旋后，不是手腕屈伸。

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 100 \
  --alignment-rate-deg-s 2 \
  --max-rate-deg-s 5 \
  --max-excursion-wrist-deg 25 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.75 \
  --absolute-alignment \
  --active-joints wrist \
  --execute
```

动作：`0 → +15 → 0 → -15 → 0`，重复两遍，每个端点保持 2–3 秒。

### 8.2 Wrist 单关节 W2：±25°

W1 无 `STOPPED`、持续 `LAG-GUARD` 或持续 `CLIPPED` 后：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 120 \
  --alignment-rate-deg-s 2 \
  --max-rate-deg-s 5 \
  --max-excursion-wrist-deg 35 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.75 \
  --absolute-alignment \
  --active-joints wrist \
  --execute
```

动作：`0 → +15 → 0 → -15 → 0 → +25 → 0 → -25 → 0`。

### 8.3 五关节扩大范围：60°、5°/s

“60°”只用于 pitch、roll 和 elbow。yaw 与 wrist 继续使用已规划的安全范围：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 240 \
  --alignment-rate-deg-s 2 \
  --max-rate-deg-s 5 \
  --max-excursion-pitch-deg 60 \
  --max-excursion-roll-deg 60 \
  --max-excursion-yaw-deg 25 \
  --max-excursion-elbow-deg 60 \
  --max-excursion-wrist-deg 30 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.75 \
  --pitch-gravity-ff-nm 3.0 \
  --roll-gravity-ff-nm 3.0 \
  --absolute-alignment \
  --active-joints pitch roll yaw elbow wrist \
  --execute
```

动作顺序：回中、屈肘、前举、侧举、前举加屈肘、侧举加屈肘、弯肘状态转掌；每项后回中。
第一遍不追求撞到所有限幅。

### 8.4 五关节扩大范围：60°、8°/s

仅在 8.3 完整通过后执行：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 240 \
  --alignment-rate-deg-s 2 \
  --max-rate-deg-s 8 \
  --max-excursion-pitch-deg 60 \
  --max-excursion-roll-deg 60 \
  --max-excursion-yaw-deg 25 \
  --max-excursion-elbow-deg 60 \
  --max-excursion-wrist-deg 30 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.75 \
  --pitch-gravity-ff-nm 3.0 \
  --roll-gravity-ff-nm 3.0 \
  --absolute-alignment \
  --active-joints pitch roll yaw elbow wrist \
  --execute
```

当前已经验证 8°/s 在中等范围内可稳定跟随；本项用于验证扩大包络后的组合动作，不应直接
跳过 5°/s 档。若只有动作端点短暂 `CLIPPED`，说明触及包络；若整个动作持续 `CLIPPED`，
则该动作没有得到完整验证。

### 8.5 通过标准与后续任务

单次通过条件：

- 无 `STOPPED` 和目标流超时；
- 无持续 `LAG-GUARD`；
- 无持续 `CLIPPED`；
- 无异常方向、抖动、踏步或躯干碰撞；
- `desired` 与 `actual` 稳定跟随，最大误差不超过 5°；
- 最终权重回到零并成功保存 CSV。

上述测试全部通过后，不再无目的增加角度和速度。使用同一标准动作做三次重复性测试，随后
进入任务级演示：机器人左手依次到达前方、左侧、胸前附近三个安全目标区域，再返回起始位。
每次保存 CSV、重定向 NPZ、终端输出和固定视角视频。

## 9. 停止与关机顺序

### 9.1 正常停止

1. 人体停止并回到安全姿势；
2. 在 BRIDGE 终端按 `Ctrl+C`；
3. 等待 `arm_sdk weight returned to zero`；
4. 停止 UDP sender；
5. 停止 `RETARGET-REAL`，等待诊断 NPZ 保存；
6. 停止 EM；
7. 再次只读确认 FSM；
8. 吊架承重后，按实验室已验证流程退出 FSM 500，再确认 FSM 状态；
9. 最后按实验室 G1 关机流程断电。

不要把拔网线、关闭 Docker、直接关电脑或直接切断机器人电源当作正常 Arm SDK 释放方法。

### 9.2 异常停止

| 现象 | 第一动作 |
| --- | --- |
| 方向轻微错误、抖动但机器人仍稳定 | BRIDGE 终端按 `Ctrl+C`，等待权重回零 |
| 电脑失去响应，机器人暂时稳定且吊架已承重 | 使用已验证的手柄流程退出运控 |
| 快速不可控动作、倾倒、碰到人或即时危险 | 立即使用现场实体急停 |

紧急情况下人员安全优先，不等待软件释放。

## 10. 常见故障

### 10.1 `WMET pose timeout` 或一直等待接收器

- 确认热点、IP、设备电量和 TCP 801；
- 确认没有另一个单/双接收器节点占用设备；
- 用 `ip neigh show dev wlp0s20f3` 重新识别地址；
- 单接收器节点显示“tracking started”但无数据，不代表数据链已经正常。

### 10.2 四元数模长约 0.707

这是原始设备数据无效，不是安装方向或人体动作造成。停止标定和实机链路，关闭所有 WEMT
节点，必要时运行 `dual_raw_quaternion_check`。不能强制归一化或放宽 `0.95–1.05` 阈值。

### 10.3 `pair skew` / `Stale WEMT data`

偶发一条可记录观察；连续出现会造成目标中断。检查接收器电量、Wi-Fi 质量、设备是否被多个
程序连接以及热点负载。实机 bridge 出现 stale 后会释放，不能靠增大超时掩盖持续丢包。

### 10.4 `Upper affine position model error ... Frame dropped`

超过 50 mm 时帧被保护性丢弃。常见原因包括设备移动、绑带/皮肤滑动、当前姿态超出标定
覆盖或环境电磁畸变。若在前举全过程持续发生，停止实机；检查轮廓线和安装，再重新标定，
不要仅增大阈值。

### 10.5 标定文件不是 dual calibration 或 `dry-run-only`

- 确认配置文件来自当前 `dual_calibration_node`；
- 设备摘下后必须重标；
- `dry_run_only: true` 只允许 MuJoCo；
- 不为进入实机而扩大残差阈值。

### 10.6 `target stream stale`

检查 `RETARGET-REAL`、`/arm_joint_target` 频率和 UDP sender 是否持续输出。偶发 0.5 秒附近
中断可在确认网络稳定后使用当前已验证的 `--target-timeout 0.75`，但不应继续增大。

### 10.7 对齐中停止

- `human ... moved during absolute alignment`：人体在对齐期间移动，重新执行并保持不动；
- 对齐超时：起始差值过大、机器人负载大或目标不稳定；重新选择更接近实机的安全起始姿势；
- 对齐前 dry-run 不通过：不要输入 `ALIGN`，先处理差值和包络。

### 10.8 `LAG-GUARD`、跟踪误差或 `CLIPPED`

- `LAG-GUARD`：命令暂缓扩大，人体保持不动等机器人追上；持续出现则降低速度/范围；
- 跟踪误差达到 5°：硬停止，不能直接放大误差阈值；
- `CLIPPED`：目标超出以机器人接管前姿态为中心的本轮包络。短暂触及端点可接受，持续裁剪
  表示没有测试到人体要求的完整动作。

### 10.9 转动前臂但 wrist 不变

1. 先看 UDP/source 中的 wrist 是否变化；
2. 肘接近伸直时 yaw/wrist 不可观，重定向会冻结最后可靠值；测试 wrist 时保持肘弯
   60–90°；
3. source 变化但 bounded 不变并出现 `CLIPPED`，说明触及包络；
4. desired 变化而 actual 不跟随才是实机执行问题。此前日志中 wrist 实机跟随误差约 1°以内，
   尚未显示电机故障证据。

### 10.10 机器人不动

- bridge 不带 `--execute` 时本来就不会动；
- 未输入 `ALIGN` 或尚未出现 `arm_sdk enabled` 时不要期待实时跟随；
- 确认 FSM 500、UDP 新鲜、`active-joints` 包含目标关节；
- 相对模式只跟踪起点后的变化；最终姿态跟随应使用 `--absolute-alignment`。

### 10.11 其他常见信息

- `does not match an available interface`：网卡未连接、未 UP 或名字错误；
- 日志目录 `Permission denied`：使用默认仓库内 `calibration_logs`，并修正目录属主/权限；
- `[Reader] take sample error`：若只在退出后偶发且本轮日志完整，可记录；若运行中持续出现，
  停止并检查 DDS/网络；
- `rcl_shutdown already called` 或退出时 rosout 无效：若只发生在 `Ctrl+C` 清理阶段，通常不
  代表此前采集失败；仍应确认实际节点已停止。

## 11. 数据保存与判定

每轮至少保存：

- `calibration_logs/g1_arm_tracking_*.csv`；
- `/wb_humanoid_mpc_ws/calibration_logs/dual_retargeting_*.npz`；
- `/wb_humanoid_mpc_ws/calibration_logs/dual_reference_*.npz`；
- 当轮 `wmet_dual_calibration.yaml` 副本；
- FSM、EM、重定向、UDP 和 BRIDGE 终端输出；
- 固定视角视频和动作时间表。

正式结果建议统计每个关节的 RMS 误差、最大误差、动作延迟、裁剪比例、保护触发次数以及
三次重复的离散程度。视觉标记的 EM-vs-视觉真值实验是另一条数据链，不是 G1 实机安全验证
的前置条件。

## 12. 当前状态与待办事项

截至 2026-11-05：

- G1 有线通信、FSM 只读检查、Arm SDK 缓慢接管/释放已验证；
- 双接收器标定、MuJoCo、REAL/UDP/BRIDGE 链路已连通；
- 五关节中等范围的 5°/s 与 8°/s 跟随已通过，当前主要限制是分关节包络而非电机跟踪；
- 当前待完成 W1、W2、五关节 60°/5°/s 和 60°/8°/s；
- 大范围测试通过后，用统一动作连续重复三次，形成正式重复性数据；
- 选定并固化“三个安全目标区域”的任务级演示，定义成功半径、完成时间和失败条件；
- 为正式实验增加统一的 session 名称、标定 YAML 快照、终端日志和 CSV/NPZ 索引；
- 分析 wrist 可观测冻结、持续 `CLIPPED` 区域和重力前馈收益，不在正式数据中途修改参数；
- 完成两手机视觉真值流程，用于人体 EM 角度的独立精度评估；
- 最终演示固定使用已经验证的范围和速度，不再现场临时扩大到未验证包络。

