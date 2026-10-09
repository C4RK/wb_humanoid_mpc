# 一发两收电磁手臂跟踪流程

这套流程使用一个固定在躯干上的发射端、一个大臂接收端和一个前臂接收端。它通过功能标定自动估计肩关节和肘关节中心，不需要输入人体臂长。

## 1. 安装约定

- 发射端：使用 `X-Y` 的 4 cm × 8 cm 大平面固定在胸前安装面；`+X` 朝人体左侧，`+Y` 朝上，`+Z` 朝前。
- 大臂接收端：位于肘上方约 8–12 cm，局部 `+Y` 从肩指向肘。
- 前臂接收端：位于腕横纹上方约 5–8 cm，局部 `+Y` 从肘指向腕。
- 使用胶带时，在皮肤和设备边缘画轮廓线。任何设备越过轮廓线都必须重新标定。
- 三个设备附近移除手表、手机和金属扣。

双接收器标定会求出实际安装旋转，因此几度的方向误差可以补偿。设备在标定后发生移动则无法补偿。

## 2. 确认两个接收器的网络地址

电脑开启电磁设备所用的 Wi-Fi 热点后，在宿主机运行：

```bash
ip -br addr show wlp0s20f3
ip neigh show dev wlp0s20f3
```

分别关闭和开启两个接收器，可以根据邻居表中新增或消失的地址识别它们。记录为：

- `UPPER_IP`：大臂接收器；
- `FOREARM_IP`：前臂接收器。

当前确认的设备映射为：

- 大臂接收器 U（新接收器）：`10.42.0.143`，MAC `48:27:e2:e3:4c:50`；
- 前臂接收器 F（原接收器）：`10.42.0.144`，MAC `48:27:e2:e3:60:4c`。

`ip neigh` 会在设备关机后暂时保留缓存项，因此旧地址仍显示一段时间是正常现象。

## 3. 进入 Docker 并构建

启动脚本会自动把工作区旁边的 `WMET-API` 只读挂载到容器。如果它不在 `/home/kunwang/WMET-API`，启动前设置 `WEMT_API_DIR`。

```bash
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc/docker
./launch_wb_mpc.bash
```

在容器中：

```bash
cd /wb_humanoid_mpc_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select remote_control --symlink-install
source install/setup.bash
```

每次重新进入容器后需要再次执行最后一条 `source install/setup.bash`。

## 4. 启动双接收器采集

容器终端 1：

```bash
ros2 run remote_control dual_em_tracker_node
```

默认地址已经设置为大臂 `10.42.0.143`、前臂 `10.42.0.144`，默认端口均为 801。如果地址发生变化，可以覆盖参数：

```bash
ros2 run remote_control dual_em_tracker_node --ros-args \
  -p upper_arm_ip:=UPPER_IP \
  -p forearm_ip:=FOREARM_IP
```

如果端口不同，可以增加：

```bash
-p upper_arm_port:=801 -p forearm_port:=801
```

节点输出：

- `/em/upper_arm_pose`
- `/em/forearm_pose`

在另外两个终端分别检查：

```bash
ros2 topic hz /em/upper_arm_pose
ros2 topic hz /em/forearm_pose
```

两路都应接近 50 Hz，并且采集节点不应持续报告 stale 或 pair skew。

## 5. 运行功能标定

保持采集节点运行，在容器终端 2 执行：

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control dual_calibration_node
```

程序依次执行：

1. 静态噪声检查；
2. 肩关节功能中心运动；
3. 肘关节屈伸和前臂旋前/旋后；
4. 手臂下垂、前举和侧举参考姿态，各重复三次；
5. 保存拟合残差、条件数、安装旋转和功能臂长。

标定文件保存到：

```text
/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control/config/wmet_dual_calibration.yaml
```

正常情况下建议满足：

- 静态位置 RMS 小于 3 mm；
- 静态姿态 RMS 小于 3°；
- 肩和肘功能中心 RMS 小于 20 mm；
- 三组参考姿态最大残差小于 10°。

胶带固定第一次可能超过这些阈值。先检查轮廓线和胶带张力，再重新采集，不要直接放宽阈值。

## 6. 只在仿真中检查关节角

保持采集节点运行，停止标定节点，再启动：

```bash
ros2 run remote_control dual_retargeting_node
```

先不启动 MPC，观察输出：

```bash
ros2 topic echo /arm_joint_target --once
ros2 topic hz /arm_joint_target
```

启动 MuJoCo：

```bash
ros2 launch g1_centroidal_mpc mujoco_sim.launch.py
```

启动顺序建议为：采集节点、重定向节点、MuJoCo/MPC。启动 MPC 前，让人体保持标定时的“上臂前举、肘弯 90°、前臂向上”参考姿态，以减小初始目标跳变。

实时节点包含：

- 60 ms 一阶低通滤波；
- 2.5 rad/s 默认关节速度限制；
- 肩中心和肘中心在线一致性检查；
- 250 ms 数据看门狗。

这些参数可以通过 ROS 参数调整。在完成仿真记录和误差评估前，不要增大关节速度限制。
