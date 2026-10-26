# 一发两收电磁手臂跟踪流程

这套流程使用一个固定在躯干上的发射端、一个大臂接收端和一个前臂接收端。它通过七个已知参考姿态标定大臂与前臂的方向，不需要输入人体臂长，也不依赖皮肤上不稳定的肩、肘功能中心。

## 1. 安装约定

- 发射端：使用 `X-Y` 的 4 cm × 8 cm 大平面贴在胸骨中线上，不要贴在左锁骨下方或胸大肌上；`+X` 朝人体左侧，`+Y` 朝上，`+Z` 朝前。
- 大臂接收端：位于肘上方约 8–12 cm 的大臂外侧或略偏后，避开三角肌和肱二头肌肌腹，局部 `+Y` 从肩指向肘。胶带应绕过大臂形成一圈，避免接收端只粘在一小块皮肤上滑动。
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

正式双设备采集前，使用原来的单设备节点逐个做桌面检查。不要同时运行
`em_tracker_node` 和 `dual_em_tracker_node`，因为它们会同时连接同一 TCP 设备。

原前臂接收器：

```bash
ros2 run remote_control em_tracker_node --ros-args -p device_ip:=10.42.0.144
```

新大臂接收器：

```bash
ros2 run remote_control em_tracker_node --ros-args -p device_ip:=10.42.0.143
```

保持接收器姿态不变，分别沿发射端 `+X/+Y/+Z` 平移约 10 cm；对应坐标应增加约
0.10 m。另开终端运行 `ros2 run remote_control check_quaternion_stability`，把接收器
局部 `+Y` 分别对准发射端 `+X/+Y/+Z`，`forearm_dir` 应依次接近三个对应的单位向量。
完成一只后按 `Ctrl+C` 停止，再测试另一只。

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

采集节点在发布前检查设备给出的原始四元数模长。正常值应接近 `1.0`；默认只接受
`0.95–1.05`。超出范围的整对数据会被丢弃，不能进入标定或重定向。如果持续出现：

```text
Invalid raw WEMT quaternion norm: upper arm=... forearm=...
```

应停止标定，并断电重启日志中指出的接收器。不要通过放宽阈值或强制归一化继续实验。
需要区分设备原始输出与 ROS 转换问题时，先停止所有 `em_tracker_node` 和
`dual_em_tracker_node`，再单独运行：

```bash
ros2 run remote_control dual_raw_quaternion_check
```

该诊断直接读取两路原始位置、四元数和硬件时间戳，不读取或修改活动标定文件。

在另外两个终端分别检查：

```bash
ros2 topic hz /em/upper_arm_pose
ros2 topic hz /em/forearm_pose
```

两路都应接近 50 Hz，并且采集节点不应持续报告 stale 或 pair skew。

## 5. 运行双接收器混合标定

保持采集节点运行，在容器终端 2 执行：

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control dual_calibration_node
```

任何设备被摘下、充电或重新粘贴后，在复用提示中输入 `n`；只有三个设备都完全没有移动时
才输入 `y`。

程序依次执行：

1. 静态噪声检查；
2. 接收器对应关系检查：大臂保持不动，只屈伸肘部并转动前臂；
3. 前臂旋转轴标定：大臂下垂、肘弯 90°、小臂向前，保持肩、肘和手腕位置不动。
   从拇指向上开始，只向手掌朝下和朝上方向各转约 45°，回到拇指向上，重复两次；
   不要转到人体活动范围的极限；
4. 下垂、前举 30°、前举 60°、前举、侧伸、整臂下垂伸直和整臂前平举伸直七个参考姿态，各重复三次；
5. 用大臂接收器的位置轨迹求大臂方向；其中两个前举中间姿态用于覆盖连续前举轨迹；用原有五个静态参考姿态单独拟合前臂
   纵向方向，用 STEP 2 的旋前/旋后动作单独拟合腕部扭转轴。程序会同时检验前臂四元数
   的正向与逆向定义，优先选择扭转轴方向残差较小的定义；完整相对旋转包含人的腕部扭转
   和参考动作误差，因此只作为评分相同时的辅助项。

标定参数与当次安装绑定。只要发射端、大臂接收端或前臂接收端中的任何一个被摘下、
重新粘贴或在胶带内转动，都要重新标定。程序发现旧标定文件时默认重新采集；只有三个
设备从上次标定后始终没有移动，才在提示中输入 `y` 复用。仅断电且安装完全未动可以
复用，摘下充电后不能复用。

本项目当前的实验流程规定：每轮实验结束后设备会被摘下充电，因此该轮标定随即失效。
诊断目录中的 `.npz` 只用于分析算法和误差，不能恢复成后续重定向使用的活动标定；旧安装
下成功保存的 YAML 同样不能用于新安装。重定向只能使用当前安装保持不变期间，由
`dual_calibration_node` 完整执行并显示 `Saved to ...` 后生成的标定文件。

七个参考姿态为：

- 下垂：大臂竖直向下，前臂水平向前，左掌朝右/身体内侧，拇指向上；
- 前举 30°：从下垂姿态开始，把保持 90°弯肘的整条 L 形左臂向前转约 30°；不转掌、不耸肩；
- 前举 60°：同样保持 90°弯肘，把大臂继续向前转到约 60°；不转掌、不耸肩；
- 前举：大臂水平向前，肘弯曲 90°，前臂自然竖直向上且不主动扭转；左掌朝右/身体内侧，
  拇指朝身体后方（朝向自己）；
- 侧伸：整条左臂向左水平伸直，肘伸直，手掌朝下，拇指向前。
- 整臂下垂伸直：左大臂保持自然下垂，肘完全伸直，整条左臂位于身体左侧；左掌朝右/
  身体内侧，拇指朝身体正前方。
- 整臂前平举伸直：整条左臂在肩高处完全伸直并指向身体正前方，不要跨向胸前或向外偏；
  左掌朝右/身体内侧，拇指向上。

“整臂前平举伸直”与“下垂”中的前臂都指向身体正前方，但上臂和肘部状态不同。程序将
两次观测共同用于前臂方向拟合，以减小胶带和软组织随姿态变化造成的正前方系统偏差；
它也会单独打印 `straight_forward` 残差，直接检验前平举工作区。新增姿态不能消除机器人
自身连杆轴线与理想人体模型之间的差异，后者仍需在 G1 关节映射中处理。

每次保持采集结束后先离开该姿态，再重新摆好下一次姿态，然后按 Enter。这样三次重复会
测到人工重复摆位误差，而不只是传感器静态噪声。

胶带固定在皮肤上时，大臂接收器的姿态会受到肩胛运动和软组织位移影响。主流程因此
不使用大臂接收器的姿态，而是根据它相对发射端的位置轨迹估计大臂方向。旧版将轨迹
强制拟合到一个固定球面，可能出现 DOWN 与 FORWARD 端点正常、前举中段却越过半径保护的
情况。新版用 DOWN、FORWARD_30、FORWARD_60、FORWARD 和 SIDEWAYS 等参考建立仿射椭球
位置模型，并继续用 50 mm 模型残差作为安装移动保护；前臂方向和
腕部旋转仍使用前臂接收器姿态，但分别使用静态参考方向轴与动态扭转轴。肩关节俯仰和侧摆由大臂位置方向计算，肩轴向旋转由
大臂与前臂形成的肘平面计算，肘角由两段方向夹角计算。该模型不需要卷尺输入人体臂长，
也不依赖功能中心扫掠。

标定文件保存到：

```text
/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control/config/wmet_dual_calibration.yaml
```

每次采完七个参考姿态，原始平均位置和旋转都会保存到：

```text
/wb_humanoid_mpc_ws/calibration_logs/dual_reference_*.npz
```

即使标定因残差过大而停止，该诊断文件仍然保留，后续可以直接分析，不必反复盲目重做动作。

正常情况下建议满足：

- 静态位置 RMS 小于 3 mm；
- 静态姿态 RMS 小于 3°；
- 对应关系检查中，大臂话题转动小于 15°、前臂话题转动大于 25°；
- 前臂旋转轴步骤中，大臂转动小于 15°、前臂总转动范围大于 55°；轴拟合 RMS 建议
  小于 8°，胶带固定时允许到 15°；
- 大臂位置方向、前臂姿态方向和腕部参考的最大残差建议小于 10°；
- 使用独立旋前/旋后轴时，人体解剖旋转轴与理想化的肘到手腕方向可能有系统夹角。
  前臂方向残差在 10–25° 时程序会保留完整误差并只允许警告保存，用于干跑验证；
  超过 15°的标定会写入 `dry_run_only`，重定向节点会拒绝开启关节目标发布；
  大臂或腕部参考超过 15°、前臂方向超过 25°时仍会拒绝保存；
- 大臂位置模型 RMS 小于 30 mm，最大误差小于 50 mm。

单次参考姿态的三次重复若相差超过 15°，程序只重新采集当前姿态。前臂旋转轴步骤最多
尝试三次；连续失败时程序会退出并要求重新固定前臂接收器，不会无限要求重复动作。
大臂位置模型刚超过建议值时程序会给出警告并询问是否保存；
只有 RMS 超过 35 mm 或最大误差超过 65 mm 时才拒绝整轮标定。

胶带固定第一次可能超过这些阈值。先检查轮廓线和胶带张力。若各姿态三次重复良好、
旋转轴拟合良好，但前臂方向在多轮标定中稳定保持 15–20°，应将其视为模型与人体解剖轴的
系统差异并进入干跑验证，不再要求操作者盲目重复整套动作。下一次运行若发现旧文件超过
上述硬限制，会自动拒绝旧文件并开始新的标定。

## 6. 只在仿真中检查关节角

保持采集节点运行，停止标定节点，再启动：

```bash
ros2 run remote_control dual_retargeting_node
```

首次检查标定时使用干运行模式。节点仍会计算并打印五个关节角，但不会向
`/arm_joint_target` 发布：

```bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=false
```

`publish_joint_targets` 默认就是 `false`。完成 dry-run 验证、准备连接 MPC/MuJoCo 时，
必须显式开启仿真专用发布：

```bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=true \
  -p simulation_only:=true
```

该模式只发布 `/arm_joint_target_sim`，`g1_centroidal_mpc` 的 MuJoCo 仿真会订阅
这个话题。即使当前标定因前臂参考误差略高于 15°而标记为 `dry_run_only`，也可以显式
使用该模式做仿真观察。

`simulation_only:=false` 时发布的是 `/arm_joint_target`。该话题保留给后续实机链路，
并且 `dry_run_only` 标定仍会被严格拒绝。

先不启动 MPC，在终端检查五个角度。动作与符号通过后，重新使用默认发布模式启动节点，
再检查目标话题：

```bash
ros2 topic echo /arm_joint_target_sim --once
ros2 topic hz /arm_joint_target_sim
```

启动 MuJoCo：

```bash
ros2 launch g1_centroidal_mpc mujoco_sim.launch.py
```

启动顺序建议为：采集节点、MuJoCo/MPC、仿真专用重定向节点。等 MuJoCo 中机器人稳定站立后，保持人体零位对应姿态再启动重定向：左上臂自然下垂、肘弯 90°、前臂水平向前、左掌朝向身体内侧、拇指向上。此时五个目标关节角应接近零，可以减小初始目标跳变。

实时节点包含：

- 60 ms 一阶低通滤波；
- 2.5 rad/s 默认关节速度限制；
- 250 ms 数据看门狗。
- 默认开启的 G1 左臂—躯干碰撞过滤器。过滤器使用 `g1_23dof.urdf` 的关节轴和固定
  变换做正向运动学，将大臂、前臂和手近似为胶囊体，并在发布前保持至少 25 mm 的
  模型间隙。危险目标优先通过最小幅度增加左肩 roll 投影到安全区；45°范围内找不到
  安全解时保持上一安全目标。它不会修改人体标定，也不会把修正量反馈到人体关节解算。

碰撞保护生效时，终端会出现一次 `Torso collision protection activated`，常规状态输出
会附带 `torso guard`、肩 roll 修正量和保护后的间隙。完整 NPZ 同时记录：

- `pre_collision_joint_angles_rad`：碰撞保护前的五个目标角；
- `commanded_joint_angles_rad`：实际发给 MPC 的安全目标角；
- `collision_raw_clearance_m` 和 `collision_safe_clearance_m`；
- `collision_filter_active`、`collision_filter_held` 和
  `collision_roll_adjustment_rad`。

仿真调试时可显式设置参数：

```bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=true \
  -p simulation_only:=true \
  -p enable_torso_collision_filter:=true \
  -p torso_collision_margin_m:=0.025
```

`enable_torso_collision_filter:=false` 只用于 MuJoCo 中做前后对照，禁止用于实机。

正侧平举时，大臂方向接近身体横向轴，`shoulder_pitch` 在数学上不可观测。实时节点在该
区域统一选用 `sideways_pitch_target_rad=0.0` 的 G1 标准解，并通过低通和速度限制平滑
进入该解。这样从前平举转到侧平举和从下垂直接转到侧平举会得到相同的肩部姿态，不再
保留进入奇异区之前的 pitch。该参数只用于侧平举奇异区，正常前举和下垂仍由测量方向
决定。在标准 pitch 分支内，roll 使用 `atan2` 连续通过 90°，因此 MuJoCo 可测试到
约 110° 的侧上举。G1 左肩 roll 上限约 129°；超过上限时节点会警告并截断目标。

这些参数可以通过 ROS 参数调整。在完成仿真记录和误差评估前，不要增大关节速度限制。
