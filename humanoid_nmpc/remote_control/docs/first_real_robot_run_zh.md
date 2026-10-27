# G1 首次实机重定向：逐步命令与判定标准

这份清单用于第一次把已经通过 MuJoCo 检查的双接收器重定向接到 23 自由度 Unitree
G1。第一次实机实验只做原地站立和左肘 5–10° 的慢速屈伸，不行走、不做完整前平举或
侧平举。

本文把命令分为两个运行环境：

- **宿主机终端**：Ubuntu 桌面上的普通终端，提示符类似 `kunwang@kw:~$`。Unitree
  SDK 状态读取和 `rt/arm_sdk` 安全桥在这里运行；
- **Docker 终端**：进入 `wb-mpc-dev` 后的终端，提示符类似
  `root@kw:/wb_humanoid_mpc_ws#`。ROS 2、电磁设备、标定、重定向和 MuJoCo 在这里运行。

不要把宿主机命令复制到 Docker，也不要在实机阶段启动本项目的 centroidal MPC。当前
实机方案使用 G1 自带运控保持下肢平衡，本项目只通过 `rt/arm_sdk` 接入手臂目标。

## 0. 控制链路和停止原则

仿真链路为：

```text
双接收器 -> dual_retargeting_node -> /arm_joint_target_sim
         -> centroidal MPC -> MuJoCo
```

首次实机链路为：

```text
双接收器 -> dual_retargeting_node -> /arm_joint_target
         -> arm_target_udp_sender -> localhost UDP 15000
         -> g1_arm_sdk_bridge.py -> G1 rt/arm_sdk
```

正常停止时，在运行 `g1_arm_sdk_bridge.py --execute` 的宿主机终端按 `Ctrl+C`，然后等待：

```text
Releasing arm_sdk control...
arm_sdk weight returned to zero.
```

出现不可控快速动作、机器人倾倒或人员危险时使用现场实体急停。`L2+B` 会进入阻尼模式；
从 FSM 500 等运控状态退出时需要同时长按约 5 秒。机器人随后不再自主平衡，只有吊架已经
能够承重且现场人员准备好时才能把它作为模式中止手段。
不要把拔网线、关闭终端或 `docker stop` 当成正常释放控制的方法。

现场停止分三级：

| 现象 | 第一动作 | 后续动作 |
| --- | --- | --- |
| 方向轻微错误、抖动但机器人仍稳定 | BRIDGE 终端按 `Ctrl+C` | 等待权重回零；结束本轮 |
| 电脑失去响应，但机器人暂时稳定且吊架已经承重 | 手柄同时长按 `L2+B` 约 5 秒 | 机器人进入阻尼，不要扶手臂；随后处理程序 |
| 快速不可控运动、即将倾倒、碰到人或存在即时危险 | 现场实体急停 | 吊架承重并清场；本轮禁止直接恢复 |

因此 `L2+B` 不是所有异常的第一按钮，也不是实体急停。双脚着地、吊架只防跌时，不要期待
按下 `L2+B` 后机器人继续站立。如果时间允许且机器人仍稳定，优先在 BRIDGE 终端按
`Ctrl+C` 走两秒平滑释放；若已经有人身或倾倒危险，直接使用实体急停。

`rt/arm_sdk` 的混合权重会影响双臂。程序虽然只改变左臂五个目标，但会保持右臂和腰部在
接管时的位置，因此机器人左右两侧都必须无人、无桌椅、无线缆。

## 1. 现场人员和硬件

开始前逐项确认：

1. 吊架连接可靠；固定 5° 测试时吊架承重、双脚离地；实时跟随时双脚着地、吊架只防跌；
2. 一人操作电脑，一人始终拿手柄并能立即操作实体急停；
3. 机器人两臂、躯干和腿周围没有人员、桌椅或线缆；
4. 机器人网线接在 `enx6c1ff701509c`，机器人地址为 `192.168.123.161`；
5. 电磁热点网卡为 `wlp0s20f3`，热点地址为 `10.42.0.1/24`；
6. 发射器、大臂接收器和前臂接收器安装后画好轮廓线；整轮实验不再摘下或移动；
7. 实体急停、手柄和吊架负责人都知道本节的正常停止与紧急停止区别。

任何电磁设备被摘下充电或重新粘贴后，旧 YAML 和旧 NPZ 都不能用于新的实机实验，必须
重新执行第 5 节标定。

## 2. 启动或进入 Docker

### 2.1 宿主机：检查容器是否已经存在

```bash
docker ps -a --filter name=wb-mpc-dev
```

含义：只查询名为 `wb-mpc-dev` 的容器，不启动或停止任何程序。

- 如果状态为 `Up`，在新宿主机终端运行：

  ```bash
  docker exec -it wb-mpc-dev bash
  ```

  含义：在正在运行的容器里再打开一个 Docker 终端。

- 如果状态为 `Exited`，运行：

  ```bash
  docker start -ai wb-mpc-dev
  ```

  含义：启动已有容器并连接它的主终端。

- 如果没有任何结果，运行：

  ```bash
  cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc/docker
  ./launch_wb_mpc.bash
  ```

  含义：新建带宿主机网络和图形界面的 `wb-mpc-dev`，并将整个工作区挂载为
  `/wb_humanoid_mpc_ws`。若已经存在同名容器，不要再次运行启动脚本。

### 2.2 Docker：构建一次 ROS 包

在一个 Docker 终端运行：

```bash
cd /wb_humanoid_mpc_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select remote_control --symlink-install
source install/setup.bash
```

各行含义：

1. 进入容器内 ROS 工作区；
2. 加载 ROS 2 Jazzy；
3. 构建本项目的 `remote_control` 包；
4. 加载刚构建的节点。

只有修改代码后需要重新 `colcon build`。每打开一个新的 Docker 终端，都必须运行：

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
```

## 3. 检查电磁设备网络

### 3.1 宿主机：确认热点和两个接收器

```bash
nmcli device status
ip -br addr show wlp0s20f3
ip neigh show dev wlp0s20f3
```

预期：

- `wlp0s20f3` 为热点连接并具有 `10.42.0.1/24`；
- 大臂接收器为 `10.42.0.143`；
- 前臂接收器为 `10.42.0.144`。

IP 不会自己交换。真正的定义是“哪一只设备安装在大臂/前臂”。若重新交换了物理安装，
必须在启动节点时相应交换参数，不能仅为了消除报错而随意交换 IP。

### 3.2 Docker：安装前逐个做桌面方向检查

先不要运行双接收器节点。每次只打开并移动一只接收器，另一只关机。

检查前臂接收器：

```bash
ros2 run remote_control em_tracker_node --ros-args \
  -p device_ip:=10.42.0.144
```

检查大臂接收器：

```bash
ros2 run remote_control em_tracker_node --ros-args \
  -p device_ip:=10.42.0.143
```

含义：单独连接一只接收器，便于确认设备身份、位置三轴和四元数响应。保持姿态不变，
分别沿发射端 `+X/+Y/+Z` 平移约 10 cm，对应位置分量应增加约 0.10 m。每只检查完成后
按 `Ctrl+C`，再启动另一只。不要同时运行单接收器节点和双接收器节点，因为它们会争用
同一个设备 TCP 连接。

桌面检查通过后，先按第 4 节把设备固定到身体；安装完成后回到下面的 3.3，启动双接收器
采集。这里特意先检查再粘贴，是为了避免安装后才发现设备身份或坐标响应异常。

### 3.3 Docker 终端 EM：启动双接收器采集

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control dual_em_tracker_node --ros-args \
  -p upper_arm_ip:=10.42.0.143 \
  -p forearm_ip:=10.42.0.144
```

含义：通过 TCP 801 同时读取两只接收器，同步后发布：

```text
/em/upper_arm_pose
/em/forearm_pose
```

通过标志：出现 `First synchronized pair published`。偶发一条 `pair skew` 可以记录观察；
若持续出现 stale、无同步数据或四元数模长不在 `0.95–1.05`，停止在这里。四元数持续为
`0.707` 左右属于无效原始数据，不能放宽阈值或强制归一化进入标定。

另开两个 Docker 终端分别运行：

```bash
ros2 topic hz /em/upper_arm_pose
```

```bash
ros2 topic hz /em/forearm_pose
```

含义：只测量两个 ROS 话题的实际频率。两路都应稳定接近 50 Hz。确认后按 `Ctrl+C` 停止
这两个频率检查，保留 `dual_em_tracker_node` 运行。

## 4. 安装电磁设备

- 发射端：胸骨中线，4 cm × 8 cm 大平面贴身体；约定 `+X` 向人体左、`+Y` 向上、
  `+Z` 向前；
- 大臂接收端：左肘上方约 8–12 cm，位于大臂外侧或略偏后；胶带绕大臂一圈；
- 前臂接收端：腕横纹上方约 5–8 cm，固定在前臂上，不在手背或腕关节；
- 移除手表、手机、金属扣等靠近设备的物品。

安装方向不必达到机械加工级平行，标定会估计安装旋转；但传感器必须与对应肢段一起运动，
不能在皮肤和胶带内滑动。安装完成后不再移动三个设备。

## 5. Docker：重新标定当前安装

保持 EM 采集运行，在新的 Docker 终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control dual_calibration_node
```

如果程序询问：

```text
Reuse existing calibration only if no device moved? [y/N]:
```

本项目每次都会摘下设备充电，所以输入 `n`。输入 `y` 会直接沿用旧安装标定，这次禁止。

### STEP 0：静态质量检查

自然站立，躯干和整条左臂保持静止，按 Enter 后静止 4 秒。

意义：测量环境、电磁噪声和胶带微动。建议位置 RMS 小于 3 mm、姿态 RMS 小于 3°。
超出时先检查金属、距离和胶带；不要连续用 `y` 跳过明显异常。

### STEP 1：接收器身份检查

保持躯干和左大臂不动，只反复屈伸肘部并转动前臂，持续 6 秒。

意义：确认 `/em/upper_arm_pose` 真来自大臂，`/em/forearm_pose` 真来自前臂。通过条件为：

- upper excursion 小于 15°；
- forearm excursion 大于 25°；
- forearm 至少比 upper 多 15°。

### STEP 2：前臂旋转轴标定

左大臂竖直下垂，肘弯 90°，前臂水平朝正前方。肩、肘和手腕位置保持不动。以拇指向上
为中位，缓慢转到约 45° 手掌向下，回中，再转到约 45° 手掌向上，回中；重复两次，
持续 8 秒。不要转到人体极限，不要弯手腕。

意义：单独识别旋前/旋后的轴，用于机器人腕 roll。通过条件为大臂转动不超过 15°、前臂
运动范围至少 55°、轴拟合 RMS 不超过 15°。连续三次失败后应重新固定前臂接收器。

### 七个静态参考姿态

每个姿态做三次。每次按 Enter 后保持 2 秒；采完后先完全离开姿态，再重新摆好下一次，
不要原地不动连续按三次 Enter。

1. **DOWN**：左大臂竖直下垂；肘弯 90°；前臂水平朝正前方；左掌朝右/身体内侧；
   拇指向上。
2. **FORWARD_30**：从 DOWN 出发，保持肘弯 90°，把整条 L 形左臂向前转约 30°；
   前臂斜向前上方；不要转掌、耸肩或前倾。
3. **FORWARD_60**：继续保持肘弯 90°，把大臂向前转到约 60°；该姿态位于
   FORWARD_30 与水平前举之间；不要转掌、耸肩或前倾。
4. **FORWARD**：左大臂水平朝正前方；肘弯 90°；前臂竖直向上；左掌朝右/身体内侧；
   拇指朝身体后方。这里不用额外扭手腕。
5. **SIDEWAYS**：整条左臂向左水平伸直；肘伸直；手掌朝下；拇指朝正前方。
6. **STRAIGHT_DOWN**：整条左臂在身体左侧自然垂直并完全伸直；左掌朝右/身体内侧；
   拇指朝正前方。
7. **STRAIGHT_FORWARD**：整条左臂在肩高处完全伸直并朝身体正前方；不要跨到胸前、
   不要向外偏；左掌朝右/身体内侧；拇指向上。

`FORWARD_30` 和 `FORWARD_60` 只约束大臂的位置轨迹。前臂方向、腕部旋转和肘角校正仍
使用原有五个参考姿态，避免新增中间姿态改变已经验证过的前臂映射。

这里的角度是大臂相对竖直下垂方向的角度。`FORWARD_30` 应明显低于 45°，大约只走完
下垂到水平前举路程的三分之一；`FORWARD_60` 才是三分之二。程序会利用大臂接收器的
相对转角检查这三个前举参考，偏离目标超过 12° 时只要求重采当前姿态。不要把手机或
金属量角器靠近电磁接收器；需要辅助时使用纸质角度模板或让另一人在侧面观察。

完成后必须看到：

```text
[Dual calibration] Saved to .../wmet_dual_calibration.yaml
```

若程序询问是否带警告保存，可以用于仿真分析；要进入实机，最终 YAML 必须满足
`dry_run_only: false`。用下面的只读命令检查：

```bash
rg -n "dry_run_only|reference_max_error_deg|upper_reference_max_error_deg|forearm_reference_max_error_deg" \
  /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control/config/wmet_dual_calibration.yaml
```

含义：显示标定是否允许实机发布及关键残差，不修改文件。看到 `dry_run_only: true` 时只能
继续做 MuJoCo，不能进入第 11–12 节的实机目标发布和实时跟随。

WARNING 不等于禁止实机。当前阈值的含义是：

| 检查量 | 结果 | 含义 |
| --- | --- | --- |
| 最大参考残差不超过 10° | 正常保存 | 标定质量达到建议值 |
| 最大参考残差超过 10° | 询问是否带警告保存 | 可以保存，但必须继续检查下面的实机资格 |
| 前臂方向最大残差不超过 15° | `dry_run_only: false` | 允许完成 MuJoCo 后进入受限实机测试 |
| 前臂方向最大残差超过 15°、不超过 25° | `dry_run_only: true` | 只允许干运行和 MuJoCo，禁止实机目标发布 |
| 大臂参考超过 15°、前臂超过 25°或腕扭转参考超过 15° | 拒绝保存 | 需要重新安装或重新采集 |

过去常见的 10–14° 残差通常会出现 WARNING，但仍可能得到 `dry_run_only: false`，这种结果
不需要放大阈值。不要单纯为绕过 `dry_run_only` 而把 15°或25°阈值调大；这会把算法已经
判定为不可靠的肩、肘或腕方向直接交给实机。若长期只能得到 15°以上的前臂残差，应先保留
MuJoCo 测试，或另行实现只允许单个关节的小范围实机模式，而不是开放整条五关节目标。

标定成功后在标定终端按 `Ctrl+C`，保留 EM 采集终端。

## 6. Docker：先做重定向干运行

第 6 节和第 7 节不同时运行。第 6 节先单独运行，用来观察数值；通过后按 `Ctrl+C` 结束
这个 `dual_retargeting_node`。第 7 节再启动 MuJoCo，并启动一个新的仿真发布版
`dual_retargeting_node`。EM 采集节点在两节期间一直保持运行。

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=false
```

含义：实时计算并打印人体对应的 G1 左肩 pitch/roll/yaw、左肘和左腕 roll，但不发布任何
机器人或仿真目标。此时依次做：

1. DOWN 保持 3 秒；
2. 只缓慢屈伸肘 20–30°后回到 DOWN；
3. 整臂伸直下垂；
4. 小幅向外抬臂约 20°后返回。

通过条件：静止时角度不持续漂移；动作方向正确；没有几十度瞬间跳变；没有持续出现
`No valid receiver pair`、旧标定的 `Upper position radius error`，或新版标定的
`Upper affine position model error`。完成后按 `Ctrl+C`，确认生成本次
`dual_retargeting_*.npz`。

## 7. Docker：MuJoCo 复核同一次安装和标定

第 7 节内部的 MuJoCo/MPC 和 RETARGET-SIM 必须同时运行，但位于两个 Docker 终端。先启动
MuJoCo/MPC，等机器人站稳，再让人体保持 DOWN 并启动 RETARGET-SIM。

### 7.1 Docker 终端 SIM：启动 MuJoCo 和 MPC

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 launch g1_centroidal_mpc mujoco_sim.launch.py
```

含义：启动仿真机器人和项目 MPC。等待机器人稳定站立，并检查启动信息中出现
`EM arm tracking subscriber ready` 和 `Initial MPC policy received`。零星的 MPC 慢循环警告
可以记录；持续严重超时或机器人不稳定时不要继续。

### 7.2 Docker 终端 RETARGET-SIM：发布仿真专用目标

人体先摆好 DOWN，然后运行：

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=true \
  -p simulation_only:=true \
  -p enable_torso_collision_filter:=true \
  -p torso_collision_margin_m:=0.025
```

参数意义：

- `publish_joint_targets:=true`：允许发布关节目标；
- `simulation_only:=true`：只发布 `/arm_joint_target_sim`，实机 UDP 发送器不订阅它；
- `enable_torso_collision_filter:=true`：启用左臂与躯干碰撞保护；
- `torso_collision_margin_m:=0.025`：模型至少保留 25 mm 间隙。

仿真动作只做：DOWN、轻微屈伸肘、小幅向外抬臂、返回 DOWN。确认方向正确、没有躯干
穿透或突然跳变。完成后先在 RETARGET-SIM 按 `Ctrl+C`，再停止 MuJoCo。保留 EM 采集。

实机阶段不要运行 `mujoco_sim.launch.py`，也不要同时保留仿真重定向节点。

## 8. 宿主机：建立 G1 有线通信并只读检查

完成第 3–7 节、同一次安装没有移动、MuJoCo 结果通过且标定为
`dry_run_only: false` 后，可以进入第 8 节。第 8 节只检查机器人网络和基本状态，不会创建
机器人命令发布器，也不会让机器人运动。

### 8.0 G1 实体开机

1. 先不运行任何 Unitree 控制程序。机器人挂在吊架上并由吊架可靠承重，双脚离地；手臂
   和腿自然放置，所有关节没有互相缠绕或被外物卡住；
2. 确认电池完全插入、卡扣到位、充电器已经断开，实体急停和手柄负责人就位；
3. 在**机器人电池的电源键**上短按一次并松开；
4. 再按第二次，这一次持续按住至少 2 秒，直到电池和机器人开始上电，然后松开；
5. 等待约 1 分钟完成启动。启动期间不要按模式组合键，不要运行控制 publisher；
6. 当全部关节进入零力矩状态后，认为初始化完成。此时机器人不能自行站立，必须继续由
   吊架承重；
7. 如果 R3 手柄尚未开机，使用手柄自己的电源键同样执行“短按并松开，再长按至少2秒”。
   手柄开机与机器人电池开机是两个独立操作；
8. 先执行下面的网络、LowState 和 FSM 只读检查，再按第 9 节进入阻尼和锁定站立。

开机动作中的“短按 + 长按”是两次独立按压。不要从关机状态直接一直长按。机器人没有在
约一分钟后进入预期状态、出现电池报警或异常声音时，不要反复按模式键；保持吊架承重并
请现场熟悉该设备的人员检查电池、卡扣和急停状态。

### 8.1 配置有线地址

```bash
sudo ip addr replace 192.168.123.222/24 dev enx6c1ff701509c
sudo ip link set enx6c1ff701509c up
ip -br addr show dev enx6c1ff701509c
ping -I enx6c1ff701509c -c 4 192.168.123.161
```

含义：给机器人网卡配置同网段静态地址并测试通信。预期显示本机
`192.168.123.222/24`，四次 ping 全部返回。该有线连接不需要默认网关，也不应抢占手机或
Wi-Fi 的互联网路由。

### 8.2 进入 Unitree Python 环境

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc
```

含义：加载已安装 `unitree_sdk2py` 的宿主机环境并进入脚本所在仓库。终端出现
`libtinfo.so.6: no version information available` 是 Conda 的库提示，不代表机器人通信失败。

### 8.3 只读检查 LowState

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_lowstate_monitor.py \
  enx6c1ff701509c --duration 15
```

含义：只订阅 `rt/lowstate`，不创建命令发布器。必须连续收到状态；机器人静止时 IMU
roll/pitch 应接近零，关节速度应接近零。五秒内收不到状态时停止排查网络。

### 8.4 只读检查 FSM

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_fsm_monitor.py \
  enx6c1ff701509c --duration 10
```

含义：只向 `sport` 服务注册 getter，读取 `fsm_id`、`fsm_mode` 和 `balance_mode`，没有模式
setter 和关节发布器。主要状态为：

| `fsm_id` | 含义 | 本次用途 |
| --- | --- | --- |
| 0 | 零力矩 | 禁止发手臂目标 |
| 1 | 阻尼 | 禁止发手臂目标；机器人不自主平衡 |
| 4 | 锁定站立 | 仅吊架承重的固定 5° 测试 |
| 500 | 单腰常规运控 | 双脚着地、站稳后的实时手臂跟随 |

不要用 `mode_machine` 代替 `fsm_id` 判断，因为朋友的实机记录中 FSM 1 和 FSM 4 都曾显示
相同的 `mode_machine=4`。

## 9. 手柄进入 FSM 4 并重复固定 5° 测试

本节所有 `g1_fsm_monitor.py`、`g1_fixed_arm_test.py` 命令都在**宿主机终端**的
`unitree-rl` Conda 环境执行，不在 Docker 中执行。

吊架承重、双脚离地，两侧手臂下方无人。当前没有任何 `rt/arm_sdk` publisher。

1. 从零力矩状态按 `L2+B` 进入阻尼；若当前处于 FSM 500 等运控状态，同时长按约 5 秒；
   观察手臂可能下垂；重新运行 FSM 监视器，必须读到 `fsm_id=1`；
2. 按 `L2+↑` 进入锁定站立；等待机器人摆好；重新运行 FSM 监视器，必须读到
   `fsm_id=4`。

按键和机器人姿态只是操作线索，最终以 `fsm_id` 读回为准。如果不是目标状态，不要尝试
其他随机组合。

### 9.1 固定动作 dry-run

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_fixed_arm_test.py \
  enx6c1ff701509c --joint left_elbow --delta-deg 5
```

含义：读取机器人状态并检查 IMU、关节速度和关节限位，不创建 `rt/arm_sdk` publisher。
必须看到 `DRY RUN ONLY`。

这里的 dry-run 意为“把程序运行到真实动作之前，但不发控制命令”。它验证网卡、LowState、
关节编号、当前姿态和安全检查是否正常。机器人不会动，也不会跟随人体。

### 9.2 执行固定动作

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_fixed_arm_test.py \
  enx6c1ff701509c --joint left_elbow --delta-deg 5 --execute
```

检查终端打印的初始角和目标角，确认现场清空后输入：

```text
MOVE
```

程序用 20 秒缓慢接管、3 秒移动约 +5°、保持 1 秒、3 秒返回，再根据当时的 Arm SDK
权重用最多 20 秒缓慢降到零。通过条件是左肘方向正确、机器人身体稳定、另一只手臂没有
异常动作，并最终出现：

```text
Fixed arm test completed; arm_sdk weight returned to zero.
```

没有出现该释放信息时，不进入下一步。

“执行固定动作”是预先写死的左肘 +5° 测试，不读取人体动作，也不是电磁重定向。它的目的
是先独立证明电脑到 `rt/arm_sdk` 的实机控制方向、接管和释放都正确。

### 9.3 接管阶段出现非目标关节速度时

如果固定动作在 `takeover` 阶段因为另一只手臂的关节速度而停止，不要放宽
`0.30 rad/s` 保护阈值，也不要直接重复固定动作。`rt/arm_sdk` 的权重会同时影响
左右手臂，因此先运行零动作、低权重接管诊断：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_fixed_arm_test.py \
  enx6c1ff701509c \
  --takeover-diagnostic \
  --diagnostic-max-weight 0.10 \
  --execute
```

运行前仍需确认吊架、急停、周围空间和 FSM 4。看到提示后输入 `DIAGNOSE`。诊断过程不改变
任何关节目标。程序按每单位权重 20 秒自动延长接管和释放时间：`0.10` 至 `0.25` 至少使用
5 秒，`0.50` 使用 10 秒，`0.75` 使用 15 秒，`1.00` 使用 20 秒；到达目标权重后保持 2 秒。
程序会
继续使用 `0.30 rad/s` 速度保护和 `2°` 零动作位移保护，并在结束或提前停止时列出每个上身
关节的峰值速度和峰值位移。权重按 `0.10 → 0.20 → 0.25 → 0.50 → 0.75 → 1.00`
逐级测试，每一级结果经过检查后再继续，不要自行跳级。完整权重零动作诊断通过后，才能继续
固定动作测试。

## 10. 手柄进入 FSM 500

1. 确认固定测试已退出，没有任何用户 Arm SDK publisher；
2. 在 FSM 4 下让双脚接触地面，吊架保留防跌余量，不向上吊住机器人；
3. 手柄按 `R1+X`，进入单腰常规运控；
4. 等机器人原地自主平衡稳定；
5. 再运行一次只读 FSM 检查：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_fsm_monitor.py \
  enx6c1ff701509c --duration 10
```

必须稳定读到 `fsm_id=500`。机器人踏步、明显晃动、脚未稳定接触地面或 FSM 不是 500 时，
停止在这里。不要进入 debug，不调用 `ReleaseMode()`，不要启动 `rt/lowcmd`。

## 11. Docker：启动实机目标链，但先不让机器人运动

此时需要保留四个终端：EM、RETARGET-REAL、UDP，以及宿主机 BRIDGE。

第 11 节是完整实机信号链的“通电前检查”：人体动作会被计算成关节目标并传到宿主机，
但宿主机 BRIDGE 不带 `--execute`，因此不创建 Unitree 命令发布器，机器人仍不会跟随。
第一次真正根据人体动作运动是在第 12 节输入 `TRACK` 之后。

第 11 节全程保持 DOWN，不做屈肘、抬臂或转腕。这里要检查的是 DOWN 第一帧能否完整传到
宿主机，以及它与机器人当前垂臂姿态是否接近。ROS 和 UDP 中虽然持续存在目标数据，但它
只到达本机安全桥；不带 `--execute` 时不会发布到机器人 `rt/arm_sdk`。BRIDGE dry-run 打印
一帧目标并退出后，人体仍保持 DOWN，直接准备第 12 节。只有输入 `TRACK`、看到
`arm_sdk enabled; tracking ROS targets.`，并完成开头三秒静止后，才从 DOWN 缓慢伸直
左肘 5–10°。当前标定把 DOWN 的人体肘角定义为 0°并限制最小值为 0°，所以从 DOWN
继续弯曲不会产生可用的负目标；第一次实机动作必须是小幅伸肘再返回。

### 11.1 Docker 终端 RETARGET-REAL

人体先恢复并保持标定时的 DOWN 姿态，然后运行：

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control dual_retargeting_node --ros-args \
  -p publish_joint_targets:=true \
  -p simulation_only:=false \
  -p enable_torso_collision_filter:=true \
  -p torso_collision_margin_m:=0.025
```

含义：发布实机专用 `/arm_joint_target`。`simulation_only:=false` 不会自行控制机器人；只有
后面的 UDP 发送器和宿主机 `--execute` 安全桥同时存在时，目标才可能到达机器人。
`dry_run_only: true` 的标定会在这里被程序拒绝。

另开 Docker 终端检查一次：

```bash
ros2 topic hz /arm_joint_target
```

预期接近 50 Hz，确认后按 `Ctrl+C` 停止频率检查。

### 11.2 Docker 终端 UDP

```bash
source /opt/ros/jazzy/setup.bash
source /wb_humanoid_mpc_ws/install/setup.bash
ros2 run remote_control arm_target_udp_sender
```

含义：从 `/arm_joint_target` 取左臂五个角度，打包后发送到
`udp://127.0.0.1:15000`。Docker 使用 host network，所以宿主机安全桥能在同一地址收到。
这个节点本身不包含 Unitree SDK，不能直接驱动机器人。预期每秒打印一行 `sent pitch=...`。

UDP 是一种轻量级网络数据报协议。这里它只用于同一台电脑内部从 Docker 把五个目标角
送到宿主机：

```text
Docker ROS 2 /arm_joint_target
  -> UDP 127.0.0.1:15000
  -> 宿主机 g1_arm_sdk_bridge.py
  -> Unitree DDS rt/arm_sdk
  -> 机器人有线网卡
```

UDP 本身不保证每个包都到达，所以发送包包含序号和时间戳，安全桥还要求目标年龄不超过
500 ms；流中断时安全桥停止跟随并释放 Arm SDK 权重。它没有把电磁数据直接通过互联网
发送到机器人。

### 11.3 宿主机 BRIDGE：完整链路 dry-run

人体继续保持 DOWN，机器人保持 FSM 500，运行：

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --relative-to-start \
  --active-joints elbow \
  --max-excursion-deg 15
```

含义：读取 LowState 和一帧 UDP 目标，检查目标有效性，然后退出；因为没有 `--execute`，
不会创建 `rt/arm_sdk` publisher。必须看到：

```text
DRY RUN: received ...; no command publisher was created.
```

`--relative-to-start` 把第一帧人体目标对齐到接管瞬间的机器人姿态，之后只应用人体相对
第一帧的角度变化。这样不会要求 FSM 500 当前约 56° 的左肘突然跳到仿真零位。第一次
实验的 `--active-joints elbow` 只允许左肘跟踪，肩和腕保持接管时的实机角度。

程序会逐关节打印 `actual`（实机当前角）、`source`（人体重定向原始角）、`applied`
（对齐后准备应用的角）和 `difference`。必须看到 `mapping mode: relative-to-start`、
`active tracked joints: elbow` 和 `DRY-RUN RESULT: PASS`。第一帧的 `applied` 应等于
`actual`；原始 `source` 与 `actual` 可以不同，因为它们来自仿真零位和 FSM 500 默认站姿
两套起始姿态。若对齐后的差值仍然失败，不要增大限制。

## 12. 宿主机：第一次 10 秒实时跟随

确认以下条件同时成立：

- FSM 刚刚读到 500，机器人自主平衡稳定；
- 人体保持 DOWN；
- EM、RETARGET-REAL 和 UDP 三个 Docker 终端都在稳定运行；
- 吊架只防跌，实体急停负责人就位；
- 机器人两侧清空；
- 没有其他 Arm SDK、`rt/lowcmd` 或控制程序。

当前安全桥会连续检查 LowState、姿态、速度、跟踪误差和目标超时，但不会在同一进程中
连续查询 `fsm_id`。因此开始前的 FSM 500 读回和手柄负责人在场不能省略；如果手柄切换
了模式，本轮实验立即结束，不在运行中重新接管。

运行：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 10 \
  --max-rate-deg-s 5 \
  --max-excursion-deg 15 \
  --max-body-speed-rad-s 0.30 \
  --target-timeout 0.50 \
  --relative-to-start \
  --active-joints elbow \
  --execute
```

参数意义：

- `--duration 10`：接管完成后只跟随 10 秒；
- `--max-rate-deg-s 5`：每个受控关节命令变化最多 5°/s；
- `--max-excursion-deg 15`：每个目标最多偏离机器人接管姿态 15°；
- `--max-body-speed-rad-s 0.30`：腿、腰、右臂和未启用的左臂关节超过该速度即停止；
- `--target-timeout 0.50`：500 ms 没有新目标就停止，允许双接收器同步产生的短暂空档；
- `--relative-to-start`：把第一帧人体目标对齐到机器人当前姿态，只跟踪后续相对变化；
- `--active-joints elbow`：本轮只让左肘跟随，肩和腕保持接管时角度；
- `--execute`：允许建立 `rt/arm_sdk` publisher，因此这一条会让实机运动。

程序先显示收到的五个新鲜目标。再次确认后，准确输入：

```text
TRACK
```

输入后人体继续保持 DOWN。程序用 20 秒将 Arm SDK 权重从零缓慢升到 1；接管期间仍持续
检查 UDP 目标是否新鲜。相对模式会在接管完成时用最新人体目标重新设置基准，因此接管期间
正常呼吸和小幅姿态变化不会积累成待执行动作。看到
`arm_sdk enabled; tracking ROS targets.` 后，10 秒跟随计时才开始，之后的动作才会被跟踪。
结束或安全检查触发后，程序根据当时权重用最多 20 秒缓慢释放。

十秒动作时间表：

1. 前 3 秒保持 DOWN，不动；
2. 用约 2 秒从 DOWN 缓慢伸直左肘 5–10°，肩和腕保持不动；
3. 保持约 1 秒；
4. 用约 2 秒返回 DOWN；
5. 剩余时间静止。

第一次不要前平举、侧平举、贴近躯干、旋转整臂或快速转掌。程序会持续检查目标新鲜度、
IMU 倾斜、身体与保持关节速度、左臂速度和跟踪误差；异常时停止跟随并进入两秒权重释放。

通过标志：

```text
arm_sdk enabled; tracking ROS targets.
active source/applied/bounded/desired/actual deg: ...
Releasing arm_sdk control...
arm_sdk weight returned to zero.
Saved ... full-rate real-robot tracking samples to ...csv
```

## 13. 正常结束顺序

1. 先让人体回到 DOWN；
2. 等 BRIDGE 自动到时退出，或在 BRIDGE 终端按 `Ctrl+C`；
3. 必须等到 `arm_sdk weight returned to zero`；
4. 停止 Docker 终端 UDP；
5. 停止 RETARGET-REAL，确认保存 `dual_retargeting_*.npz`；
6. 停止 EM；
7. 再运行一次 FSM 只读检查并记录状态；
8. 机器人退回阻尼或关机时，按照实验室既有流程并确保吊架已承重。从 FSM 500 退出时同时
   长按 `L2+B` 约 5 秒，并用 FSM 监视器确认 `fsm_id=1`。不要在本文未验证的手柄组合之间
   尝试切换。

保存以下材料：

- `/wb_humanoid_mpc_ws/calibration_logs/dual_reference_*.npz`；
- `/wb_humanoid_mpc_ws/calibration_logs/dual_retargeting_*.npz`；
- `~/humanoid_mpc_ws/src/wb_humanoid_mpc/calibration_logs/g1_arm_tracking_*.csv`；
- 标定、重定向、UDP、安全桥和 FSM 终端输出；
- 固定视角现场视频。

## 14. 第一次实验的通过条件

- 当前安装的标定成功且 `dry_run_only: false`；
- MuJoCo 中 DOWN 和小幅伸肘再返回的方向正确，无躯干碰撞；
- 固定 5° 测试在 FSM 4 下完成并正常释放权重；
- 实时接管前后 FSM 均为 500；
- 接管和释放时机器人不晃动、不踏步；
- 左肘实际角跟随目标，方向正确且没有振荡；
- 没有 stale target、倾斜、超速或跟踪误差停止；
- 最终明确看到 Arm SDK 权重回到零，并保存 CSV 与 NPZ。

全部满足后，下一次实验才把实时时长增加到 20 秒，并依次加入约 5° 的肩 pitch、向外
shoulder roll 和腕 roll。完整 90° 动作、行走以及本项目全身 MPC 真机控制都不属于第一次
实机实验。

## 15. 手柄与控制方式依据

- 朋友仓库 FSM 1/4 与停止实测：
  <https://github.com/jiakefu9-hub/hold-my-beer-mpc/blob/main/docs/g1_field_validation/sessions/20260915_A2_STOP_INTERLOCK.md>
- 朋友仓库 FSM 4 固定手臂动作实测：
  <https://github.com/jiakefu9-hub/hold-my-beer-mpc/blob/main/docs/g1_field_validation/sessions/20260915_A2_WEIGHT05.md>
- 朋友仓库 FSM 500 落地平衡手臂动作实测：
  <https://github.com/jiakefu9-hub/hold-my-beer-mpc/blob/main/docs/g1_field_validation/sessions/20260916_A3_HARDWARE.md>
- Unitree 当前手柄模式和 `rt/arm_sdk` weight 说明：
  <https://github.com/unitreerobotics/xr_teleoperate/wiki/Motion>

## 16. 已归档的预实验

- [2026-10-22 G1 左臂实时重定向预实验记录](records/2026-10-22_real_robot_pretest/README_zh.md)：
  包含 33 份实机 CSV 的统计、一次安全中止、当天 7 份标定原始数据、终端记录和文件校验值。
- [2026-10-23 G1 左臂逐级扩大范围测试](records/2026-10-23_real_robot_range_test/README_zh.md)：
  包含三档分关节安全包络的四次运行、一次目标流超时停止、逐关节跟踪误差和数据连续性统计。

## 17. 五关节通过 20° 后的功能动作测试

### 17.1 五个电机是否同时变化

安全桥以 50 Hz 运行，每 20 ms 把腰部、左右臂保持目标和已启用的左臂目标写进同一个
`rt/arm_sdk` 命令包。因此，`--active-joints pitch roll yaw elbow wrist` 时，五个左臂关节
在同一个控制周期收到目标。腿部不由这个桥控制，继续由 FSM 500 负责平衡。

五个关节不会被强制走完全相同的轨迹。人体每个关节的变化量不同，每个关节分别经过幅度
限制和 `--max-rate-deg-s` 速度限制；机器人各电机的负载和响应也不同。所以它们可以同时
开始更新，但停止时间、峰值角度和实际跟踪误差可以不同。未列入 `--active-joints` 的左臂
关节以及右臂和腰部保持接管时的角度。

### 17.2 本轮目标

已经完成五关节 5°、10°、15° 和 20° 的相对跟随验证后，下一步做一次中等范围的功能
动作序列，检查关节组合、动作换向和躯干碰撞。先使用以下分关节上限：

- pitch：20°；
- roll：20°；
- yaw：15°；
- elbow：30°；
- wrist：20°；
- 所有关节命令限速：5°/s。

这些角度都是相对于接管结束时机器人姿态的最大变化量，不是机器人的绝对关节角。

### 17.3 先做 dry-run

确认 RETARGET-REAL 和 UDP sender 连续输出、机器人保持 FSM 500 后，在宿主机运行：

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc

python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --relative-to-start \
  --active-joints pitch roll yaw elbow wrist \
  --max-excursion-pitch-deg 20 \
  --max-excursion-roll-deg 20 \
  --max-excursion-yaw-deg 15 \
  --max-excursion-elbow-deg 30 \
  --max-excursion-wrist-deg 20
```

这一条没有 `--execute`，机器人不会运动。必须看到五行关节信息以及
`DRY-RUN RESULT: PASS`。

### 17.4 实机功能序列

dry-run 通过后运行：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 90 \
  --max-rate-deg-s 5 \
  --max-excursion-pitch-deg 20 \
  --max-excursion-roll-deg 20 \
  --max-excursion-yaw-deg 15 \
  --max-excursion-elbow-deg 30 \
  --max-excursion-wrist-deg 20 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.50 \
  --relative-to-start \
  --active-joints pitch roll yaw elbow wrist \
  --execute
```

安全桥默认在硬跟踪误差阈值前 1°启动滞后保护。例如
`--max-tracking-error-deg 5` 时，活动关节的命令与实测角接近 4°差值后，桥接器会暂停
继续扩大该关节命令，等待实机关节追上；误差回落后自动继续，并在终端标记
`LAG-GUARD`。5°硬停止仍然保留。该机制根据实机跟踪能力动态减速，不改变人体目标，
也不允许用它掩盖电磁丢帧、URDF 越界或姿态异常。

输入 `TRACK` 后，在 20 秒接管期间保持当前基准姿态。只有看到
`arm_sdk enabled; tracking ROS targets.` 后才按下面时间表动作：

1. 0–5 秒：保持基准姿态；
2. 5–15 秒：缓慢屈肘约 20°，再回到基准；
3. 15–30 秒：缓慢小幅前举约 15°，再回到基准；
4. 30–45 秒：缓慢向左侧抬约 15°，再回到基准；
5. 45–60 秒：pitch 和 roll 同时变化约 10°，做一次斜向抬臂，再回到基准；
6. 60–75 秒：肩部小幅前举约 10°，同时屈肘约 20°，再回到基准；
7. 75–85 秒：前臂缓慢向两个方向各旋转约 10°，每次都回中；
8. 85–90 秒：回到基准并保持。

整个序列保持躯干稳定，动作之间回到基准，不追求碰到限幅。出现碰撞趋势、踏步、突然反向、
明显抖动或异常关节方向时，立即停止人体动作，并在桥接终端按 `Ctrl+C`；等待
`arm_sdk weight returned to zero`。本轮结束后先分析新 CSV，再决定是否扩大到
pitch 30°、roll 25°、yaw 15°、elbow 45°、wrist 30°。

### 17.5 同一天逐步扩大范围

三个测试必须串行完成，不能同时运行多个 `g1_arm_sdk_bridge.py`。每一档结束后必须看到
`arm_sdk weight returned to zero`，确认 FSM 仍为 500，并让人体和机器人回到各自的基准
姿态。若出现 `STOPPED`、碰撞趋势、踏步、异常方向、明显抖动或无法回到基准，当天停止扩大
范围。设备没有移动时，三个测试之间不需要重新标定。

第一档就是 17.4 的中等范围测试：pitch 20°、roll 20°、yaw 15°、elbow 30°、wrist 20°，
跟随时间 90 秒。

第一档通过后，运行第二档：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 100 \
  --max-rate-deg-s 5 \
  --max-excursion-pitch-deg 25 \
  --max-excursion-roll-deg 20 \
  --max-excursion-yaw-deg 15 \
  --max-excursion-elbow-deg 35 \
  --max-excursion-wrist-deg 25 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.50 \
  --relative-to-start \
  --active-joints pitch roll yaw elbow wrist \
  --execute
```

第二档动作顺序：保持 5 秒；屈肘并返回；前举并返回；侧举并返回；转掌并回中；斜向抬臂
并返回；前举同时屈肘并返回；最后保持 5 秒。动作期间不必刻意撞到每一个限幅值。

第二档通过后，运行第三档：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 120 \
  --max-rate-deg-s 5 \
  --max-excursion-pitch-deg 30 \
  --max-excursion-roll-deg 25 \
  --max-excursion-yaw-deg 15 \
  --max-excursion-elbow-deg 45 \
  --max-excursion-wrist-deg 30 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.50 \
  --relative-to-start \
  --active-joints pitch roll yaw elbow wrist \
  --execute
```

第三档在看到 `arm_sdk enabled` 后按以下时间安排：

1. 0–5 秒：保持基准；
2. 5–25 秒：缓慢屈肘到较大范围，保持 2 秒，再返回；
3. 25–40 秒：缓慢前举，再返回；
4. 40–55 秒：缓慢侧举，再返回；
5. 55–70 秒：肩 yaw 向一个方向变化、回中、向另一方向变化、再回中；
6. 70–95 秒：前臂向一个方向旋转、回中、向另一方向旋转、再回中；
7. 95–115 秒：做一次斜向抬臂加屈肘的复合动作，再返回；
8. 115–120 秒：保持基准。

`--duration` 只计算完全接管后的跟随时间。每条命令之前另有 20 秒接管，结束后最多还有
20 秒释放。在 `arm_sdk enabled` 之前不要开始动作。在同一天的三个测试中保持
`--max-rate-deg-s 5` 不变，避免把幅度效应和速度效应混在一起。

## 18. 从相对验证切换到绝对姿态跟随

### 18.1 两种映射的区别

前面的实机验证使用 `--relative-to-start`：接管结束时的人体目标被映射到机器人当时姿态，
之后只执行人体目标的变化量。这能避免接管时的大幅动作，但同一个人体姿势在不同的机器人
启动姿态下可能得到不同的机器人姿势。

最终动作跟随使用 `--absolute-alignment`。双接收器重定向输出本身已经是 G1 模型的绝对
关节目标；安全桥在 20 秒接管结束前收集 2 秒稳定目标，随后冻结该目标，以默认 2°/s 的
速度把已启用的实机关节对齐过去。实机在目标误差小于 2° 并保持 1 秒后才显示：

```text
ABSOLUTE ALIGNMENT COMPLETE
arm_sdk enabled; tracking ROS targets.
```

此后机器人直接跟踪绝对重定向目标。配置的安全包络仍然有效；超出包络的目标会被截断。
`--relative-to-start` 和 `--absolute-alignment` 不能同时使用。带 `--execute` 时必须明确选择
其中一种，防止遗漏映射参数后意外控制实机。

绝对对齐期间必须保持人体起始姿势不动。以下任一条件都会停止并平滑释放控制：

- 接管末段的目标波动超过 2°；
- 对齐期间人体活动超过 4°；
- UDP 目标过期；
- LowState 中断、身体倾斜、非活动关节运动或跟踪误差越界；
- 绝对目标超过本轮的分关节位移上限或 URDF 范围；
- 90 秒内没有完成对齐。

### 18.2 首次绝对对齐只测试左肘 15°

标定 `DOWN` 中人体肘目标通常接近 0°，而 FSM 500 启动后的 G1 左肘通常约为 55–56°。
第一次绝对对齐不直接跨越这约 56°。先在 RETARGET-REAL 和 UDP sender 正常运行时观察：

```text
sent ... elbow=... deg
```

缓慢调整并保持人体肘部，使 UDP 的 `elbow` 稳定在 **45–55°**。肩和手腕保持不动。这只是
第一次机制验证的起始姿势；它不要求保持标定 `DOWN`。机器人保持 FSM 500，吊架只防跌，
现场仍按第 1 节配置人员、手柄和实体急停。

先在宿主机执行 dry-run：

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc

python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --absolute-alignment \
  --active-joints elbow \
  --max-excursion-elbow-deg 15 \
  --target-timeout 0.75
```

这一条不会创建命令发布器。确认输出包含：

```text
mapping mode: absolute-alignment
DRY-RUN RESULT: PASS
```

如果失败，查看 `elbow` 行的 `actual`、`source`、`difference`。调整并重新保持人体肘部，让
`difference` 小于 15°；第一次不要为了通过而放大 15° 上限。

dry-run 通过后执行：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 30 \
  --alignment-rate-deg-s 1 \
  --max-rate-deg-s 3 \
  --max-excursion-elbow-deg 15 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.75 \
  --absolute-alignment \
  --active-joints elbow \
  --execute
```

看到目标和限幅信息正确后输入：

```text
ALIGN
```

随后按这个顺序操作：

1. 20 秒接管期间继续保持人体起始肘姿势；机器人应保持原姿态；
2. 出现 `Aligning the robot...` 后仍然保持不动；左肘以不超过 1°/s 对齐绝对目标；
3. 看到 `ABSOLUTE ALIGNMENT COMPLETE` 和 `arm_sdk enabled` 后，再保持 5 秒；
4. 人体肘目标缓慢增加约 5°，保持 2 秒，再回到起始值；
5. 人体肘目标缓慢减小约 5°，保持 2 秒，再回到起始值；
6. 最后保持不动，等待程序结束并确认 `arm_sdk weight returned to zero`。

通过标准：绝对对齐没有 `STOPPED`；终端中的 `source/applied/bounded` 数值相同且没有
`CLIPPED`；机器人实际肘角收敛到相同的绝对目标；两次约 5° 的动作方向正确；没有踏步、
振荡或碰撞趋势；CSV 正常保存。若人体目标超过本轮安全包络，`bounded` 会停在允许边界并
显示 `CLIPPED`，此时机器人不会执行超出边界的部分。通过后再把允许的初始差值按 15°、
30°、45° 分级扩大，最后验证人体 `DOWN` 到机器人绝对 `DOWN`。不要从本节首测直接跳到
五关节完整绝对对齐。

## 19. 肩 Pitch 重力前馈首轮验证

大臂前举时，如果命令保持不动后仍存在随前举角增大的稳态误差，这不是通信延迟或单纯的
速度限制。安全桥提供一个默认关闭的实验参数 `--pitch-gravity-ff-nm`。它只作用于左肩
Pitch，按 `tau = peak * sin(commanded_pitch)` 生成前馈力矩；`peak` 是肩 Pitch 为正负
90° 时的力矩幅值。默认值为 0 N·m，首轮调试的软件硬上限为 3 N·m。没有显式写出该参数
时，既有控制行为完全不变。

首次只用 1 N·m 幅值，不直接使用 3 N·m。保持 RETARGET-REAL 与 UDP sender 连续运行，
人体起始 Pitch 保持在约 0°。机器人使用吊架保护，现场人员继续持有急停。执行：

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc

python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 60 \
  --alignment-rate-deg-s 1 \
  --max-rate-deg-s 1 \
  --max-excursion-pitch-deg 35 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.75 \
  --pitch-gravity-ff-nm 1.0 \
  --absolute-alignment \
  --active-joints pitch \
  --execute
```

输入 `ALIGN` 后，在 `ABSOLUTE ALIGNMENT COMPLETE` 之前保持人体 Pitch 不动。完成后按以下
顺序操作：

1. 0° 附近保持 5 秒；
2. 缓慢前举到 Pitch 约 -15°，保持 10 秒；
3. 缓慢返回 0°，保持到程序结束。

终端会额外显示 `ff=...Nm`。-15°、幅值 1 N·m 时理论前馈约为 -0.26 N·m。CSV 新增
`command_pitch_gravity_ff_nm` 列。若动作方向错误、误差增大、出现振荡或异常声音，立即按
宿主机 `Ctrl+C` 并准备使用急停，不得继续提高幅值。通过条件是方向正确、无振荡，且 -15°
保持段的稳态误差不大于无前馈基线约 1.8°。只有比较完这份 CSV 后，才决定是否进行
2 N·m 以及 -30° 的第二档测试。

## 20. 肩 Roll 重力前馈分级验证

肩 Roll 前馈默认关闭，由 `--roll-gravity-ff-nm` 显式启用。桥接器把接管瞬间测得的实机
Roll 角作为 DOWN 零点，使用
`tau = peak * sin(commanded_roll - takeover_down_roll)`。因此，每次启动桥接器前，机器人
必须处于正常 DOWN 姿态，人体也保持当前标定的 DOWN；不能在手臂已经侧举时启动并把该姿态
错误地记录为重力零点。首轮软件上限为 3 N·m。

先在相同的 30° 安全包络和 1°/s 速度下依次比较 1、2、3 N·m。每档都执行：DOWN 保持
5 秒，缓慢到 Roll 约 +20°、+30°、+40°并各保持 5 秒，然后按相反顺序返回 DOWN。若某档
已经把高角度保持误差降到约 2°以内，不再继续增加；若误差变大、实际角超过命令、振荡或
出现异常声音，立即停止，并且不得进入下一档。

三个命令只改变下面这一项：

```bash
--roll-gravity-ff-nm 1.0
--roll-gravity-ff-nm 2.0
--roll-gravity-ff-nm 3.0
```

其余完整参数保持一致：

```bash
python3 humanoid_nmpc/remote_control/scripts/g1_arm_sdk_bridge.py \
  enx6c1ff701509c \
  --duration 105 \
  --alignment-rate-deg-s 1 \
  --max-rate-deg-s 1 \
  --max-excursion-roll-deg 30 \
  --max-body-speed-rad-s 0.30 \
  --max-tracking-error-deg 5 \
  --target-timeout 0.75 \
  --roll-gravity-ff-nm 1.0 \
  --absolute-alignment \
  --active-joints roll \
  --execute
```

终端会显示 `DOWN zero=... deg` 和当前 `ff=...Nm`；CSV 会记录
`command_roll_gravity_ff_nm` 与 `roll_gravity_zero_rad`。分档测试只允许一次改变前馈幅值，
动作、速度和保持时间必须相同，这样三份日志才能直接比较。
