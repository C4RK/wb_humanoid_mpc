# 电磁手臂重定向：仿真与真机验证顺序

这份清单把问题分成“传感器与逆解”“MPC 跟踪”“真机执行”三层。每一层单独通过后再连接下一层，出现误差时才能定位来源。

## 1. 重新标定并检查质量

当前 `wmet_calibration.yaml` 的静态参考残差为：down 31.5°、forward 27.1°、sideways 0°。这组数据不适合精度实验。新的标定程序会在最大残差超过 15° 时提示并默认不保存。

在容器中构建并加载代码：

```bash
cd /wb_humanoid_mpc_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select remote_control --symlink-install
source install/setup.bash
```

依次启动：

```bash
ros2 run remote_control em_tracker_node
ros2 run remote_control calibration_node
```

看到已有标定时输入 `n`。标定时尤其注意：

- 发射器在整个过程不能移动；接收器绑带不能松动。
- Step 0 肘关节始终锁直，肩部覆盖尽可能大的球面。
- Step 1 必须同时改变肩部方向和肘部角度。
- Step 2 上臂自然竖直向下。
- Step 3 上臂水平向前，肘弯 90°，前臂竖直向上，腕部保持中立。
- Step 4 上臂水平向左侧，肘伸直。
- 三个参考残差最好都小于 10°，必须小于 15° 才进入精度实验。

如果认真重复后残差仍超过 15°，先停止继续调 MPC。这通常表示接收器局部 `+Y` 轴并未准确沿着肘到腕方向，需要增加“接收器安装方向”标定，而不是继续修改关节增益。

## 2. 只验证重定向，不启动 MPC

启动：

```bash
ros2 run remote_control retargeting_node
ros2 topic hz /em/pose
ros2 topic hz /arm_joint_target
ros2 topic echo /arm_joint_target --once
```

按下列动作逐一保持 5 秒，并记录终端角度：

| 动作 | 主要观察量 | 合格现象 |
|---|---|---|
| 上臂向前、前臂向上（Step 3） | yaw、wrist | 两者接近 0° |
| 上臂不动，弯伸肘部 | elbow | elbow 连续变化，pitch/roll 基本不变 |
| 肘弯约 90°，只转动上臂内外旋 | yaw | yaw 连续变化，wrist 不应同幅变化 |
| 整条手臂姿态不动，只旋前/旋后 | wrist | wrist 连续变化，肩部三关节基本不变 |
| 肘逐渐伸直 | yaw | 接近伸直时 yaw 保持最后稳定值，不跳变 |

建议验收指标：静止 5 秒的关节标准差小于 2°；yaw/wrist 可放宽到 3°；孤立动作对其他关节的串扰小于 5°；话题频率稳定在设备额定频率附近。

记录数据：

```bash
ros2 bag record /em/pose /arm_joint_target
```

## 3. 单独验证 MPC，不接电磁设备

先启动仿真：

```bash
ros2 launch g1_centroidal_mpc mujoco_sim.launch.py
```

再发布一个固定的 22 维目标。下面只令左肘弯曲 5°：

```bash
ros2 topic pub -r 20 /arm_joint_target sensor_msgs/msg/JointState \
"{position: [-0.05, 0.0, 0.0, 0.1, -0.05, 0.0, -0.05, 0.0, 0.0, 0.1, -0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0873, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

这一层验证 MPC 能接收目标、MuJoCo 中关节方向正确、机器人保持双脚接触和站立平衡。先分别测试 pitch、roll、yaw、elbow、wrist 的小角度，再测试组合姿态。

## 4. 电磁设备到 MPC 的完整仿真

完整链路：

```text
/em/pose → retargeting_node → /arm_joint_target → centroidal MPC → MuJoCo
```

动作顺序使用“静态姿态 → 单关节慢动作 → 多关节慢动作 → 正常速度动作”。每次记录 `/em/pose`、`/arm_joint_target` 和仿真实际关节状态话题。主要指标为：

- 目标关节与仿真实际关节的 RMS 误差；
- 输入到实际运动的延迟；
- 静止抖动和最大关节速度；
- 双脚是否保持接触，质心和躯干是否出现不必要的大幅运动；
- MPC 是否持续收敛，是否出现求解超时或关节限位。

## 5. 真机任务门槛

真机按以下顺序推进，每一步都保留吊架和急停人员：

1. 只读低层状态，确认关节索引、符号、单位和更新率。
2. 不启用电磁设备，重复单关节 5° 固定动作。
3. 五个手臂关节分别做小角度动作，加入速度限制、超时回零和急停。
4. 双脚固定站立，给 MPC 缓慢的固定手臂目标，观察踝、髋、腰的平衡补偿。
5. 接入电磁目标，先静态姿态切换，再做连续慢动作。
6. 最后才提高动作范围和速度，并记录与仿真相同的指标。

对于当前课题，MPC 的价值是让手臂改变全身质心和角动量时，由腿、腰和接触力共同补偿。双脚固定的站立实验通常应表现为小幅踝、髋和躯干调整；不需要为了每次手臂动作踏步。只有固定双脚无法满足平衡约束时，才把踏步作为后续扩展。
