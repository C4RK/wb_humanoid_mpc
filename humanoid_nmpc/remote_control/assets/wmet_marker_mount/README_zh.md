# WEMT 接收器与 ArUco 标记一体固定架

本目录包含适配当前实测接收器外壳的参数化固定架：

- 接收器外壳：`23 × 13 × 70 mm`；
- 圆点开关：约直径 `2 mm`、突出 `1 mm`；
- 打印后的 ArUco 纸张：`77.5 × 77.5 mm`；
- 魔术绑带：宽 `25 mm`。

## 文件用途

- `wmet_receiver_fit_coupon.step/.stl`：20 mm 长的卡槽截面试装件；
- `wmet_aruco_arm_mount.step/.stl`：完整固定架；
- `parameters.json`：全部尺寸、派生尺寸与打印参数；
- `generate_wmet_marker_mount.py`：参数化 CadQuery 源文件，位于 `scripts/`。
- `preview_01` 至 `preview_05`：无需三维软件即可查看的多角度 PNG 预览图。

STEP 可导入 SolidWorks 或 FreeCAD；STL 可以直接导入切片软件。SolidWorks 导入 STEP 后得到实体，
但不会自动恢复 CadQuery 的完整特征历史。

## 必须先打印试装件

先用 PLA 打印 `wmet_receiver_fit_coupon.stl`，并分别试装两个准备用于大臂和前臂的接收器。

合格标准：

1. 接收器能从 70 mm 长轴方向平稳滑入；
2. 不需要强行挤压外壳；
3. 横向没有明显晃动；
4. 滑动过程不会按下圆点开关；
5. 倒置并轻轻晃动时，试件不会明显松旷。

如果太紧，重新生成时增加单侧间隙，例如：

```bash
python3 humanoid_nmpc/remote_control/scripts/generate_wmet_marker_mount.py \
  --side-clearance 0.50 \
  --thickness-clearance 0.55
```

如果明显松动，将对应间隙每次减小 `0.10–0.20 mm`。不要直接缩放 STL，因为缩放会同时改变
标记板和 25 mm 绑带槽。

## 完整件打印建议

- 材料：正式件优先 PETG，试装件可以使用 PLA；
- 层高：`0.20 mm`；
- 外壁：至少 3 层；
- 填充：`25–35%`；
- 打印方向：77.5 mm 标记所在的平面朝下，平放在热床；
- 支撑：设计目标为不需要支撑；
- 数量：卡槽确认合适后打印两件相同固定架。

打印机的首层象脚可能使滑入口变紧。如果只有入口最初几毫米偏紧，先清理毛边，不要立即整体放大。

## 组装

1. 将接收器开机；
2. 让带开关的 `23 × 70 mm` 面朝向标记板背面；
3. 让充电口一端从固定架的 `-Y` 远端滑入；
4. 滑到近端挡块后，在远端对齐孔中穿入一段约 `34–36 mm` 长的 `1.75 mm` 打印耗材，
   作为非金属保险销；
5. 在两条身体支撑轨顶部粘贴约 `3 mm` 厚 EVA 防滑泡棉；
6. 两条 25 mm 魔术绑带分别穿过近端和远端的左右绑带槽；
7. 将 `77.5 × 77.5 mm` 标记纸居中粘到完全平整的正面。

保险销只用于防止接收器从滑槽退出。可以在销的外端贴一小片胶带防止自行滑出；不要用金属
螺钉、磁铁或金属卡扣代替。

## 佩戴方向

固定架背面的凸起箭头指向 `+Y` 近端：

- ID10 大臂：箭头朝肩膀，充电口端朝肘部；
- ID20 前臂：箭头朝肘部，充电口端朝手腕。

组件安装在左臂外侧偏前的位置，使标记面大致朝向两台手机之间。使用两条绑带防止旋转，但不要
勒到手臂麻木或影响血液循环。

## 重新生成

生成脚本依赖 CadQuery。建议在独立 CAD 环境中安装，避免影响 ROS 环境：

```bash
python3 -m pip install cadquery
python3 humanoid_nmpc/remote_control/scripts/generate_wmet_marker_mount.py
```

单位全部为毫米。任何一次取出、重新插入或重新佩戴后，在正式采集前都应重新检查接收器与标记
是否固定，并重新完成当前实验所需的对齐/标定。

如需重新生成 PNG 预览图：

```bash
python3 humanoid_nmpc/remote_control/scripts/generate_wmet_mount_previews.py
```
