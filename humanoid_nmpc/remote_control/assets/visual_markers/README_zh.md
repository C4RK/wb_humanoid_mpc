# 双手机视觉验证打印件

本目录中的打印件统一使用 OpenCV `DICT_4X4_50`。

## 文件

- `charuco_A4_left.pdf`：320×240 mm ChArUco 板的左半部分；
- `charuco_A4_right.pdf`：ChArUco 板的右半部分；
- `aruco_ID10_ID20_A4.pdf`：ID 10（大臂）和 ID 20（前臂）小板；
- `charuco_8x6_320x240mm_full_preview.png`：完整图案预览，不用于普通 A4 打印；
- `aruco_id10_60mm.png`、`aruco_id20_60mm.png`：单个图案备份。

## 打印设置

三个 PDF 都按 A4 物理页面生成。打印时必须选择：

```text
A4
纵向
实际尺寸 / 100%
关闭“适应页面”与“缩小超大页面”
```

先只打印一份，并用尺子检查：

- ChArUco 每个方格：40.0 mm；
- 小标记黑色外框：60.0 mm；
- 小标记裁切区域：80×80 mm。

若误差超过约 0.5 mm，先修改打印设置，不要靠修改软件中的尺寸补偿。

## 拼接大板

1. `left` 与 `right` 各打印一张；
2. 左页只沿图案的右边缘精确裁切；
3. 右页只沿图案的左边缘精确裁切；
4. 把两条裁切边无缝对接，拼缝位于第 4、5 列方格之间；
5. 从背面用胶带固定，正面不要用反光胶带覆盖图案；
6. 整体粘到平整的 A3 泡沫板或硬纸板；
7. 完整棋盘图案应为 320×240 mm，不能有可见缝隙、重叠或翘曲。

拼接后用尺子检查跨越拼缝的两个相邻方格，总宽应为 80.0 mm。拼接误差会直接进入双相机
外参误差；如果接缝明显，重新裁切和拼接。

## 裁切小板

ID 10 和 ID 20 可以从同一张 A4 中分别裁下。每块保留完整的 80×80 mm 白色底板，黑色
标记必须位于中央，四周约有 10 mm 白边。将纸张平整粘到轻质非金属硬板上，并在背面标记：

```text
ID 10：UPPER ARM / 大臂
ID 20：FOREARM / 前臂
```

不要覆膜，不要使用金属底板，不要把文字或胶带贴到标记正面。

## 重新生成

```bash
conda activate unitree-rl
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc

python3 humanoid_nmpc/remote_control/scripts/generate_visual_marker_prints.py
```
