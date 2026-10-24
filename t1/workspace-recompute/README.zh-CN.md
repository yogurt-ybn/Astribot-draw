# Astribot T1：固定平面采样可达画布

这是针对官方 Astribot T1 模型重新计算的画布，不是缩放 S1 的旧画布。用于生成图案的模板是 `results-final/blank-transparent.png` 或 `blank-white.png`，原尺寸 **616 × 560 像素**。每像素 2.5 mm，对应 **1.540 × 1.400 m**。绘制平面世界 Y = -0.42 m；光棒发光端在官方 arm tool site 沿 link7 局部 -Y 延伸 80 mm，位置为 link7 [0,-0.23,0] m。

## 坐标和分区

连续图像边缘坐标，u 向右、v 向下；像素中心为整数 + 0.5。

- x = -0.77 + 0.0025 u
- y = -0.42
- z = 1.89 - 0.0025 v

不得再使用 S1 的 570 × 471 尺寸或其像素映射。本图不受旧摄影相机画幅裁切；若仍用旧相机，必须重新设置视场／机位／投影，不能把本图当作旧相机画面。

`marginal-mask-left.png` / `marginal-mask-right.png` 是每臂分别找到姿态见证的区域。`mask-left.png` / `mask-right.png` 进一步按世界 x=0 分配设计区域：机器人左臂在画面右侧，右臂在左侧。这样分配不证明两只手可以任意同步绘制。紫色仅代表两臂**分别**找到过见证，不代表同时可达。

## 计算范围与验收

- 底盘结构固定，不考虑人体。头部、夹爪采用记录中的既定状态。
- 在世界 X/Z 平面使用 20 mm 网格；图像每个 8×8 像素色块代表一个采样点的可视化支撑单元，不是块内每个像素都已求解。
- 使用 11 个明确列出的躯干姿态，包括前后屈伸、近似竖直下蹲和 ±0.4 rad 转向；未穷尽四维躯干连续空间。姿态表见 calibration.json。
- 每个采样点通过多初值、最近 FK 种子逆解搜索。活动臂关节限位余量 ≥0.04 rad，发光端位置误差 <0.5 mm，末端朝向不受约束。
- 每个接受点有完整机器人关节姿态，另一只机械臂停在该躯干姿态下已检查的停放姿态。3,947 个规定的几何对筛查要求间距 ≥2 mm。
- 当前找到左臂 3,409、右臂 3,406 个点；姿态 0 单独找到左 2,248、右 2,240 个点。新增区域来自所采样躯干姿态，并不代表机器人全部最大能力。
- 重新验证了全部 6,815 个点的实际 MuJoCo 正运动学及关节余量；每臂均匀选取 160 个见证再检查完整碰撞对，均通过。全部接受点生成时都经过碰撞筛查。pixel→world→pixel 数值一致性检查通过。

## 不能据此推出的结论

1. 未找到逆解的灰色区域不是严格的不可达证明，可能受网格、种子和姿态采样限制。
2. 这是对躯干姿态取并集的**单臂边缘可达区域**。左手和右手两个点的见证可能要求不同的躯干姿态。每个同步图案仍需共同躯干、双臂连续逆解和完整运动检查。
3. 没有在这里验证接近/绘制/收回的动态过程。路径连续性、速度、轨迹限位、动态跟踪均由执行后端检验。
4. 机械连接排除项仍适用，尤其官方 T1 代理网格在复合肘关节 3/5、前臂 5/夹爪基座接口的固有重叠。保留手指/垫块/光棒等其他检查，但不能声称被排除部位的折叠碰撞已全面解决。
5. 凸包代理、有限采样与很薄的模型间距不构成实机安全认证。

## 文件

- `t1-canvas-overview-zh.png`：中文范围说明图。
- `drawing-regions.png`：设计分臂底图；`marginal-regions.png`：边缘区域与重叠。
- `mask-*.png` / `marginal-mask-*.png`：对应二值掩膜。
- `blank-transparent.png` / `blank-white.png`：原尺寸空白模板。
- `calibration.json`：尺寸、映射、姿态与范围。
- `workspace-witnesses.npz`：所有接受点的世界坐标、完整 qpos、躯干索引、残差和碰撞距离记录。
- `workspace-summary.json`、`independent-checks.json`：计算与复核结果。
- `torso-sampling-extension.png`：蓝色为姿态 0，绿色为额外躯干姿态找到的扩展点。

## 重新计算

本目录需配合当前 T1 迁移代码中的 `t1_model.py` / `kinematics.py`，并通过官方固定版本下载模型资源。不要重新分发上游模型网格；本包不包含网格或编译模型。

设置 `T1_DRAWING_ROOT` 指向含 `t1_model.py` 的目录、`ASTRIBOT_T1_ASSETS` 指向下载的官方 descriptions 目录，然后在已安装 MuJoCo/NumPy/SciPy/Pillow 的 Python 环境运行：

```bash
OPENBLAS_NUM_THREADS=1 python compute_canvas.py
python render_canvas.py
OPENBLAS_NUM_THREADS=1 python verify_canvas.py
```

计算时会在 results-final/model 生成本地编译模型，用于复核；它不包含在轻量交付包中。读取已有结果不需要重新计算。官方资源固定于 aos 分支 cab872dc00081d2c5b096c7a19e563050eb7a2ff。
