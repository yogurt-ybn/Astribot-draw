# 固定底盘、躯干与双臂同步光绘

## 本次完成

三组原始图案均保持不变：波浪、圆滑叶片、双旋涡。左右手使用同一归一化进度和五次时间律，同步开始/结束绘制；接近、绘制、收回均同时驱动双臂，并使用一条共享躯干轨迹。底盘固定，人体未加入本次碰撞模型。

躯干并非伸缩升降杆，而是四个受控旋转关节。前三关节可使上身升降和前后移动，第四关节为偏航。本次 J1=0.05 rad，J2 随共同目标高度变化，J3=-J1-J2 保持上身近似竖直，J4=0。选用的后倾参数为波浪0.65、叶片0.85、旋涡0.75。双手逆解根据同一躯干位姿补偿，从而保持固定世界绘制平面的图案。

这是**姿态引导的共享躯干轨迹与双臂协调控制**，不是通用全身最优控制器。头部/夹爪保持既定姿态，底盘不动。初始化使用已说明的收臂躯干姿态；初始化后执行通过 MuJoCo 执行器积分，不修改 qpos 瞬移。录像设置 qpos 仅用于重放。

## 碰撞模型

原始视觉网格的碰撞大多关闭。本版使用各部件原始网格的 MuJoCo 凸包，配合原夹爪垫块及光棒胶囊，建立 **1,071 个显式活动碰撞对**，带真实接触响应：
- 双臂之间（包括手和光棒）：324 对。
- 同一臂非邻接部件：160 对。
- 机械臂/工具与躯干、头部、底座等：496 对。
- 非邻接躯干/头部/底座：55 对。
- 臂/工具与地面：36 对。

自然机械连接部位、直接邻接关节及部分复合关节网格有明确排除，详见 collision_model_report.json 中的 excluded_pairs 和说明。**肘部 link3/link5 保持检测**，没有为了通过轨迹删除它。凸包会填充真实网格凹腔，因此模型重叠不等于已证实实物碰撞；本次选择绕开这些保守凸包重叠的姿态。

在本模型与当前 MuJoCo 3.4 配置的复现实验中，native CCD 曾对明显分离的特定网格对返回假零距离，因此本版显式 nativeccd=disable，使用 libccd。正向分离测试以及四类故意碰撞负例验证了距离/接触响应，见 collision-negative-controls.json。零接触本身不是通过依据。

规划全流程每50ms检查全部碰撞对，要求至少2mm几何余量；实际积分每10ms检查全部碰撞对，模型接触margin为1mm。大于等于50mm的距离记为50mm截断值，并非精确最短距离。独立检查文件如存在，提供最小时刻周围更密采样结果。**离散采样不是连续扫掠的数学保证**。

## 结果

- 波浪：115.27s，最小模型间距4.393mm，单臂RMS约0.307mm，上躯干高度变化15.6cm。
- 圆滑叶片：112.68s，最小间距2.083mm，RMS约0.369mm，高度变化22.5cm。
- 双旋涡：180.55s，最小间距2.277mm，RMS约0.192mm，高度变化18.9cm。

高度参考是 astribot_torso_link_4 的世界z，不是地面到头顶。底盘位置变化为0；臂和躯干关节全程保持正位置限位余量。有限状态、无仿真警告、无接触；详细最小间距的部件/时刻/阶段见每组 results.json。

这些2–4mm间距非常薄，不能照搬至实机。凸包几何、机械邻接排除、有限采样、理想动力学及未验证的真实速度/加速度上限都限制结论。结果仅支持这些具体仿真轨迹，**不是完整实机碰撞安全认证**。也没有重算躯干运动后的最大画布；原570×471掩膜仍是原中立躯干平面的设计子集。

## 复现

使用Python3.12，安装 requirements.txt。下载固定版 Astribot 描述：

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/Astribot-Dev/astribot_descriptions.git descriptions
git -C descriptions checkout 8c651a1abbe2821824a692aa3a3aa4cbf88e4122
.venv/bin/python download_assets.py
ASTRIBOT_ASSETS="$PWD/descriptions" OPENBLAS_NUM_THREADS=1 .venv/bin/python sync.py tidal_waves
ASTRIBOT_ASSETS="$PWD/descriptions" OPENBLAS_NUM_THREADS=1 .venv/bin/python sync.py rounded_leaves
ASTRIBOT_ASSETS="$PWD/descriptions" OPENBLAS_NUM_THREADS=1 .venv/bin/python sync.py twin_galaxies
.venv/bin/python render_sync.py
```

模型资源不打包，遵循所附上游许可证。每次模型编译较耗内存，建议逐组运行，不并行编译。渲染还需ffmpeg、EGL/OpenGL、DejaVu/Noto字体。

inputs 含原始有序图案JSON、原中立躯干的IK种子及网格，仅用于初始化候选。sync.py 会重新求解当前躯干下的轨迹并重新做碰撞验证，不直接把旧关节序列同时播放。完整100Hz execution.npz 可由脚本生成；轻量包内 execution-10hz.npz 是抽样结果，不能冒充完整检查数据。

## 独立复核

independent-final-audit.json 重新计算了跟踪误差、18个受控躯干/机械臂关节余量、原图案的折线一致性，并对每组最小间距时刻前后50ms做1ms关节插值复核，最小间距仍为4.393/2.083/2.277mm。目标曲线相对原JSON折线偏差均小于0.019mm。该局部加密不等于全程连续扫掠证明。

最终适配器的v2输入解析与py_compile已复核，三组坐标转换后的世界点与已执行目标逐元素完全相同（final-adapter-check.json）。此次输入适配修订未重跑完整动力学；已有动力学记录仍对应相同点列、规划方法和参数。
