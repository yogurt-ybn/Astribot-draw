# Astribot T1 绘图 SDK

把规划器、厂家 SDK 和执行日志分开，支持连接、只读状态、轨迹执行、取消及退出清理。控制对象固定为躯干 4 + 左臂 7 + 右臂 7 个关节，单位 rad、rad/s、s。底盘固定；头、夹爪、轮子不在本接口范围。

实现依据仿真电脑上的 `astribot-dev-app` skill、`astribot knowledge` 接口文档及已安装的厂家 SDK。ROS 适配已验证 3.4.2、3.4.8，Link 私有缓存适配仍限 3.4.2；连接时检查版本与位置/速度命令签名。新版优先使用官方 `auto_acquire_control=False`，旧版抑制构造时的控制权申请，避免只读连接触发控制。

## 当前新电脑：simulation-deploy Docker 环境

代码部署在 `~/code/Astribot-draw`，上游 SVG 规划基线为 `04afcd6fa536dc9b31d658c1423ba2bfa2514b4a`。本目录提供 SDK 执行适配和当前姿态规划扩展。

仿真由已有的 `~/code/astribot-agent-meta/aos/astribot-e2e/simulation-deploy/start-simulation.sh` 启动。当前容器名 `astribot-simulation`，内部 SDK 3.4.8 / HAL 1.3.6 / Link 3.1.7 / NumPy 1.26.4 / MuJoCo 3.6.0，ROS domain 25。

```bash
cd ~/code/Astribot-draw
bash robot_sdk/run_sim.sh probe   # 自动进入已有容器，只读验证
bash robot_sdk/run_sim.sh demo    # 双肘 10°、两次往返，约 12 秒；不是绘图
```

不传参数只显示帮助。SVG 绘图使用下方的 `draw` 命令；`demo` 是单独的通信演示。

`--runtime auto` 优先使用正在运行的 `astribot-simulation` 容器；`--runtime docker --container NAME` 可指定容器，`--runtime local` 使用下方的本地源码环境。容器必须有部署脚本的 `io.astribot.simulation-only=true` 标记。启动命令 `bash robot_sdk/run_sim.sh start` 会调用已有 `start-simulation.sh`，可用 `--launcher PATH` 覆盖路径；已经运行时无需重新启动。

Docker 适配把本 SDK 临时复制进已有容器，使用容器内的 Python、ROS 消息和依赖，不安装或升级容器依赖、不重启仿真。Ctrl+C 会向容器内 SDK 进程请求取消；执行完成后报告复制回宿主机 `output/sdk-sim-<命令>/`，终端会打印实际宿主机日志目录。容器临时目录保留供诊断，重建容器后可直接再次运行宿主机脚本。

首次部署验证：SDK 的 28 项测试和上游 T1 的 5 项轻量数据回归均通过；只读反馈正常；SDK 仿真中 2 秒双肘命令完成。实际关节运动幅度约 0.02 rad，约 1.15°，仅用于通信核验。

容器模型 revision 为 `279e704ef9543c0e2154cc31aa4bce5b9f769b25`，上游参考模型依据 `cab872dc00081d2c5b096c7a19e563050eb7a2ff`。绘图适配直接使用容器当前资产重新构建规划模型、检查路径，并与独立的仿真 `t1_kinematic.xml` 比较末端 FK，不能仅凭旧报告判定当前仿真可执行。

## SVG → 双臂轨迹 → 官方仿真执行

仿真启动后，在宿主机运行：

```bash
cd ~/code/Astribot-draw
bash robot_sdk/run_sim.sh draw svg_planner/inputs/ginkgo.svg

# 换图只需指定另一张符合格式的 SVG：
bash robot_sdk/run_sim.sh draw /path/to/drawing.svg
# 仅规划，不申请控制权或发送动作：
bash robot_sdk/run_sim.sh draw /path/to/drawing.svg --plan-only
# 可降低执行速度，图形不缩放、不改线：
bash robot_sdk/run_sim.sh draw /path/to/drawing.svg --speed-scale 0.5
```

输入采用上游 [SVG_INPUT_INTERFACE.md](../svg_planner/SVG_INPUT_INTERFACE.md)：616×560 画布，两个直接子 `path`，分别 `id="left"` / `id="right"`，支持 `M L C Q Z`。机器人左臂对应画面右侧。普通编辑器导出的 groups、transforms、嵌入位图或多条分离子路径会被明确拒绝，不会静默改图。

`draw` 自动完成严格 SVG/画布检查、当前躯干/双臂/头/夹爪/底盘反馈读取、双臂耦合 IK、从当前姿态入场的 18 关节 RRT、同步绘制与返回、200→125 Hz 位置/速度重采样及每条命令的间隙检查，再通过 SDK 执行。用户不需要提前准备 NPZ。生成的 NPZ 仅为内部缓存与复核证据。

规划仍使用上游有限躯干候选族和确定性种子预算，不保证所有符合 SVG 格式的图案都能执行。搜索失败或间隙不足时保留拒绝原因。没有自动缩放、镜像、删线或跳到中立首帧。

原始 `04afcd6` 代码需要一次性应用 `robot_sdk/patches/svg-measured-start.patch`；当前分支和部署均已包含该适配，无需重复应用。它只给上游 `ik_candidate` 增加可选的实测起点，独立 `svg_plan.py` 默认共同中立起点和原基准不变。上游更新后可通过 `git apply --check robot_sdk/patches/svg-measured-start.patch` 检查是否仍适用。

规划时使用 MuJoCo 几何包围球保守剔除远离的碰撞对，候选对仍调用原生 `mj_geomDistance`，不改变原有 3947 对覆盖和阈值；已与原始穷举查询对比。该优化避免把大量时间用于明显相距很远的几何体。

执行前检查首帧；执行中检查控制权、反馈时效、关节跟踪，以及未控制的底盘、头、夹爪是否改变。底盘必须保持固定；夹爪需接近闭合，未观测的被动指节使用近闭合模型，不自动移动它们。实测关节姿态每 50 ms 做一次相同代理模型的碰撞检查，发现穿透则中止。

每次 `draw` 生成独立目录 `output/sdk-sim-draw/svg-<时间戳>/`，终端打印实际宿主机路径：

- `input.svg`、`input-preview.png`：原始 SVG 字节和解析后的目标。
- `svg-run.json`：各阶段、模型版本、当前起点、选中的 IK 分支、规划及执行结果。
- `sdk-plan.npz` / `.validation.json`：完整 q/dq 命令及其哈希绑定检查记录。
- `<执行ID>.jsonl`：目标、实测关节、辅助部件、阶段、反馈年龄和调度延迟。
- `measured-drawing.npz`、`measured-vs-svg.png`：从关节反馈计算的独立仿真模型 FK 与 SVG 目标对比；灰色为输入，蓝/橙为实测轨迹。

计划检查和执行分别有结果：SDK 命令执行完成后，还必须通过绘图阶段最大末端位置误差 ≤2 mm 的检查，才报告 `completed`。末端位置来自收到的关节反馈计算，不是相机或外部定位测量；固定底盘、工具偏置 `[0,-0.23,0]`、机械相邻排除等范围与上游一致。仍是空中光绘，不包含实体画笔接触或力控制。

规划和执行会打印进度。若仿真暂停或窗口停止更新，客户端先检查 ROS 状态流，反馈缺失时在构造 SDK 控制会话前返回失败；运行中反馈过期会中止。Ctrl+C 请求软件取消，日志保存在本次独立目录。

## 安装与离线验证

Python >= 3.10。普通 mock 只依赖 NumPy，不导入厂家 SDK，也不连接网络。

```bash
python -m pip install -e ./robot_sdk
python -m astribot_draw_sdk probe
python -m astribot_draw_sdk demo --output output/sdk-mock
```

`mock` 是一阶关节跟随器，用来检查协议、时序与取消流程；没有刚体动力学或碰撞。`demo` 从当前反馈开始，双肘相对起始姿态移动 10°，在峰值停留 1 秒，再返回；重复两次，共约 12 秒。使用五次平滑曲线，峰值速度约 0.164 rad/s，仍遵守 SDK 执行限速和关节限位。结果和逐帧 JSONL 存在输出目录。

已有厂家 SDK 的仿真环境使用其原有 NumPy，避免升级导致原生库 ABI 冲突：

```bash
python -m pip install --no-deps -e ./robot_sdk
```

`requirements.txt` 是通过 Astribot 依赖工具处理的部署依赖；`pyproject.toml` 声明通用兼容范围。每个进程只创建一个 SDK 会话，SDK 的环境配置具有进程作用域。

## 本地源码环境（不使用 Docker 时）

默认使用电脑上现成的 **`astribot_simulation`**，通过厂家 SDK 本地运行时和 ROS2 控制 T1。两端均加载官方 `env.sh --robot t1`，使用 `ASTRIBOT_COMMU_BACKEND=ros` 和相同的 `ROS_DOMAIN_ID`（默认 25）。

终端 1 启动官方仿真（已经运行则无需重复启动）：

```bash
bash robot_sdk/run_sim.sh start --runtime local --sim-dir /path/to/astribot_simulation
```

`start` 激活 `astribot_simulation` Conda 环境，加载 ROS2 Humble 和官方仿真 `env.sh`，然后在仿真目录按当前测试配置启动，保留碰撞：

```bash
export ASTRIBOT_EGL_CONTEXT_HANDOFF=1
python3 astribot_simulation.py astribot_t1 --sensors custom
```

默认寻找仓库旁的 `../astribot_simulation`，可用 `--sim-dir` 指定。若已手动用上述命令启动仿真，直接运行 `probe` 或 `demo`，无需再运行 `start`。`custom` 的传感器配置不改变本 SDK 使用的关节状态/指令接口；两端仍需相同 ROS 通道和 domain。

终端 2 运行控制脚本：

```bash
bash robot_sdk/run_sim.sh probe  # 只读检查
bash robot_sdk/run_sim.sh demo   # 显式：12 秒双肘 10° 往返两次
bash robot_sdk/run_sim.sh --python /usr/bin/python3.10
```

脚本可从任意目录调用，自动加载 SDK 源码，无需先 pip 安装。控制端默认使用 `/usr/bin/python3.10`，可通过 `--python` 或 `PYTHON` 指定；`--mock` 执行离线测试，`--output` 指定日志目录。

当前电脑的厂家 SDK 装在系统 Python 用户目录，仿真 Conda 环境通过 `PYTHONNOUSERSITE=1` 隐藏该目录，因此不能直接把控制解释器换成仿真 Conda Python。脚本为系统 SDK Python 恢复用户目录，两端 ROS 环境仍保持一致。

`probe` 只读，不申请控制权、不发运动命令。`demo` 会申请本地 SDK 租约并发送运动命令。ROS 反馈必须带 `frame_id=simulation`，时效按最旧部件的实际接收时间检查。HAL 的空关节名数组按已核对的官方 T1 配置顺序解释。ROS 本地租约没有跨进程仲裁，因此同一 domain 内应只运行一个运动控制脚本。

已同步到当前仿真电脑 `/home/astribot/astribot_ws/Astribot-draw/robot_sdk`，无需安装也可以在该仓库根目录运行：

```bash
bash robot_sdk/run_sim.sh probe
```

显式 Link 兼容路径供已有 SDK Link 服务的仿真使用：`bash robot_sdk/run_sim.sh probe --transport link --peers tcp/127.0.0.1:7447`。两端必须使用相同通道；本机 Link 地址不能证明隔离，router 可能转发到真机。它不是当前官方 ROS 仿真的默认入口。

`sdk-hardware` 共用适配器，默认禁止运动；CLI 仅允许只读 probe。真实硬件接入需要在 Python 调用方显式启用策略并完成实际环境验证。软件 `cancel()` / Ctrl+C 不能代替物理急停。

## 对接 GitHub 更新的 T1 规划器

支持已审阅的 Astribot-draw `136b93f` 的 `t1_sync.plan()` / `t1_wings.plan()`：必须从规划函数直接取得解析 `qs`、`dqs`，不能拿 10 Hz 回放记录的实测 qpos/qvel 充当命令。旧 `plan.npz` 丢失了 dqs，需要重新导出。

在有 MuJoCo、SciPy 和 T1 模型资产的规划环境运行：

```bash
python robot_sdk/tools/export_t1.py --t1-dir /path/to/Astribot-draw/t1 --assets /path/to/aos/descriptions --design waves --output output/t1-waves
```

工具按模型关节名映射到 18 关节，补齐最终 home 的零速度帧，把原生 200 Hz 的 q/dq 用三次 Hermite 插值到 125 Hz，对每个重采样姿态重新检查原规划器的全套碰撞对（2 mm 阈值），输出：

- `sdk-plan.npz`：时间轴、命令位置/速度、关节名、分组、阶段、适用范围。
- `sdk-plan.validation.json`：绑定上述文件 SHA256、几何检查结果与模型来源。
- `t1-collision-model.json`：检查覆盖与明确排除的机械相邻碰撞对。

这是离散姿态几何检查，不能证明连续路径、被排除关节区域或真实硬件安全；仿真动力学与实际跟踪效果由下一步 SDK 执行验证。该工具不连接机器人。重采样检查可能耗时数分钟。

把两个 `sdk-plan` 文件一起复制到仿真 SDK 环境后执行：

```bash
bash robot_sdk/run_sim.sh run output/t1-waves/sdk-plan.npz --output output/t1-waves-execution
```

执行前将仿真初始化到轨迹首帧，或由规划器从反馈姿态生成经过检查的接近段。默认首帧误差大于 0.03 rad 会拒绝并报告偏差最大的关节；执行器不会自动插入接近动作。相机/世界坐标旋转属于规划模型，关节命令不再额外旋转。HDF5 全身回放需要独立的全身/WBC 契约，不能塞进本 18 关节接口。

## Python 接口

```python
from astribot_draw_sdk import RobotSession, Trajectory

plan = Trajectory.from_npz('output/t1-waves/sdk-plan.npz')
with RobotSession('sdk-sim', transport='ros',
                  log_dir='output/t1-waves-execution') as robot:
    print(robot.state())
    execution = robot.execute(plan)  # 立即返回，厂家调用留在同一个工作线程
    # execution.cancel()            # 从其他线程请求取消
    result = execution.wait(timeout=float(plan.time[-1]) + 10)
    print(result)
```

`Trajectory` 支持不规则原始采样率，位置与解析速度始终同步；文件执行需对应的哈希验证报告。`export_model_trajectory()` 可接其他规划器，参数和模型映射见源码。

执行器使用单调时钟按经过时间取样，默认 125 Hz，下发 `control_way='direct'`、`use_wbc=False`、`add_default_torso=False` 和成对位置/速度。厂家 SDK 自身的 MotionLimiter 仍然存在，必须通过反馈日志验证它对跟踪的影响。Python 调度不是硬实时。

默认检查：速度 <= 0.3 rad/s、单步 <= 0.03 rad、反馈年龄 <= 250 ms、调度延迟 <= 50 ms、跟踪误差 <= 0.15 rad；同时遵守厂家关节限位。不裁剪越界命令，不突发补发过期帧。Link 服务端检查已有控制者；ROS 本地租约不具备跨进程抢占检查。结束后确认反馈收敛，取消/故障时只在仍持有控制权且反馈新鲜时下发当前姿态零速度保持，再释放自己的控制权。传输断开时不能保证保持命令送达，日志会记录失败。

JSONL 包含目标 q/dq、实测 q/dq、阶段、反馈序号/年龄和调度延迟。汇总报告区分 `mock_protocol` 与 `sdk-sim`；离线 mock 通过不能作为仿真动力学通过。

## 验证

```bash
PYTHONPATH=robot_sdk/src python -m unittest discover -s robot_sdk/tests -v
```

测试覆盖命名映射、200→125 Hz 位置速度插值、导出检查与文件哈希、厂家下发参数、线程归属、首帧拒绝、辅助部件变化、反馈过期、控制权丢失、取消和资源释放，并对碰撞包围球剔除与原生穷举查询进行对比。

当前 SVG 适配在仿真容器 Python 3.10 中通过 35 项 SDK 测试和 25 项上游 SVG 测试。银杏 SVG 已完成完整 SDK 执行：约 218.8 秒、27352 条命令、12994 个绘图反馈样本；左右末端最大误差约 1.097 / 1.104 mm，RMS 约 0.384 / 0.386 mm，实测姿态最小采样间隙约 3.981 mm。几何与误差范围见上方说明。

参见 [精简验证记录](examples/svg-ginkgo-verification.json) 和 [实际反馈轨迹对比](examples/ginkgo-measured-vs-svg.png)。完整命令与反馈日志保存在执行电脑本次输出目录，未随代码发布；模型资产不包含在本包中。
