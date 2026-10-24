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
# 按末端速度规划；耗时由路径长度、起停和关节约束决定：
bash robot_sdk/run_sim.sh draw /path/to/drawing.svg --joint-speed 1.0 --tip-speed 0.2
# 更慢，图形不缩放、不改线：
bash robot_sdk/run_sim.sh draw /path/to/drawing.svg --tip-speed 0.1
```

输入采用上游 [SVG_INPUT_INTERFACE.md](../svg_planner/SVG_INPUT_INTERFACE.md)：616×560 画布，两个直接子 `path`，分别 `id="left"` / `id="right"`，支持 `M L C Q Z`。机器人左臂对应画面右侧。普通编辑器导出的 groups、transforms、嵌入位图或多条分离子路径会被明确拒绝，不会静默改图。

`draw` 自动完成严格 SVG/画布检查、当前躯干/双臂/头/夹爪/底盘反馈读取、双臂耦合 IK、从当前姿态入场的 18 关节 RRT、同步绘制并停在绘制末端、200→125 Hz 位置/速度重采样及每条最终命令的间隙/形状检查，再通过 SDK 执行。用户不需要提前准备 NPZ。生成的 NPZ 仅为内部缓存与复核证据。

规划仍使用上游有限躯干候选族和确定性种子预算，不保证所有符合 SVG 格式的图案都能执行。搜索失败或间隙不足时保留拒绝原因。没有自动缩放、镜像、删线或跳到中立首帧。

SDK 绘制在共同归一化弧长进度上等间距求 IK，较长臂的采样弧长步长不超过 2.5 mm；自动加密整个均匀网格，使重采样折线相对输入解析折线的偏差界不超过 0.1 mm。这样避免左右不对称路径合并出极近节点后，关节样条二阶导数被放大、加速度计时持续降速。原速度、加速度预算及最终指令检查保持有效；`svg-run.json` 的 `target_sampling` 记录本次采样点数与偏差界。独立 SVG 规划器默认仍保留原始顶点采样。

五组新 SVG 在官方仿真全部完成运动，规划器 34 项与 SDK 44 项测试通过。不对称螺旋实际完整运动由 71.12 秒降到 11.67 秒，最大反馈末端误差 1.84 mm；同一记录起点的完整计划对照为 71.11→11.96 秒。S 曲线与展翼峰值误差 2.04/2.54 mm，超过原 2 mm 审计线，原始失败结果保留。实际顺序运行的起点不同；同起点对照为无运动的离线复验。见 [可复现 SVG 与精简验证记录](examples/generalization/README.md)。

原始 `04afcd6` 代码需要一次性应用 `robot_sdk/patches/svg-measured-start.patch`；当前分支和部署均已包含该适配，无需重复应用。它给上游 `ik_candidate` 增加可选的实测起点，并给 `schedule` 增加 `return_home` 和 `timing` 参数。SDK 的 `draw` 使用 `return_home=False, timing="feedrate"`：末帧速度为零，确认反馈收敛后结束，不自动返回。独立 `svg_plan.py` 仍默认返回，原基准不变。上游更新后可通过 `git apply --check robot_sdk/patches/svg-measured-start.patch` 检查是否仍适用。

规划时使用 MuJoCo 几何包围球保守剔除远离的碰撞对，候选对仍调用原生 `mj_geomDistance`，不改变原有 3947 对覆盖和阈值；已与原始穷举查询对比。该优化避免把大量时间用于明显相距很远的几何体。

执行前检查首帧；执行中检查控制权、反馈时效、关节跟踪，以及未控制的底盘、头、夹爪是否改变。底盘必须保持固定；夹爪需接近闭合，未观测的被动指节使用近闭合模型，不自动移动它们。实测关节姿态每 50 ms 做一次相同代理模型的碰撞检查，发现穿透则中止。

仿真调试可给 `draw` 加 `--skip-gripper-checks`：跳过夹爪反馈完整性、闭合位置与漂移检查，手指几何使用固定模型预设。该选项只允许官方仿真 ROS 绘制，不发送夹爪指令；其他保护及绘制精度审计保持有效。此模式的碰撞结果不代表实际手指开度或夹持荧光棒的几何验证。

每次 `draw` 生成独立目录 `output/sdk-sim-draw/svg-<时间戳>/`，终端打印实际宿主机路径：

- `input.svg`、`input-preview.png`：原始 SVG 字节和解析后的目标。
- `svg-run.json`：各阶段、模型版本、当前起点、选中的 IK 分支、规划及执行结果；`timings_s` 记录 SVG 检查、模型/状态准备、IK、时间规划、原始轨迹检查、重采样检查、执行和反馈核验耗时。`input_to_trajectory_ready` 从 CLI 开始准备 SDK 到通过全部轨迹检查，包含初始化，不含 shell 文件复制。
- `sdk-plan.npz` / `.validation.json`：完整 q/dq 命令及其哈希绑定检查记录。
- `<执行ID>.jsonl`：目标、实测关节、辅助部件、阶段、反馈年龄和调度延迟。
- `measured-drawing.npz`、`measured-vs-svg.png`：从关节反馈计算的独立仿真模型 FK 与 SVG 目标对比；灰色为输入，蓝/橙为实测轨迹。

计划检查和执行分别有结果：绘图阶段末端位置误差只作为质量指标记录，2 mm 是提示参考线。超过参考线时打印 `SVG_ACCURACY_WARNING`，保存峰值、RMS 与 `measured_drawing.status="warning"`，正常完成的运动仍报告 `completed`、CLI 返回 0，批量脚本继续执行。此项审计发生在运动结束后，不因绘制精度偏差判定任务失败；控制权、反馈时效、明显关节跟踪异常、限位与碰撞保护保留。末端位置来自收到的关节反馈计算，不是相机或外部定位测量；固定底盘、工具偏置 `[0,-0.23,0]`、机械相邻排除等范围与上游一致。仍是空中光绘，不包含实体画笔接触或力控制。下面的历史验证记录保留当时的严格 2 mm 判定，不按新策略改写。

此策略在官方容器通过 45 项 SDK 测试，并离线复验之前 S 曲线、展翼的原始反馈日志：2.042/2.543 mm 偏差均保留原值，审计正常返回并记录 warning。见 [精度提示策略验证](examples/generalization/accuracy-policy-verification.json)；本次日志复验未发送新的运动。

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

## 选择已保存的图案轨迹（0–57，官方仿真）

```bash
bash robot_sdk/run_pattern.sh --list
bash robot_sdk/run_pattern.sh       # 提示输入 0–57
bash robot_sdk/run_pattern.sh 0     # 爱心
bash robot_sdk/run_pattern.sh 57    # 音符与星光
bash robot_sdk/run_pattern.sh 0 --plan-only
```

脚本使用图案库的固定连续编号 0–57。`--list` 显示准确对应关系。需要已启动的官方 `astribot-simulation` 容器，以及 `pattern_library/v1/pattern_lookup.json` 和三个对应文件：`source.svg`、`sdk-plan.npz`、`sdk-plan.validation.json`。

绘制段固定在原画布世界坐标，不绑定上一次测试的机器人起点。脚本丢弃缓存中的旧进场，读取当前关节反馈，为共享躯干和双臂重新检查/规划到精确绘制起点的进场，然后复用缓存的躯干与双臂绘制曲线；不重新求整段绘制 IK、不移动或缩放 SVG。查询文件的 `canvas_drawing_entries` 保存由原优化计划提取的零速度绘制起点和时间，并绑定原命令轨迹 SHA256。

保持 125 Hz、关节上限 1 rad/s、末端上限 0.2 m/s，以及当前仿真使用的固定夹爪几何。最终每条命令仍检查几何、源 SVG 形状和速度，执行时保留反馈与实际姿态保护；检查失败时不发送该次绘制运动。正常绘制结束停在末端。模型 revision、画布标定、头部/底盘位置需与缓存几何相符。`--output DIRECTORY` 指定报告目录；默认保存在 `output/pattern-编号-时间/`。Ctrl+C 取消当前任务。

已在官方仿真验证：48 项 SDK 测试、34 项规划器测试及全部 58 个编号的新进场与缓存绘制衔接检查通过。0 和 57 号已分别完成实际运动，约 15.45 秒与 21.72 秒；结束停在绘制末端。

## Python 接口（通用轨迹）

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

执行器使用单调时钟按经过时间取样，默认 125 Hz，下发 `control_way='direct'`、`use_wbc=False`、`add_default_torso=False` 和成对位置/速度。当前 SDK 3.4.8 文档说明成对 q/dq 直接转发，不做 SDK 位置滤波/差分；驱动、仿真动力学和反馈链路仍有跟踪时延。Python 调度不是硬实时。

通用执行策略默认速度 <= 0.3 rad/s；SVG 的 CLI 按 `--joint-speed` 设置应用限速（默认 1 rad/s），并与厂家实际速度限值取较低者。共同检查：单步 <= 0.03 rad、反馈年龄 <= 250 ms、调度延迟 <= 50 ms、跟踪误差 <= 0.15 rad；同时遵守厂家关节限位。不裁剪越界命令，不突发补发过期帧。Link 服务端检查已有控制者；ROS 本地租约不具备跨进程抢占检查。结束后确认反馈收敛，取消/故障时只在仍持有控制权且反馈新鲜时下发当前姿态零速度保持，再释放自己的控制权。传输断开时不能保证保持命令送达，日志会记录失败。

JSONL 包含目标 q/dq、实测 q/dq、阶段、反馈序号/年龄和调度延迟。汇总报告区分 `mock_protocol` 与 `sdk-sim`；离线 mock 通过不能作为仿真动力学通过。

## 验证

```bash
PYTHONPATH=robot_sdk/src python -m unittest discover -s robot_sdk/tests -v
```

测试覆盖命名映射、200→125 Hz 位置速度插值、导出检查与文件哈希、厂家下发参数、线程归属、首帧拒绝、辅助部件变化、反馈过期、控制权丢失、取消和资源释放，并对碰撞包围球剔除与原生穷举查询进行对比。

首轮含返回的 SVG 适配在仿真容器 Python 3.10 中通过 35 项 SDK 测试和 25 项上游 SVG 测试。该轮银杏 SVG 已完成完整 SDK 执行：约 218.8 秒、27352 条命令、12994 个绘图反馈样本；左右末端最大误差约 1.097 / 1.104 mm，RMS 约 0.384 / 0.386 mm，实测姿态最小采样间隙约 3.981 mm。几何与误差范围见上方说明。

参见 [精简验证记录](examples/svg-ginkgo-verification.json) 和 [实际反馈轨迹对比](examples/ginkgo-measured-vs-svg.png)。完整命令与反馈日志保存在执行电脑本次输出目录，未随代码发布；模型资产不包含在本包中。

新的不返回版本已在同一官方仿真中验证：35 项 SDK 和 26 项 SVG 测试通过。银杏输入到完整轨迹检查通过约 27.04 秒（IK 4.03 秒、时间规划 0.10 秒、原始轨迹检查 5.06 秒、重采样检查 13.22 秒，另含初始化/模型准备）；运动约 109.41 秒，输入到结果约 138.22 秒。绘制最大反馈 FK 误差约 1.108 mm。结束约 10.95 秒后只读反馈仍停在末端，零速度，无返回阶段。计时不含 shell 文件复制和最终 SDK 关闭，耗时随输入与机器负载变化。见 [不返回验证记录](examples/svg-ginkgo-stop-at-end-verification.json)。

速度规划使用每个样条区间的解析关节导数上界，乘以同一区间的五次平滑进度速度上界，避免把不同时刻的两个全局最大值相乘。独立规划器基准仍为关节 0.2 rad/s、末端 0.05 m/s；SDK 绘图可显式传入速度上限，默认关节 1 rad/s、末端 0.2 m/s，125 Hz SDK 控制及末帧零速度保持。独立规划器的默认 `timing="legacy"` 保持历史基准，当前 SDK 默认使用下面的弧长进给规划；该全局平滑进度方法作为可选 timing 模式保留。

几何检查合并到实际执行的 125 Hz 指令：省去原始 200 Hz 轨迹的重复 FK/几何遍历；原始轨迹仍向量化检查有限数、关节限位和速度。最终每条指令仍做全套碰撞对的保守剔除与精确候选距离查询（间隙至少 2 mm），绘图位置误差最多 1 mm。检查仅运行 MuJoCo 运动学，不计算无关动力学；缓存碰撞类别/名称，避免每帧重复构造数千项字典。执行中仍每 50 ms 检查实际姿态代理碰撞，并持续检查反馈与跟踪。没有提供跳过全部几何检查的执行开关；`native_plan.geometry_validation` 与 `command_validation` 明确区分原始数值检查和最终指令检查。

优化版本在官方仿真中通过 37 项 SDK 与 29 项 SVG 测试。银杏实测：输入到完整检查通过 14.41 秒（原始数值检查 0.007 秒，最终指令逐帧检查 4.69 秒），运动 84.85 秒，输入到核验结果 100.63 秒，125 Hz 平均控制频率保持；左右最大反馈 FK 误差 1.538/1.718 mm，通过 2 mm 阈值。此次从前一轮停留的末端起步，入场姿态与上一轮不同；固定相同原始起点、IK 分支和速度上限的离线计划对比为 109.395→78.675 秒（绘制 103.95→73.23 秒）。新旧几何算法在 21 个姿态、全部 3947 对上距离及几何变换差异均为零；该离线对比没有执行额外动作。结束后只读反馈仍保持末端，轨迹无返回阶段；闭合 SVG 的末端可能与本次起始姿态重合。完整精简证据见 [优化验证记录](examples/svg-ginkgo-optimized-verification.json)。

## 按末端速度执行，不固定总时长

`--tip-speed` 默认 0.2 m/s，是虚拟绘图点的规划目标巡航速度及指令上限；`--joint-speed` 默认 1 rad/s，是 18 个受控关节的规划上限，运行时还与厂家实时限值取较低者。没有固定执行时长：路径更长则用时更长，起停、拐弯以及关节速度/加速度约束会降低局部速度。双臂共用进度以保持既有耦合 IK/碰撞关系；最长一侧以目标速度巡航，较短一侧按路径长度比例同步减速，并非两臂不同长度时仍以同速同时完成。

弧长进给模式以各段关节样条导数/曲率上界构造速度和加速度预算，再前后传播规划进度速度；起点和终点速度为零。关节加速度预算为 2 rad/s²，进度起停对应的工具切向加速度预算为 0.4 m/s²，均为仿真规划参数而非厂家额定值。工具速度留 5% 插值和跟踪裕量，0.2 m/s 的指令巡航名义值约为 0.19 m/s。入场至少 4 秒并检查工具速度，避免用绘制速度强行加快姿态过渡。`--duration` 仅为可选的更慢调试时长，默认不传；短于当前受约束计划则在执行前拒绝。`--speed-scale` 默认 1，保留减速兼容，不传 duration 时若要提高速度应直接调整 tip-speed/joint-speed。

最终每条 125 Hz 计划指令仍检查间隙、形状、关节速度，并用工具点 Jacobian × 解析关节速度检查末端速度。当前快速仿真配置默认使用 20 ms 命令前瞻：名义轨迹/计时不变，成对位置/速度取未来 20 ms 的同一条曲线，补偿测得约 21 ms 的链路/跟踪时延。JSONL 同时记录 trajectory_time_s 与 command_time_s；误差核验仍使用原始名义时刻和原有 2 mm 阈值，不事后平移反馈时间。command-preview-ms 可在 0–30 ms 内调整，0 关闭；硬件执行禁止前瞻配置。

当前弧长进给版本在官方仿真完成验证：43 项 SDK 与 31 项 SVG 测试通过。银杏左右各约 1.953 米，入场 4 秒、绘制 23.72 秒，实际运动 27.729 秒；输入到检查通过 10.25 秒，输入到结果 38.39 秒，平均控制频率 125.00 Hz，结束停在末端。指令峰值：关节 0.422 rad/s、绘图点 0.190 m/s；反馈 FK 位置最大误差 1.696/1.605 mm，通过原 2 mm 检查。接收关节速度经 FK Jacobian 计算的瞬时峰值约 0.223/0.221 m/s，仍有伺服超调：tip-speed 是规划指令约束，并非已实现的物理硬限速。报告将两者分开，未把反馈速度当作通过的速度门限。1 米与 2 米直线合成回归分别约 5.74/11.005 秒，说明耗时随长度改变，非固定时长。见 [弧长进给验证记录](examples/svg-ginkgo-feedrate-verification.json)。
