# 通用 T1 SVG 双臂同步规划与验证

本版本对每个 SVG 运行相同配置、相同确定性搜索，使用同一个全身中立起点。底盘固定，躯干四关节参与 18DOF 入场/返回；绘制阶段使用统一有限躯干候选族。不是按图形编写动作，也不是完整全身优化器。

## 运行

需要 Python 3.10+ 与 `pip install -r requirements.txt`。官方模型不随包分发；下载见 `download_t1_assets.py`；它会在本地生成资产校验清单，仓库不分发该清单或模型资产。模型固定为 Astribot T1 官方 aos 提交 `cab872dc00081d2c5b096c7a19e563050eb7a2ff`。

```bash
cd svg_planner
GIT_LFS_SKIP_SMUDGE=1 git clone --branch aos https://github.com/Astribot-Dev/astribot_descriptions.git descriptions
git -C descriptions checkout cab872dc00081d2c5b096c7a19e563050eb7a2ff
python download_t1_assets.py
export ASTRIBOT_T1_ASSETS="$PWD/descriptions"
OPENBLAS_NUM_THREADS=1 python -m unittest -v
OPENBLAS_NUM_THREADS=1 python svg_plan.py inputs/ginkgo.svg --out my-run --mode simulate --seeds 12
# 对 inputs 下全部 SVG 顺序执行；拒绝覆盖已冻结同名基准目录：
# 轻量包已经含参考结果，先移开参考目录，避免覆盖证据：
mv runs/frozen-v3-analytic-time runs/reference-v3
OPENBLAS_NUM_THREADS=1 python run_benchmark.py
# 完整重跑后（已有本地 plan.npz/execution.npz）再验证：
PYTHON=python ./verify_benchmark.sh
```

`--mode static` 只验证格式/画布；`plan` 增加完整运动学计划；`simulate` 增加实际 MuJoCo 动力学。退出 0 只代表请求的阶段通过，不代表实机安全。

每个 SVG 使用同一条 CLI，无专用参数。新增图案只需要放入 `inputs`，不需要修改算法。批处理冻结源码、输入、画布及环境元数据，保留拒绝原因，不改图形、不静默缩放。

## 查看证据

- `runs/frozen-v3-analytic-time/freeze.json`：运行前配置和哈希。
- `runs/frozen-v3-analytic-time/summary.json`：三例逐阶段结果。
- 每例 `report.json`：候选搜索、全计划门控、实际误差、间隙和返回姿态结果。
- `plan.npz`、`execution.npz`：本地完整计划/执行状态（较大，轻量交付不包含）。可重跑复现。
- `GENERIC_METHOD.md`：明确适用范围、算法、限制。
- `SVG_GENERATION_SPEC.md`：给生成 agent 的 SVG 约定。该格式/平滑性预检不等于可执行性证明。
- `CHANGELOG.md`：相对 v1 的统一修订。v1 在新增共同起点功能后中止，绝不计入成功或失败。

所有碰撞检测均采用相同 3947 个显式碰撞对及原有复合关节接合处排除项。轨迹是空中光点，不包含实体画笔接触/力控制。详见方法说明；不发送任何实机指令。

## 轻量包中的复核样本

`review-samples/*-10hz-review.npz` 保留成功例每20个原始5ms步的状态，且包含最后一步：`qdes/dqdes/qpos/qvel/actual/desired/phase/home`、原始步索引、指令时间及步后实际状态时间。它们仅供复核，不是可直接发送给控制器的轨迹；所有通过结论来自未降采样的原始执行与报告所列检测频率。`verification.json` 保留完整源 NPZ 哈希。重跑可生成完整5ms轨迹和CSV。

`SVG_INPUT_INTERFACE.md` 是解析器接口约束；`SVG_GENERATION_SPEC.md` 是生成 agent 使用的更严格平滑图形规范。`examples/` 是既有解析器回归测试素材，`inputs/` 才是本次三个独立生成的正式测试输入。

## 测试图片与结果

| 原始 SVG | 同源 PNG | v3 结果 |
|---|---|---|
| [银杏](inputs/ginkgo.svg) | [预览](inputs/ginkgo.png) | 仿真通过，238.755 s |
| [锦鲤](inputs/koi.svg) | [预览](inputs/koi.png) | 仿真通过，295.635 s |
| [翅膀](inputs/wings.svg) | [预览](inputs/wings.png) | 静态通过，IK 搜索拒绝，未执行 |

[实际轨迹对比图](visuals/three-design-comparison.jpg) · [结果与限制](RESULTS.md)

`generation/` 提供独立生成规范、prompt、匹配的静态校验器与遮罩。先安装其 requirements，再运行 `python generation/validate_and_render.py --help`。静态通过不表示可执行。

### 发布与复现说明

见 [PUBLISHING.md](PUBLISHING.md)。参考 freeze 保留原始仿真源码哈希；本次仅做可移植路径和打包调整，没有重新跑动力学仿真。`verify_benchmark.sh` 用于完整重新运行后核验，不能在缺少完整 NPZ 的轻量仓库上直接通过。`PUBLICATION_SHA256.json` 描述当前公开文件哈希，`publication-checks.json` 记录本次轻量检查。

模型来源：[Astribot-Dev/astribot_descriptions](https://github.com/Astribot-Dev/astribot_descriptions/tree/cab872dc00081d2c5b096c7a19e563050eb7a2ff)。此处保留来源署名，不替上游声明许可证或再分发模型权利。

按用户要求，本次 GitHub 发布不包含 MP4；保留实际轨迹对比图片和 10Hz 复核数据。渲染元数据记录的是原始本地生成过程，不表示视频已上传。

## 面向 agent 的遮挡构图要求

更新后的 [SVG 生成规范](SVG_GENERATION_SPEC.md) 与 [任务提示](AGENT_PROMPT.md) 明确：中心 50 cm（u=[208,408]）是 ASSUMPTION_ONLY 视觉遮挡，不是禁画区域。满足原始对应可达遮罩的完整路径可以进入中央区域；仅独立预览覆盖遮挡层，主要辨识特征尽量在两侧保留。50 cm 尚未经人体/相机标定，不能替代人体安全或全机器人碰撞检查。此文档更新不改变规划器和可达遮罩，也没有新增 IK、规划或仿真结果。`generation/` 内规范与提示保持一致。
