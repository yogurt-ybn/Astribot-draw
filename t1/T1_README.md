# Astribot T1 本地绘图适配

独立于仓库历史 S1 结果。此目录中的 T1 文件及 t1-results 才是新模型测试；不发布上游模型资产。

- 模型：Astribot-Dev/astribot_descriptions，aos，cab872dc00081d2c5b096c7a19e563050eb7a2ff。
- 固定底盘和车轮；四躯干关节可动，双臂同步。已配置 T1 关节范围、执行器、工具点、官方碰撞代理。
- 新画布616×560，2.5mm/px，平面y=-0.42m；t1-canvas/calibration.json为标定依据。新mask为采样支持范围，不是最大可达空间证明。
- 三组waves/leaves/spiral由同一生成规范生成，使用共享躯干上下运动。执行记录来自 MuJoCo actuator dynamics，没有在执行中重置qpos。

## 重现
Python依赖：numpy, scipy, pillow, mujoco==3.4.0；可视化另需ffmpeg和EGL。
下载官方固定提交资产（见download_t1_assets.py；默认本目录descriptions），设置：
```
GIT_LFS_SKIP_SMUDGE=1 git clone -b aos https://github.com/Astribot-Dev/astribot_descriptions.git descriptions
git -C descriptions checkout cab872dc00081d2c5b096c7a19e563050eb7a2ff
python download_t1_assets.py
export ASTRIBOT_T1_ASSETS="$PWD/descriptions"
python t1_sync.py waves
python t1_sync.py leaves
python t1_sync.py spiral
python t1_negative_controls.py
python t1_render.py
```
原始执行输出含大型scene.t1.mjb，仅供本地缓存，不需要交付/提交。紧凑包只含代码、图像、规范、测试报告和降采样测量记录；完整5ms记录留在本地。

## 已知边界
碰撞3947显式对，官方凸包代理+结构邻接排除，详见t1-collision-model.json。复合肘3/5局部折叠未覆盖，不能声称完整自碰撞安全认证。控制使用模型bias/passive前馈与期望速度补偿，是理想模型仿真；不代表真实执行器误差。当前位置点跟踪不约束笔姿态/纸面接触力，适用于光绘式轨迹演示。无人体、无底盘移动，无硬件运行许可。躯干轨迹预设，非全身优化求解器。规划距离每50ms、执行距离每10ms采样；物理积分5ms。有限测试不保证连续时间安全。

官方aos分支许可尚未核实，所以本包不包含网格、XML原始资产或编译模型，不将这些上游资产重新分发到本仓库。


发布说明：本次仅修正模型目录默认值及报告路径以便仓库复现，未重新运行完整动力学。原始已验证执行记录保留；发布前重跑轻量测试。asset_manifest.json 是下载器自行生成的缓存清单，不作为仓库输入。

## 更大范围：双翼用例

[大幅双翼测试与复现](T1_WINGS_README.md) 使用约 1.271 m 翼展、0.655 m 单翼高度；已完成同步双臂、共享躯干、进场/绘制/返回的实际仿真与独立审计。运行 `python t1_wings.py wings`。原三组小图保留作为回归用例。
