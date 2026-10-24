# USB 摄像头光绘

Windows / Linux 局域网光绘工具，使用 Python、OpenCV 和 HTML。后台连接 USB 相机并处理图像，浏览器提供操作界面。默认针对青蓝色光棒，重点通过背景学习和颜色/发光强度筛选，排除固定亮灯、窗户、白衣服及较暗的蓝色管身。所有画面和诊断数据都保存在本机。

## 安装与启动

在仓库根目录执行（Python 3.10 以上）：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r camera_lightpainting\requirements.txt
.\start_lightpaint.cmd
```

安装后可以直接双击根目录的 `start_lightpaint.cmd`，然后用浏览器打开 http://127.0.0.1:8765/ 。同一局域网的设备使用 `http://相机电脑的局域网IP:8765/` 访问。启动器优先使用模块或仓库根目录中的 `.venv`，其次使用 Windows Python Launcher 或 PATH 中的 Python；可用环境变量 `LIGHTPAINT_PYTHON` 指定解释器。脚本不依赖 Codex 或开发者本机路径。

也可以直接运行：

```powershell
.\.venv\Scripts\python.exe camera_lightpainting\web_app.py
.\.venv\Scripts\python.exe camera_lightpainting\web_app.py --video 'C:\path\to\test.mp4'
```

## 绘制流程

### Linux / Ubuntu

需要 Python 3.10 以上，不依赖图形桌面或 Tk。可通过 SSH 启动。在仓库根目录创建虚拟环境并安装模块依赖：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r camera_lightpainting/requirements.txt
bash start_lightpaint.sh --camera 0
```

Linux 使用 V4L2 相机接口；可用 `--camera /dev/v4l/by-id/...-video-index0` 指定稳定设备路径。默认监听 `0.0.0.0:8765`；可用 `--host`、`--port` 修改。相机电脑处理全部画面，多个浏览器共享同一相机、参数与画布；关闭网页不会停止后台或诊断录制。网页面向可信局域网使用，没有登录功能。用 Ctrl+C 停止前台服务，或通过部署后的用户服务管理后台进程。

### 操作

1. 选择相机编号并连接。编号由 Windows 驱动顺序决定。
2. 固定相机，移走绘制光源，点击“采集背景”。背景采集会清空画布。
3. 青蓝光棒选择“青蓝光棒”，手电筒选择“白光 / 手电”。
4. 选择绘制模式，点击“开始绘制”，再移动光源。可以暂停、清空和保存 PNG。

| 模式 | 行为 |
| --- | --- |
| 取亮长曝光 | 对筛选后的真实像素取最大值，保留光源颜色、发光面积，不进行路径插值或断点补线。它是数字取亮叠加，不是物理光量积分。 |
| 彩虹轨迹 | 提取光源中心，平滑后绘制渐变光轨。连续检测点之间形成路径；漏检后另起一笔，不补接断点。 |

“过滤静止光源”默认开启。预览时较快学习背景，绘制时较慢更新；亮度变化不足的固定像素不参与绘制。清空及切换模式保留背景，切换相机或镜像重新学习。相机移动或环境光大幅变化后应重新采集背景。

“亮度阈值”默认 230：青蓝模式判断颜色亮度，白光模式判断灰度亮度。“变化阈值”默认 25，越大背景过滤越强；光源漏检时可降低，固定亮点干扰时可提高。识别区域可在左侧画面拖框限定。

空格开始/暂停，Esc 暂停，Ctrl+S 保存。输出图片和录制文件默认位于 `camera_lightpainting/output/`，可用 `--output-dir` 指定；PNG 在 `captures/` 子目录，网页保存按钮会同时触发浏览器下载。启用“相机背景”时，暗化的实时背景仍显示在结果下面，它与累积的光轨分开处理。

## 诊断录制

在网页下方的“诊断录制”区域操作；无需额外启动程序。

1. 点击“开始录制”，先录制约 3 秒空背景。
2. 采集背景、开始绘制，按实际执行速度移动光源，保留平时出现问题的环境光。
3. 点击“停止录制”。等待打包完成后，点击“下载诊断包”。关闭浏览器不会自动结束录制。

每次录制保存在 `camera_lightpainting/output/diagnostics/日期时间/` 和同名 ZIP 中：

| 文件 | 用途 |
| --- | --- |
| `raw.avi` | 原始分辨率、未镜像、未去背景的相机画面，无音频。 |
| `debug.avi` | 输入与定位点、光源筛选掩码、筛选后的光源、最终结果组成的四格视频。 |
| `capture.csv` / `processed.csv` | 帧编号、时间、处理耗时、跳帧计数、定位、参数和状态。 |
| `background_*.npy/png` / `events.jsonl` | 背景模型快照、采集背景和开始绘制等操作。 |
| `manifest.json` / `final.png` | 环境版本、写盘丢帧统计和最终图片。另保存当次算法源代码。 |

AVI 使用标称帧率，真实时间以 CSV 为准。算法与采集在同一后台线程逐帧执行，预览只显示最新结果；预览跳帧不会跳过算法输入。若处理速度低于相机帧率，实际采集速度仍可能下降。录制缓冲有容量限制，磁盘写入过慢造成的丢帧会记入统计。

## 模块与测试

```text
camera_lightpainting/
  lightpaint_engine.py    # 背景模型、光源筛选、取亮累积和彩虹渲染
  capture.py              # 相机/视频采集和逐帧处理，独立于界面刷新
  web_app.py              # 局域网 HTTP 接口与服务入口
  web_controller.py       # 共享相机状态、操作与预览编码
  web/index.html          # HTML/CSS/JS 操作界面
  lightpaint_app.py       # 保留的旧 Tk 界面，默认启动器不再使用
  diagnostic_recorder.py  # 异步诊断写盘与 ZIP 打包
  start_lightpaint.ps1    # Windows Python 启动器
  tests/                 # 合成画面、视频管线和诊断包测试
```

在仓库根目录运行：

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s camera_lightpainting/tests -t camera_lightpainting -v
```

测试覆盖固定亮源过滤、移动白衣服与蓝色管身排除、青蓝光源保留、真实像素累积、禁止断点补线、预览未读取时的完整逐帧处理，以及诊断视频、CSV 对应关系和 ZIP 完整性。网页测试覆盖参数校验、背景采集、拖框区域、暂停、清空、PNG 保存、下载路径限制和诊断录制生命周期。无需物理相机或私人测试视频。

## 当前限制

针对单个主要光源；多光源可能选错目标。青蓝模式尚未适配其他颜色。同色且足够亮的背景、强反射、曝光饱和仍可能干扰，需要背景校准或限定区域；长时间停住的绘制光源也可能被学入背景。处理宽度最多 960 像素，取亮模式保留实际发光面积，因而可能比彩虹模式粗。此模块仅做图像处理与记录，不控制机器人执行。
