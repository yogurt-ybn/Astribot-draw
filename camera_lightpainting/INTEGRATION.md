# 光绘模块接入接口 · v1

本模块负责相机图像处理、光轨累积和诊断记录。机器人调度、运动控制、相机标定和空间坐标转换由上层 pipeline 提供。本次整理保留 `lightpaint_engine.py` 中的背景过滤与绘制算法。

## 模块边界

```text
USB / 视频 / 上游 RGB 帧
        │
        ├─ capture.CameraWorker：采集、限宽 960、镜像、逐帧调用
        │
        ▼
pipeline.LightPaintingPipeline：参数校验、会话状态、线程锁
        │
        ▼
lightpaint_engine.LightPainter：背景模型 → 光源筛选 → 定位 → 光轨累积
        │
        ▼
pipeline.FrameResult：图像 / 掩膜 / 光点 / 时间戳 / 统计
        ├─ 上游程序直接读取，继续处理
        ├─ diagnostic_recorder：原始视频与处理证据
        └─ web_controller → web_app → web/：局域网接口与操作页面
```

同进程集成优先使用 Python 接口。独立进程通过 HTTP 控制现有相机服务。Python 接口不打开相机、不启动网页、不写文件。每个相机或独立绘制作业使用一个 pipeline 实例；同一实例串行处理帧。

## Python：传入帧、取回结果

从仓库根目录运行，依赖为 `camera_lightpainting/requirements.txt`：

```python
import cv2
from camera_lightpainting import LightPaintingPipeline, PipelineSettings

pipeline = LightPaintingPipeline(PipelineSettings(mode='exposure', light_color='cyan'))

# 上游负责取得画面；两个输入必须尺寸、镜像方向一致。
empty_bgr = cv2.imread('empty_scene.png')
frame_bgr = cv2.imread('frame.png')
pipeline.set_background(cv2.cvtColor(empty_bgr, cv2.COLOR_BGR2RGB))
pipeline.set_roi((20, 20, 600, 440))  # 可选：像素边界，右/下界不包含

result = pipeline.process_frame(
    cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB),
    accumulate=True,
    sequence=1,  # 可选；省略时自动递增
    # captured_ns=采集时的 time.monotonic_ns()，省略时由模块生成
)

image_rgb = result.image       # 累积结果 RGB uint8 H×W×3
source_mask = result.mask      # uint8 H×W，0/1
point_xy = result.position    # (x, y) 或 None
metadata = result.metadata()  # 可 JSON 序列化的元数据
cv2.imwrite('painting.png', cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))

pipeline.process_frame(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB), accumulate=False)
pipeline.break_stroke()        # 外部暂停边界：下一次累积另起一笔
pipeline.configure(threshold=235, change_threshold=30)
pipeline.clear()               # 清画布，保留背景及 ROI
pipeline.clear(keep_background=False)
```

输入为 **RGB、uint8、H×W×3**，宽高至少 2；OpenCV 默认 BGR 需要转换。Python 接口不自动缩放或镜像，调用者决定前处理。改变输入尺寸或相机方向后，应清空画布、重设 ROI 并重新采集背景。ROI 应在输入图像内。`process_frame(accumulate=False)` 继续学习背景但不增加累积帧数；显式暂停时调用 `break_stroke()`，防止跨暂停连接。

`FrameResult` 的数组是独立快照，后续帧、调用者修改输入或结果不会改变其他结果或内部画布。序号与时间戳可由上游提供；模块不排序、不做时间同步。`captured_ns` 是单调时钟值，**不是 Unix 时间，也不能跨电脑直接比较**。

`FrameProcessor` Protocol 声明逐帧处理方法，可作为上游的类型边界。`WebController(pipeline=...)` 支持注入 `LightPaintingPipeline` 实例或保留相同控制契约的派生实现；底层 `CameraWorker(..., processor=...)` 调用 `process_frame`。当前诊断记录还使用同一个 `LightPainter` 的内部背景及跟踪状态，替换整套算法时须一起适配该诊断契约。

### 参数

| 字段 | 类型 / 范围 | 默认 |
| --- | --- | --- |
| `mode` | `neon` / `exposure` | `neon` |
| `light_color` | `cyan` / `white` | `cyan` |
| `threshold` | 整数 100–254 | 230 |
| `change_threshold` | 整数 8–80 | 25 |
| `width` | 整数 1–16 | 4 |
| `filter_static` | bool | true |
| `only_bright` | bool | true |
| `overlay` | bool | false |
| `overlay_brightness` | 整数 0–100，底图亮度百分比 | 60 |

参数修改先完整校验再应用；非法值抛出 `ValueError`。切换模式、光源类型或静止过滤会清空已有画布，保留背景和 ROI。阈值、宽度只影响后续处理；背景叠加及底图亮度可以重新渲染已有光轨。镜像是采集层设置，不在 `PipelineSettings` 内。

叠加使用最后一帧 `accumulate=True` 的 RGB 底图，与纯光轨逐通道取最大值；底图先乘 `overlay_brightness / 100`。`accumulate=False` 不更新该底图，`break_stroke()` 标记暂停。即使绘制时未开启叠加，暂停后也可以 `configure(overlay=True)`，然后 `snapshot()` 立即重新合成；不增加累积帧数、不重新检测光源。`clear()` 丢弃底图。底图是处理后（缩放/镜像）的原始 RGB，不含定位标记和区域框。

`FrameResult.metadata().overlay` 和 `/api/v1/state` 的 `overlay` 字段含 `enabled`、`brightness`、`base_sequence`、`base_captured_ns`、`frozen`。暂停后处理帧序号可能继续变化，底图序号保持不变。PNG 的 `X-Frame-Sequence` 标识输出快照对应的处理帧，底图来源以 overlay 元数据为准。HTTP 暂停或调整叠加后会立即重新渲染，GET PNG 无需等待下一相机帧。

## HTTP：连接现有相机服务

部署地址示例：`http://192.168.0.75:8765`。API 版本前缀固定为 `/api/v1`，旧的 `/api/state`、`/api/settings`、`/api/action` 等入口仍兼容。所有控制请求使用 JSON；成功返回 JSON，控制条件不满足返回 HTTP 409 和 `{"error":"原因"}`。非法请求体返回 400，跨站或不允许的主机请求返回 403。服务面向可信局域网，无用户登录。

| 方法 | 路径 | 内容 |
| --- | --- | --- |
| GET | `/api/v1/capabilities` | API 版本、模式、格式、接口发现 |
| GET | `/api/v1/state` | 连接、绘制、背景校准、帧率、参数、诊断状态 |
| GET | `/api/v1/devices` | 相机设备列表 |
| POST | `/api/v1/settings` | 修改参数，例如 `{"threshold":235}`；另支持 `mirrored` bool |
| POST | `/api/v1/action` | 见下表，成功通常返回最新状态 |
| GET | `/api/v1/frame` | 最新处理帧元数据，尚无帧时 409 |
| GET | `/api/v1/result.png` | 无损 RGB 累积图像，无累积结果时 409 |
| GET | `/api/v1/preview/{view}.jpg` | JPEG 预览，view 为 `camera` / `selected` / `mask` / `result` |
| GET | `/files/...` | 下载 save 或录制接口返回的 PNG / ZIP 地址 |

`/api/v1/frame` 包含 `sequence`、`captured_ns`、`resolution: [w,h]`、`position`、`normalized_position`、`source_pixels`、`tracking_event`、`processing_ms`、帧级参数及累积统计。坐标针对**处理后的画面**，已经包含相机采集层的缩放和镜像；不是机器人或世界坐标。

PNG 响应携带 `X-Frame-Sequence` 和 `X-Captured-Ns`，与本次图像快照对应。单独获取的 `/frame` 与 PNG 可能属于不同帧，不能假设两个 HTTP 请求是原子快照。预览和元数据接口只提供最新帧，浏览器不消费全部帧；需要逐帧、同步掩膜与坐标的上游应使用 Python `FrameResult`。

### 控制动作

| action | 额外参数 | 结果 |
| --- | --- | --- |
| `background` | 无 | 从最新画面采集背景并清空画布，保持当前绘制/暂停状态 |
| `start` / `pause` | 无 | 开始/暂停，暂停保留画布并断开当前笔画 |
| `clear` | 无 | 清空画布，保留背景和 ROI |
| `roi` | `coordinates: [x0,y0,x1,y1]` 或 null | 0–1 归一化矩形边界，最终至少 8×8 像素；null 取消 |
| `connect` | `source: "0"` 或 Linux `/dev/...` 路径 | 切换相机并重置背景、ROI 和画布 |
| `save` | 无 | 写入服务器 PNG，返回 `{name,url}` |
| `record_start` | 无 | 开始异步诊断录制 |
| `record_stop` | 无 | 停止并异步打包；轮询 state.recording.finalizing/result |

所有浏览器和 HTTP 调用者控制同一个会话，关闭页面不会停止绘制或诊断。建议由上层 pipeline 指定单个控制方，其他页面用于观察。

### HTTP 客户端示例（Python 标准库）

```python
import json
from urllib.request import Request, urlopen

base = 'http://192.168.0.75:8765'

def action(name, **extra):
    data = json.dumps({'action': name, **extra}).encode()
    request = Request(base + '/api/v1/action', data=data,
                      headers={'Content-Type': 'application/json'})
    with urlopen(request, timeout=10) as response:
        return json.load(response)

action('pause')
# 此时应已确认画面中没有绘制光源。
action('background')
action('start')
# 上层执行其绘制任务；本模块不触发机器人运动。
action('pause')
with urlopen(base + '/api/v1/result.png', timeout=10) as response:
    image = response.read()
    frame_sequence = response.headers['X-Frame-Sequence']
with open('painting.png', 'wb') as stream:
    stream.write(image)
```

接入服务时可用 `python -m camera_lightpainting.web_app --camera 0` 或现有启动脚本。预留的扩展边界为帧输入、帧处理结果与会话控制；当前未引入 ROS、消息队列或机器人执行依赖。
