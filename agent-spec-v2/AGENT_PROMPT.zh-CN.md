请完整阅读 AGENT_DRAWING_SPEC.md、execution-contract.json 与 inputs/ 掩膜和标定，再生成双臂一笔画图案。当前模式是固定底盘、真实躯干关节与双臂协同、双臂同步开始和结束绘制，不考虑人体。

严格输出 paths.json、570×471 RGBA 透明 drawing.png、独立 preview.png、generate.py 和 validate_drawing.py 产生的 validation.json。使用 drawing_contract.py 标准渲染器；不得改掩膜、阈值或验证器来通过。

两臂各一条平滑曲线，遵循区域及边界余量。JSON 的 execution 元数据须与 execution-contract.json 完全相同。静态设计通过后仍保持 robot_execution_status=not_tested。图案 agent 不生成关节指令、不假设单臂通过代表双臂同步通过。

独立执行后端必须联合规划一条共享躯干轨迹和两臂轨迹，检查全过程双臂及机器人自身碰撞，再实际积分执行。报告碰撞覆盖、排除对、最小距离、时刻、同步时间及跟踪结果，未验证项不得冒充通过。保留输入图形，若需改形状，说明并另存新版本。
