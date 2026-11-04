# RK3588＋四路 OV9281 鱼眼避障算法包

目标环境：RK3588、64 位 Ubuntu 22.04、Python 3.10、OpenCV。输出同一套 MAVLink 2 `OBSTACLE_DISTANCE`，供 PX4 或 ArduPilot 分别配置使用。

**这是一套可运行、可移植的研究基线，已在 Windows 和 Ubuntu 22.04 WSL 中完成离线算法、协议测试及用户提供的定制 ArduPilot SITL 测试；尚未经过你的四目硬件、RK3588、PX4 SITL 或实机飞行验证。**拿到相机后需要完成驱动、硬件同步、标定、遮挡掩膜和飞控制动验收。示例参数不会被当作真实标定发往飞控。

## 已实现

- 全角度 KB4 鱼眼投影，支持大于 180°镜头的后半球边缘，避免直接套用普通针孔投影。
- 四组相邻相机虚拟针孔重投影、SGBM 双向匹配、左右一致性、纹理及视差不确定度筛选。
- 机体 FRD 坐标下的水平 72 个距离分区；每格 5°、单位厘米；未知为 65535。
- 近距离优先的有效点簇选择、保守距离下界、可选角向膨胀；不把缺测、远处失配或无纹理区宣布为畅通。
- Linux V4L2 单平面/多平面 mmap 采集，使用内核单调时钟时间戳、帧序号、有限队列及帧组时间差检查。
- 独立 MAVLink 发布线程，按采集时间淘汰过期结果；只发送障碍消息、伴随计算机心跳和状态文字。
- 带唯一 ID 的 ArUco 标定板、KB4 内参优化和四目联合外参优化。
- 合成四目场景、离线回放、结果网页、点云、性能测量、日志统计、接收检查、Ubuntu 安装脚本和 systemd 示例。

这里的“全向”指水平环向，且是否有有效测距取决于真实重叠、距离、遮挡和纹理。上下避障、VIO/SLAM、自主航线规划、目标识别、动态障碍预测不在本包内。

## 先看什么

1. `docs/01_调研与选型.md`：官方资料、为什么这样实现、镜头和近场盲区。
2. `docs/02_算法与数据协议.md`：几何、筛选、消息、未知和过期行为。
3. `docs/03_Ubuntu2204部署.md`：从安装到相机节点、回放和实时运行。
4. `docs/04_四目标定.md`：打印、采集、求解和掩膜。
5. `docs/05_PX4与ArduPilot接入.md`：两套飞控的配置区别及验收。
6. `docs/06_测试与验收.md`：已完成的验证、未验证项和速度测定方法。
7. `docs/07_PC动态仿真与实时性.md`：当前电脑实测速率、动态四目图像仿真和简化制动演示。
8. `docs/08_ArduPilot_SITL实测.md`：实际飞控程序的接收、停车、未知失效及图像闭环实测。

新增 PC 动态仿真：双击 `打开动态仿真.cmd` 或打开 `examples/realtime_20261103/simulation.html`，可以播放和拖动时间轴。这是已计算结果的交互回放；没有运行 PX4/ArduPilot SITL。四组双目并行配置在 i7-12700K 上的完整分辨率连续 1000 次计算 P95 约 34 ms，详见原始报告；不能推断 RK3588 或真实飞机性能。

直接打开 `examples/synthetic_demo/report.html` 可以看合成场景结果及可调的制动距离估算。`front.png` 等是合成鱼眼输入；`pair0_left.png` 等是重投影图；`cloud.npz` 是机体坐标三维点。

## Ubuntu 22.04 快速开始

把整个文件夹复制到 RK3588，例如 `/opt/omniavoid` 或自己有写权限的工作目录，然后在项目根目录运行：

```bash
bash scripts/install_ubuntu2204.sh
bash scripts/run_demo.sh
```

安装脚本需要网络和 apt 权限，会安装用户空间依赖、建立本项目 `.venv` 并运行测试；不会安装板卡驱动，也不会修改飞控参数或自动启用服务。

先用示例检查几何覆盖：

```bash
.venv/bin/python -m omniavoid inspect --config configs/rig.example.json
```

完成真实标定后，先只记录结果：

```bash
.venv/bin/python -m omniavoid run --config configs/rig.calibrated.json \
  --duration 60 --output logs/bench.jsonl
```

实物验证、飞控接入条件全部完成后，才开启数据发送：

```bash
.venv/bin/python -m omniavoid run --config configs/rig.calibrated.json \
  --transmit --output logs/live.jsonl
```

`--transmit` 需要配置中的真实标定、机体系对齐、遮挡区域、硬件同步及曝光时间戳验证项全部成立；回放和合成模式没有发送选项。算法不解锁、不改飞行模式、不发送速度或位置指令。

## 当前默认值的含义

示例几何是相机中心距机体中心 0.12 m、相邻视轴相隔 90°、水平标称 200°的理想等距鱼眼。这些是说明算法的假设，**不对应你尚未购买的镜头和机架**。

采集示例为 1280×800；每对虚拟图 384×192、64 级视差；发布目标 20 Hz、最大结果年龄 200 ms。这些是起步配置，不是已经测得的 RK3588 性能或飞行速度。`capture.fps` 仅记录期望帧率，实际帧率、触发和曝光由板卡 BSP/传感器工具配置。

`examples/benchmark_windows.json` 是开发电脑上的静态图像计算测试，不能换算成 RK3588 帧率。实际允许速度需要可靠测距范围、端到端延迟和实机制动距离共同确定。

## 最重要的限制

四目 200°也可能存在近场双目盲区。无纹理墙面、玻璃、细线、反光、重复纹理、低照度、旋翼遮挡和运动模糊都可能导致漏检或误检。有效分区数量不等于障碍检出概率。可靠低速原型建议为近场增加 ToF 等独立测距；具体融合和失效策略需在飞控侧另行验收。

代码位于 `omniavoid/`；仅 Python 调度，主要图像运算由 OpenCV 原生实现完成。当前版本不调用 RKNN/NPU、Mali OpenCL 或 Rockchip FEC，不需要训练权重，也不承诺硬实时。

## ArduPilot 实际 SITL（v0.2.1）

已运行用户提供的定制 arducopter，完成距离接收、Loiter停车、全未知失效及实际四目图像算法接入测试。双击根目录“打开ArduPilot仿真.cmd”查看回放，详见 `docs/08_ArduPilot_SITL实测.md`。开启避障停在约1.03m；全未知后恢复前进，不能当作故障停车方案。此结果不涵盖PX4、RK3588或实飞。
