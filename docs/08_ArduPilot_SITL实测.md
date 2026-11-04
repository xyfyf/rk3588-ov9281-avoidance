# ArduPilot SITL 接入实测（2026-11-03）

本次实际启动用户提供的 `/home/eft/ardupilot/build/sitl/bin/arducopter`，不是此前的一维点质量控制演示。
固件报告 `FIRMWARE_VERSION FC2.0_S3.0.87_R_T`；SHA256：`da5f1e39993b760f91e12b09472ee2c13417f7c09790c805632821170818a2ba`。Ubuntu22.04/WSL，x86_64，speedup=1。
用户的ArduPilot仓库含未提交修改，本次没有修改或重置仓库；结果仅适用于该二进制，不自动推广到其他官方或定制版本。

## 已确认的结果

- 地面方向测试：前/右/后/左1/2/3/4米均获得正确回显。
- 原始0 cm被该固件忽略；1 cm被接受。v0.2.1的正式编码器将内部饱和0映射为线上1 cm，接收复测通过。未知65535保持不变，不把未知补成近障。
- 关闭避障：越过4米外的虚拟墙面，末2秒北向速度均值0.754 m/s。
- 开启避障：持续向前输入仍停在墙前，最小机体参考点间距1.029 m，末2秒北向速度均值-0.0033 m/s。
- 第7秒起持续发布全未知：恢复前进并越过墙面，末2秒北向速度均值0.735 m/s。**全未知不是停车命令。**
- 实际四目图像算法接入：未越过盒体前表面；减速过程最小至盒体前表面的平面间距0.676 m，最终回放间距约1.03 m，末2秒北向速度均值-0.0012 m/s。

正式飞行对照都在3m左右高度执行，俯仰1100PWM持续16秒，之后回中并LAND至自动上锁。已检查回读遥控量、飞行阶段高度、参数一致性、二进制一致性。
参数包括`PRX1_TYPE=2, AVOID_ENABLE=0/2, AVOID_BEHAVE=1, AVOID_MARGIN=1, AVOID_BACKUP_SPD=0, LOIT_SPEED=100`。
LOIT_SPEED单位为cm/s，是配置值而非硬限速承诺；本次开启避障飞行的实际峰值为1.392 m/s，存在瞬态超调。不能从此推导最大安全飞行速度。

## 图像与消息频率必须区分

合成场景：四个640×400理想KB4鱼眼，虚拟双目384×192，四对并行，OpenCV 4.5.4，每对OpenCV线程1。
由飞控位置/偏航驱动软件光线渲染，再运行当前`StereoPipeline`、`process_images`和正式`obstacle_message`编码器；飞控接收结果并自行控制，脚本没有根据距离发停车指令。

本轮共102组四目图像，新图像平均5.23 Hz；渲染中位数155.7 ms，算法中位数31.2 ms。
障碍消息实测平均18.40 Hz，其中包含对最新图像结果的重复发布；不代表每秒获得20组新图像。
图像源时间戳在重复发布时保留。软件渲染是本轮传感器速率的主要限制；不能据此推导RK3588只能运行这一频率，也不能据PC算法耗时证明RK3588可达20Hz。
仅此隔离测试允许图像年龄650ms；正式运行仍为200ms，未修改标定、同步、真实采集及发送门限。

## 已知边界

- 虚拟障碍没有物理碰撞网格；“越过”按位置几何判断。盒体横向范围为±0.35m，墙面场景为无限平面。
- 图像相机只跟随水平位移和偏航，忽略横滚、俯仰、高度变化、曝光、运动模糊、真实标定误差与相机/串口延迟。不能称为完整六自由度视觉传感器仿真。
- 之前的图像回放已出现偏近误报和近场未知；本次能停车不能证明测距可靠，也不能保证细线、玻璃、弱纹理障碍可检出。
- 仅水平避障，不涵盖上下方；间距未减机体或旋翼半径。未测PX4、RK3588、实物OV9281和实飞。
- 后续需要飞控侧明确的感知失效处置；持续发未知或critical心跳不能替代这一功能。本轮没有添加自动切模式或速度指令来掩盖失效结果。

## 复现

在Ubuntu22.04项目根目录、安装依赖后执行：

```bash
SITL_BINARY=/home/eft/ardupilot/build/sitl/bin/arducopter \
ARDUPILOT_REPO=/home/eft/ardupilot \
PYTHON=.venv/bin/python bash scripts/run_ardupilot_sitl.sh
```

脚本依次运行接收、关闭/开启避障、未知和图像场景，创建带时间的新目录并生成`sitl.html`。每轮SITL约需1到2分钟，初始化需等待EKF，现有EEPROM目录不可复用。
也可单独运行`python3 tools/ardupilot_sitl_test.py --scenario avoid_on --output examples/new_sitl/avoid_on_01`。
本机WSL用户Python装有OpenCV5，因此实测通过预加载Ubuntu发行版OpenCV4.5.4运行；新建符合pyproject依赖的独立环境即可避免此冲突。

测试器只启动自己的Linux`/build/sitl/` ELF子进程、连接固定localhost TCP16410、校验sysid42和SITL参数，使用独立EEPROM；finally清理自己启动的进程。
ARMING_CHECK=0等参数只用于这个模拟机，不能复制到实机。不会连接真实飞控，正式算法仍不发送解锁、模式、速度或位置指令。

## 文件与无效运行

`examples/ardupilot_sitl_20261103/sitl.html`为可离线回放报告，`summary.json`为汇总，`validation_audit.json`记录一致性检查。
各运行目录保留`result.json`、`telemetry.jsonl`、`sitl_stdout.log`、独立参数/EEPROM及DataFlash日志；后续运行另记录`obstacle_inputs.jsonl`。
当前正式对照为`avoid_off_05`、`avoid_on_01`、`avoid_dropout_01`、`visual_on_01`，接收修复复测为`reception_02`。

排障记录保留：avoid_off_01等不到局部位置；02在EKF就绪前解锁失败；03遥控覆盖ID不一致，落地后仍记录，虽然旧脚本completed=true但测试无效；04输入过小未接近墙面。
根因修正：等EKF正常融合后重新请求普通解锁，地面站来源ID与MAV_GCS_SYSID匹配；起飞后检查RC回读与高度。局部位置会在EKF正常融合后出现，兼容路径亦可从全球位置派生并标注来源。

本次版本在Windows和Ubuntu22.04 WSL上均通过42项单元测试，含新增的零距离线上编码回归测试。
