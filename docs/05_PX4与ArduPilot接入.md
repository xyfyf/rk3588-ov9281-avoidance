# PX4 和 ArduPilot 接入

同一程序发同一格式的 MAVLink 2 `OBSTACLE_DISTANCE`，不要求 ROS。本文针对多旋翼；不同机型和版本支持范围不同。以下是官方文档对应的配置方向，不是可直接批量导入的飞控参数文件。本包不写飞控参数。

## 共同步骤

选择与板卡连接匹配的 UART 或网络端口，核对双方 MAVLink2、波特率、系统/组件 ID 和消息路由。查看飞控内部的数据，不能仅凭电脑接收到了 UDP 就认定飞控已使用。

按前、右、后、左轮流摆放目标，核实 0/18/36/54 四个分区以及厘米单位、顺时针方向和 `angle_offset`。本包点云已转到机体中心，不要在飞控侧重复旋转同一角度。对应安全距离应包含机体/桨盘尺寸和制动余量。

观测有效频率和源采集时间；检查启动无数据、单相机停流、断链、全未知、进程挂起/退出的行为。两种飞控都需单独记录结果。

## PX4

官方 Collision Prevention 文档描述的入口是多旋翼相应的 Position 模式，配合 acceleration-based 位置控制；不要假设在任意 Offboard、Auto 或其他模式里收到消息就自动刹车。

- `CP_DIST`：期望保持的最小距离；按虚拟传感器原点（本包为机体中心）及桨盘外缘安排余量。负数禁用。
- `CP_DELAY`：传感与速度跟踪延迟，需要实测；不是简单设置成一个相机帧周期。
- `CP_GO_NO_DATA`：官方默认 0 表示不允许向无数据方向运动。初期保留 0，并实测验证。
- `CP_GUIDE_ANG`：允许偏转寻找更空方向的角度；第一阶段可先验证停车，之后再调引导。
- `MPC_POS_MODE`：依目标固件参数枚举选择官方要求的 acceleration-based 模式，不盲填其他版本的枚举整数。

确认 `obstacle_distance` / collision prevention 相关 uORB 信息更新；可在目标固件可用的 PX4 控制台执行 `listener obstacle_distance` 查看，再检查 flight log 和模式控制行为。

先在 SITL 或不装桨条件下验证消息与控制设定，再按实机测试流程验证。在 `CP_GO_NO_DATA=0` 下，如果画面无纹理导致大量未知，可能不允许向这些方向运动；这符合当前保守测距设计，不能用“未知=最远距离”绕过。

来源：<https://docs.px4.io/main/en/computer_vision/collision_prevention.html>

## ArduPilot Copter

官方 RealSense 示例配置为：连接的 `SERIALx_PROTOCOL=2`（MAVLink2），匹配 `SERIALx_BAUD`；`PRX1_TYPE=2` 使用 MAVLink proximity。老版本可能使用 `PRX_TYPE` 命名，应按本机参数元数据核实。

官方示例用 `AVOID_ENABLE=7`（所有对应屏障来源），其中包含 proximity。按现有系统已有的围栏/传感器配置选择所需位，不要盲目覆盖现有功能。

第一阶段以 Loiter 的 Simple Avoidance 停车行为验证：

- `AVOID_MARGIN`：障碍保持距离，考虑桨盘及制动余量。
- `AVOID_BEHAVE=1`：官方示例的 Stop；Slide 为其他行为，需单独验证。
- AltHold 还涉及 `AVOID_DIST_MAX`、`AVOID_ANG_MAX` 等倾斜避让设置，不能当作 Loiter 下的位置制动效果。

若要任务航线自主绕路，再按官方 BendyRuler 文档配置 `OA_TYPE=1`、`OA_BR_LOOKAHEAD`、`OA_MARGIN_MAX` 等，并验证障碍数据库和对应模式。收到距离不是已经具备路径规划。

在 Mission Planner 的 MAVLink Inspector 查看消息；通过 Proximity 和机载 PRX 日志核对方向/距离。地面站 proximity 视图可能按较粗角区显示，不意味着本包只发了 8 个数。

**ArduPilot 的未知值/传感器失效不应假定与 PX4 的 `CP_GO_NO_DATA=0` 等价。**本包只发布未知和健康状态，不发送强制刹车命令。需要针对实际固件和模式验证旧障碍何时过期、无数据后是否继续接受飞行指令，以及需要什么额外失效策略。

来源：

- <https://ardupilot.org/copter/docs/common-realsense-depth-camera.html>
- <https://ardupilot.org/copter/docs/common-simple-object-avoidance.html>
- <https://ardupilot.org/copter/docs/common-oa-bendyruler.html>

## 版本和测试记录

每次验收记录飞控固件版本/commit、机型、模式、完整相关参数、伴随计算机配置和对应日志。升级固件后重新检查消息字段与控制行为；文档与索引不能替代目标版本上的验收。

## v0.2.1 定制固件接收与控制实测

用户提供的FC2.0_S3.0.87_R_T在Loiter下接受四方向距离，开启避障停在理想墙前约1.03m；持续发送全未知后恢复前进并越墙。原始0cm会被忽略，正式编码器现将内部0映射为线上1cm并已复测。仅对此二进制成立，详见 `08_ArduPilot_SITL实测.md` 及原始日志；PX4尚未验证。
